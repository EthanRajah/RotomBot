import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForImageTextToText
import sqlite3
import time
import os
import logging
import sys
from datetime import datetime

# MLX specific imports
import mlx.core as mx
from mlx_vlm import load, generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("vlm_processor.log"),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger("VLMProcessor")

# Configuration
DB_PATH = "drone_data.db"
IMAGE_BASE_DIR = "drone_images"
CHECK_INTERVAL_SECONDS = 5
MODEL_PATH = "mlx-community/SmolVLM2-2.2B-Instruct-mlx"
# MODEL_PATH = "HuggingFaceTB/SmolVLM2-2.2B-Instruct"

# Make sure we're in the correct directory for relative paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(BASE_DIR)

def get_pending_detections_with_details():
    """
    Get detections that need VLM processing along with all details
    needed to construct the correct image path.
    """
    try:
        conn = sqlite3.connect(DB_PATH)
        # Enable row factory to get column names
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute('''
            SELECT id, building, date AS detection_date, time AS detection_time, image_path 
            FROM building_events
            WHERE vlm_description IS NULL OR vlm_description = ''
            ORDER BY date ASC, time ASC
        ''')

        # Convert to list of dictionaries for easier access
        results = [dict(row) for row in cursor.fetchall()]
        conn.close()
        return results
    except Exception as e:
        logger.error(f"Database error: {str(e)}")
        return []

def construct_image_path(detection):
    """
    Construct the correct image path based on database information.
    This uses date and waypoint to find the image.
    """
    try:
        # Get date from detection in YYYY-MM-DD format
        date_str = detection['detection_date']
        waypoint = detection['building']

        # Date directory
        date_dir = os.path.join(IMAGE_BASE_DIR, date_str)

        # If the exact path exists, use it
        if os.path.exists(detection['image_path']):
            logger.info(f"Using exact image path from database: {detection['image_path']}")
            return detection['image_path']

        # If date directory doesn't exist, we have a problem
        if not os.path.exists(date_dir):
            logger.error(f"Date directory not found: {date_dir}")
            return None

        # List all images in the date directory
        files = os.listdir(date_dir)

        # Find images that match this waypoint (format: YYYYMMDD_HHMMSS_waypoint_X.jpg)
        building_files = [f for f in files if f.endswith(f"building_{waypoint}.jpg")]

        if not building_files:
            # Try looser matching - any file containing the waypoint number
            building_files = [f for f in files if f"building_{waypoint}" in f]

        if building_files:
            # If multiple matches, use the closest to the detection time
            if len(building_files) > 1:
                # Parse the detection time (HH:MM:SS)
                det_time = datetime.strptime(detection['detection_time'], "%H:%M:%S")

                # Find the file with the closest timestamp
                best_match = None
                min_diff = float('inf')

                for file in building_files:
                    # Extract time from filename (format: YYYYMMDD_HHMMSS_waypoint_X.jpg)
                    try:
                        # Split by underscore and get the time part
                        time_part = file.split('_')[1]
                        file_time = datetime.strptime(time_part, "%H%M%S")

                        # Calculate time difference in seconds
                        time_diff = abs((datetime.combine(datetime.today(), det_time.time()) -
                                         datetime.combine(datetime.today(), file_time.time())).total_seconds())

                        if time_diff < min_diff:
                            min_diff = time_diff
                            best_match = file
                    except:
                        # If parsing fails, just continue to the next file
                        continue

                # Use the best match if found
                if best_match:
                    return os.path.join(date_dir, best_match)
                else:
                    # If all parsing failed, just use the first file
                    return os.path.join(date_dir, building_files[0])
            else:
                # Just one match, use it
                return os.path.join(date_dir, building_files[0])

        # If no match by waypoint, use any image from that day as a fallback
        if files:
            logger.warning(f"No specific waypoint match for waypoint {waypoint}. Using first image in date directory.")
            return os.path.join(date_dir, files[0])

        # No images found
        logger.error(f"No images found in date directory: {date_dir}")
        return None

    except Exception as e:
        logger.info(f"Can't construct image path. No image found for entry.")
        return None

def update_detection(detection_id, vlm_description):
    """Update detection record with VLM description."""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()

        # Set manual inspection flag if "severe" is in the description
        needs_inspection = 1 if ("severe" in vlm_description.lower() or
                                 "critical" in vlm_description.lower()) else 0

        cursor.execute('''
            UPDATE building_events
            SET vlm_description = ?, manual_inspection_needed = ?
            WHERE id = ?
        ''', (vlm_description, needs_inspection, detection_id))

        conn.commit()
        conn.close()
        logger.info(f"Updated detection {detection_id}: Manual inspection = {needs_inspection}")
        return True
    except Exception as e:
        logger.error(f"Error updating database: {str(e)}")
        return False

def process_image_with_vlm(image_path, model, processor, config):
    """Process an image using the VLM model."""
    try:
        # Define the prompt
        prompt = """You are an expert in structural analysis. Make sure to follow the format strictly. If the image is not a structural crack, state 'None' in each section. Otherwise, carefully analyze the structural crack image and provide insights in the following structured format. Give bullet responses for each section:
- Type of Defect: Describe the defect or anomaly present in the image\n
- Severity Level: Estimate the severity as a Minor, Moderate, or Severe class\n
- Potential Causes: Explain what could have led to the defect\n
- Recommended Actions: Suggest appropriate mitigation steps\n"""

        # Apply the chat template
        formatted_prompt = apply_chat_template(
            processor, config, prompt, num_images=1
        )

        # Generate the output
        output = generate(model, processor, formatted_prompt, [image_path], verbose=False)
        post_processed_output = postprocess_vlm_output(output)
        return post_processed_output

    except Exception as e:
        print(f"Error in VLM processing: {str(e)}")
        return f"Error processing image: {str(e)}"
    
def postprocess_vlm_output(output):
    """Post process the VLM output to follow the required format."""

    # Split the output into sections
    section_headers = ["Type of Defect:", "Severity Level:", "Potential Causes:", "Recommended Actions:"]
    defect = output.split(section_headers[0])[1].split(section_headers[1])[0].strip()
    severity = output.split(section_headers[1])[1].split(section_headers[2])[0].strip()
    causes = output.split(section_headers[2])[1].split(section_headers[3])[0].strip()
    actions = output.split(section_headers[3])[1].strip()

    # If defect is "None" replace vlm output with "No structural defect found"
    if defect == "None":
        return "No structural defect found."

    # Combine into a set of four bullet points based on the sections
    vlm_description = (
    f"<ul>"
    f"<li><strong>{section_headers[0]}</strong> {defect}</li>"
    f"<li><strong>{section_headers[1]}</strong> {severity}</li>"
    f"<li><strong>{section_headers[2]}</strong> {causes}</li>"
    f"<li><strong>{section_headers[3]}</strong> {actions}</li>"
    f"</ul>"
    )
    return vlm_description


def main():
    """Main function to run the VLM processor."""
    # /opt/anaconda3/envs/dev/bin/python vlm-processor.py
    logger.info("Starting VLM processor")

    # Load the VLM model
    # try:
    #     logger.info(f"Loading model: {MODEL_PATH}")
    #     processor = AutoProcessor.from_pretrained(MODEL_PATH)
    #     model = AutoModelForImageTextToText.from_pretrained(
    #         MODEL_PATH,
    #         torch_dtype=torch.float16 if torch.backends.mps.is_available() else torch.float32,
    #         device_map="auto"
    #     )
    #     logger.info("Model loaded successfully")
    # except Exception as e:
    #     logger.error(f"Failed to load model: {str(e)}")
    #     return
    # Load the model and processor
    model, processor = load(MODEL_PATH)
    config = load_config(MODEL_PATH)

    # Main processing loop
    try:
        while True:
            # Check for pending detections
            detections = get_pending_detections_with_details()

            if detections:
                logger.info(f"Found {len(detections)} detections to process")

                for detection in detections:
                    detection_id = detection['id']
                    logger.info(f"Processing detection {detection_id} for building {detection['building']}")

                    # Construct image path based on database information
                    image_path = construct_image_path(detection)

                    if not image_path or not os.path.exists(image_path):
                        continue

                    # Process the image
                    logger.info(f"Processing image: {image_path}")
                    description = process_image_with_vlm(image_path, model, processor, config)

                    # Update the database
                    update_detection(detection_id, description)
                    logger.info(f"Completed processing for detection {detection_id}")
            else:
                logger.info("No pending detections found")

            # Wait before checking again
            logger.info(f"Waiting {CHECK_INTERVAL_SECONDS} seconds...")
            time.sleep(CHECK_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        logger.info("VLM processor stopped by user")
        # Clear cache and exit
        
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")


if __name__ == "__main__":
    main()