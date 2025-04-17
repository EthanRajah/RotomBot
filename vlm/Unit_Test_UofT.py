import requests
import base64
import argparse
import json
import os


class DroneAPIClient:
    """Client for sending crack detection data to the API from one device to another."""

    def __init__(self, api_url):
        """Initialize the client.

        Args:
            api_url: Base URL of the API server (e.g., http://192.168.1.100:5070)
        """
        self.api_url = api_url
        self.detection_endpoint = f"{api_url}/api/detection"
        self.detections_endpoint = f"{api_url}/api/detections"

    def send_image_and_text(self, image_path, waypoint, description=None, manual_inspection=False):
        """Send an image file with text description to the server.

        Args:
            image_path: Path to the image file to send
            waypoint: Waypoint number
            description: Text description of the crack
            manual_inspection: Whether manual inspection is needed

        Returns:
            dict: API response
        """
        # Check if image exists
        if not os.path.exists(image_path):
            return {
                "status_code": 0,
                "error": f"Image file not found: {image_path}"
            }

        # Read image file
        with open(image_path, 'rb') as img_file:
            image_data = img_file.read()

        # Prepare payload
        payload = {
            "waypoint": waypoint,
            "image": base64.b64encode(image_data).decode('utf-8'),
            "vlm_description": description or "",
            "manual_inspection_needed": manual_inspection
        }

        # Send request to API
        try:
            response = requests.post(
                self.detection_endpoint,
                json=payload,
                headers={"Content-Type": "application/json"},
                timeout=30  # Longer timeout for large images
            )

            return {
                "status_code": response.status_code,
                "response": response.json() if response.status_code in (200, 201) else response.text,
                "sent_data": {
                    "waypoint": waypoint,
                    #"vlm_description": description,
                    "manual_inspection_needed": manual_inspection,
                    "image_size_bytes": len(image_data),
                    "image_path": image_path
                }
            }
        except requests.exceptions.RequestException as e:
            return {
                "status_code": 0,
                "error": str(e)
            }

    def get_all_detections(self):
        """Retrieve all detections from the server.

        Returns:
            dict: API response with detections
        """
        try:
            response = requests.get(self.detections_endpoint)
            return {
                "status_code": response.status_code,
                "detections": response.json().get("detections", []) if response.status_code == 200 else []
            }
        except requests.exceptions.RequestException as e:
            return {
                "status_code": 0,
                "error": str(e),
                "detections": []
            }


def main():
    """Main function to run the client."""
    # python Unit_Test_UofT.py --server 100.66.208.226:5081 --image andrew.jpeg --waypoint 69
    parser = argparse.ArgumentParser(description='Send crack detection data to API server')
    parser.add_argument('--server', required=True, help='Server address (e.g., 192.168.1.100:5069)')
    parser.add_argument('--image', required=True, help='Path to image file')
    parser.add_argument('--waypoint', type=int, required=True, help='Waypoint number')
    parser.add_argument('--description', help='Description of the detection')
    parser.add_argument('--manual', action='store_true', help='Flag for manual inspection needed')
    parser.add_argument('--view', action='store_true', help='Open web view after sending')

    args = parser.parse_args()

    # Format the server URL
    if not args.server.startswith(('http://', 'https://')):
        server_url = f"http://{args.server}"
    else:
        server_url = args.server

    # Create client and send detection
    client = DroneAPIClient(server_url)

    print(f"Sending image to {server_url}...")
    result = client.send_image_and_text(
        image_path=args.image,
        waypoint=args.waypoint,
        description=args.description,
        manual_inspection=args.manual
    )

    # Display results
    if result.get("status_code") in (200, 201):
        print("\nSuccess! Data sent to server.")
        print(f"Status code: {result['status_code']}")
        print(f"Response: {json.dumps(result['response'], indent=2)}")
    else:
        print("\n Error sending data to server.")
        if "error" in result:
            print(f"Error: {result['error']}")
        else:
            print(f"Status code: {result['status_code']}")
            print(f"Response: {result['response']}")

    if args.view:
        view_url = f"{server_url}/view"
        print(f"\nView results in your browser at: {view_url}")


if __name__ == "__main__":
    main()