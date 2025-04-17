import requests
import base64

# --- Config ---
SERVER = "http://10.42.0.101:5072"
IMAGE_MAP = {
    "A": r"/Users/ethan/Documents/Robotics/ROB498/vlm/cracking.jpg",
    "B": r"/Users/ethan/Documents/Robotics/ROB498/vlm/cracks.jpeg",
}

# --- Step 1: Get waypoints ---
waypoints_resp = requests.get(f"{SERVER}/api/next_waypoints")
waypoints = waypoints_resp.json().get("waypoints", [])

# --- Step 2: Get latest event IDs per building ---
event_ids_resp = requests.get(f"{SERVER}/api/latest_events")
event_map = event_ids_resp.json().get("latest_events", {})

# --- Step 4: Send back inspection images & mark as completed ---
for building, image_path in IMAGE_MAP.items():
    event_id = event_map.get(building)
    if event_id:
        try:
            with open(image_path, "rb") as img_file:
                encoded_image = base64.b64encode(img_file.read()).decode("utf-8")

            payload = {
                "status": "completed",
                "description": f"Drone inspection completed for Building {building}.",
                "image": encoded_image
            }

            complete_resp = requests.put(
                f"{SERVER}/api/building/event/{event_id}",
                json=payload
            )

            if complete_resp.ok:
                print(f"[✔] Uploaded inspection image for Building {building} (event {event_id})")
            else:
                print(f"[✖] Failed to upload image for {building}: {complete_resp.text}")

        except Exception as e:
            print(f"[!!] Error reading or sending image for {building}: {e}")
