import requests

# Step 1: Get waypoints
local = "http://10.42.0.103:5072"
waypoints_resp = requests.get(f"{local}/api/next_waypoints")
waypoints = waypoints_resp.json().get("waypoints", [])

# Step 2: Get latest event IDs per building
event_ids_resp = requests.get(f"{local}/api/latest_events")
event_map = event_ids_resp.json().get("latest_events", {})

# Step 3: Update each to "en_route"
for building in waypoints:
    event_id = event_map.get(building)
    if event_id:
        print(f"Marking building {building} (event {event_id}) as en_route")
        update_resp = requests.put(
            f"{local}/api/building/event/{event_id}",
            json={"status": "en_route"}
        )
        if update_resp.ok:
            print(f"Building {building} marked as en_route")
        else:
            print(f"Failed to update {building}: {update_resp.text}")
