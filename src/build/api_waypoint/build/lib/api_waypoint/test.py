import requests
# Step 1: Get waypoints
waypoints_resp = requests.get("http://10.42.0.103:5072/api/next_waypoints")
waypoints = waypoints_resp.json().get("waypoints", [])
print("IM GONNA BUSH")
# Step 2: Get latest event IDs per building
event_ids_resp = requests.get("http://10.42.0.103:5072/api/latest_events")
event_map = event_ids_resp.json().get("latest_events", {})

# Step 3: Update each to "en_route"
for building in waypoints:
    event_id = event_map.get(building)
    if event_id:
        # Get the current status of the event
        event_resp = requests.get(f"http://10.42.0.103:5072/api/event/{event_id}")

        if not event_resp.ok:
            print(f"Failed to get event for building {building}: {event_resp.text}")
            continue

        event_data = event_resp.json().get("event", {})
        current_status = event_data.get("status")

        # Only update if status is "sent"
        if current_status == "sent":
            print(f"Marking building {building} (event {event_id}) as en_route")
            update_resp = requests.put(
                f"http://10.42.0.103:5072/api/building/event/{event_id}",
                json={"status": "en_route"}
            )
            if update_resp.ok:
                print(f"Building {building} marked as en_route")
            else:
                print(f"Failed to update {building}: {update_resp.text}")
        else:
            print(f"Skipping building {building} with status '{current_status}'")
    else:
        print(f"No event ID found for building {building}")