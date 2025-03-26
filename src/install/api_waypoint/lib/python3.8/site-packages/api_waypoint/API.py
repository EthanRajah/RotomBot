from flask import Flask, request, jsonify, send_file
import base64
import os
from database_setup import DroneDatabase
import sqlite3

app = Flask(__name__)
db = DroneDatabase(db_path="drone_data.db", image_dir="drone_images")


@app.route('/api/building/event', methods=['POST'])
def add_building_event():
    """Endpoint to receive any building event - status update or detection."""
    try:
        # Get JSON data from request
        data = request.json

        # Validate required field
        if 'building' not in data:
            return jsonify({"error": "Missing required field: building"}), 400

        # Extract data from request
        building = data['building']
        status = data.get('status')
        vlm_description = data.get('vlm_description', '')
        manual_inspection = bool(data.get('manual_inspection_needed', False))

        # Process image if present
        image_data = None
        if 'image' in data:
            try:
                image_data = base64.b64decode(data['image'])
            except Exception as e:
                return jsonify({"error": f"Invalid image data: {str(e)}"}), 400

        # Add event to database
        event_id = db.add_building_event(
            building=building,
            status=status,
            image_data=image_data,
            vlm_description=vlm_description,
            manual_inspection=manual_inspection
        )

        return jsonify({
            "success": True,
            "message": "Building event recorded successfully",
            "event_id": event_id
        }), 201

    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/building/event/<int:event_id>', methods=['PUT'])
def update_building_event(event_id):
    """Endpoint to update an existing building event."""
    try:
        # Get JSON data from request
        data = request.json

        # Extract data from request
        status = data.get('status')
        vlm_description = data.get('vlm_description')
        manual_inspection = data.get('manual_inspection_needed')

        # Process image if present
        image_data = None
        if 'image' in data:
            try:
                image_data = base64.b64decode(data['image'])
            except Exception as e:
                return jsonify({"error": f"Invalid image data: {str(e)}"}), 400

        # Update event in database
        success = db.update_event(
            event_id=event_id,
            status=status,
            vlm_description=vlm_description,
            manual_inspection=manual_inspection,
            image_data=image_data
        )

        if success:
            return jsonify({
                "success": True,
                "message": f"Event {event_id} updated successfully"
            }), 200
        else:
            return jsonify({"error": f"Event {event_id} not found or no updates specified"}), 404

    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/events', methods=['GET'])
def get_all_events():
    """Endpoint to retrieve all events."""
    try:
        events = db.get_all_events()
        return jsonify({"events": events}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/building/events', methods=['GET'])
def get_building_events():
    """Endpoint to get events for a specific building."""
    try:
        building = request.args.get('building')
        events = db.get_building_events(building)
        return jsonify({"events": events}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/building/latest', methods=['GET'])
def get_latest_building_event():
    """Endpoint to get the latest event for a building."""
    try:
        building = request.args.get('building')
        if not building:
            return jsonify({"error": "Missing required parameter: building"}), 400

        event = db.get_latest_event_by_building(building)
        if event:
            return jsonify({"event": event}), 200
        else:
            return jsonify({"error": f"No events found for building {building}"}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/event/<int:event_id>', methods=['GET'])
def get_event(event_id):
    """Endpoint to retrieve a specific event."""
    try:
        event = db.get_event(event_id)
        if event:
            return jsonify({"event": event}), 200
        else:
            return jsonify({"error": "Event not found"}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/', methods=['GET'])
@app.route('/view', methods=['GET'])
def view_all():
    """Web interface to view all building events."""
    # Get all events
    all_events = db.get_all_events()

    html = '''
    <html>
    <head>
        <title>Drone Monitoring System</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            h1, h2 { color: #333; }
            table { border-collapse: collapse; width: 100%; margin-top: 10px; margin-bottom: 30px; }
            th { background-color: #f2f2f2; padding: 10px; text-align: left; }
            td { padding: 10px; border: 1px solid #ddd; }
            .image-cell { text-align: center; }
            .crack-image { max-width: 200px; max-height: 150px; border: 1px solid #ccc; }
            .severe { color: #d9534f; font-weight: bold; }
            .moderate { color: #f0ad4e; }
            .minor { color: #5bc0de; }
            .sent { color: #5bc0de; }
            .en_route { color: #f0ad4e; }
            .arrived { color: #5cb85c; }
            .completed { color: #5cb85c; font-weight: bold; }
            .update-form { display: none; background: #f9f9f9; padding: 15px; border: 1px solid #ddd; margin-top: 5px; }
            .update-button { background: #4CAF50; color: white; border: none; padding: 5px 10px; cursor: pointer; }
            .cancel-button { background: #f44336; color: white; border: none; padding: 5px 10px; cursor: pointer; }
        </style>
        <script>
            function showUpdateForm(id) {
                document.getElementById('form-' + id).style.display = 'block';
            }

            function hideUpdateForm(id) {
                document.getElementById('form-' + id).style.display = 'none';
            }

            async function updateEvent(id) {
                const status = document.getElementById('status-' + id).value;

                // Create request body
                const data = {};
                if (status) data.status = status;

                try {
                    const response = await fetch('/api/building/event/' + id, {
                        method: 'PUT',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify(data)
                    });

                    if (response.ok) {
                        alert('Event updated successfully!');
                        location.reload();
                    } else {
                        const error = await response.json();
                        alert('Error: ' + error.error);
                    }
                } catch (error) {
                    alert('Error updating event: ' + error);
                }
            }
        </script>
    </head>
    <body>
        <h1>Drone Monitoring System</h1>

        <!-- Single Consolidated Table -->
        <div>
            <h2>Building Events</h2>
            <table>
                <tr>
                    <th>ID</th>
                    <th>Date</th>
                    <th>Time</th>
                    <th>Building</th>
                    <th>Status</th>
                    <th>Image</th>
                    <th>Vlm Description</th>
                    <th>Manual Inspection</th>
                </tr>
    '''

    if not all_events:
        html += '''
        <tr>
            <td colspan="10" style="text-align: center;">No events recorded yet</td>
        </tr>
        '''
    else:
        for event in all_events:
            status_class = event.get("status", "").replace('-', '_') if event.get("status") else ""
            event_id = event.get("id", "")

            # Format the row
            html += f'''
            <tr>
                <td>{event_id}</td>
                <td>{event.get("date", "")}</td>
                <td>{event.get("time", "")}</td>
                <td>{event.get("building", "")}</td>
                <td class="{status_class}">{event.get("status", "").replace('_', ' ').title() if event.get("status") else ""}</td>
            '''

            # Image cell
            if event.get("image_path"):
                image_path = event["image_path"].replace('\\', '/')
                html += f'''
                <td class="image-cell">
                    <a href="/images/{image_path}" target="_blank">
                        <img src="/images/{image_path}" class="crack-image" alt="Building {event.get('building', '')}">
                    </a>
                </td>
                '''
            else:
                html += '<td></td>'  # Empty image cell

            # VLM Description
            vlm_description = event.get("vlm_description", "")
            severity_class = ""
            if vlm_description:
                vlm_description_lower = vlm_description.lower()
                if "severe" in vlm_description_lower:
                    severity_class = "severe"
                elif "moderate" in vlm_description_lower:
                    severity_class = "moderate"
                elif "minor" in vlm_description_lower:
                    severity_class = "minor"

            html += f'''
                <td class="{severity_class}">{vlm_description}</td>
                <td>{"Yes" if event.get("manual_inspection_needed", 0) == 1 else ""}</td>
            </tr>
            '''

    html += '''
            </table>
        </div>
    </body>
    </html>
    '''

    return html


@app.route('/images/<path:filename>')
def serve_image(filename):
    """Serve images from the image directory."""
    return send_file(filename)

@app.route('/api/next_waypoints', methods=['GET'])
def get_next_waypoints():
    """Send a list of buildings (waypoints) for the drone to visit."""
    try:
        # Example logic: get buildings where the latest event is not "completed"
        conn = sqlite3.connect(db.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute('''
            SELECT building, MAX(id) as latest_id
            FROM building_events
            GROUP BY building
        ''')
        rows = cursor.fetchall()

        buildings_to_visit = []
        for row in rows:
            cursor.execute('SELECT status FROM building_events WHERE id = ?', (row['latest_id'],))
            status = cursor.fetchone()['status']
            if status != "en_route":
                buildings_to_visit.append(row['building'])

        conn.close()
        return jsonify({"waypoints": buildings_to_visit}), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/latest_events', methods=['GET'])
def get_latest_events():
    """Return latest event IDs per building (for status updates)."""
    try:
        conn = sqlite3.connect(db.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute('''
            SELECT building, MAX(id) AS latest_event_id
            FROM building_events
            GROUP BY building
        ''')

        events = {row['building']: row['latest_event_id'] for row in cursor.fetchall()}
        conn.close()

        return jsonify({"latest_events": events}), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    # /opt/anaconda3/envs/dev/bin/python /Users/ethan/Documents/Robotics/ROB498/vlm/API.py
    app.run(host='0.0.0.0', port=5072, debug=True)