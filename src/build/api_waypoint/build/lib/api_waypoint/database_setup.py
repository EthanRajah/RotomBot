import sqlite3
import os
from datetime import datetime


class DroneDatabase:
    def __init__(self, db_path="drone_data.db", image_dir="drone_images"):
        """Initialize the drone database connection."""
        self.db_path = db_path
        self.image_dir = image_dir

        # Create image directory if it doesn't exist
        os.makedirs(self.image_dir, exist_ok=True)

        # Initialize database and create tables if they don't exist
        self._init_database()

    def _init_database(self):
        """Create the SQLite database table if it doesn't exist."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        # Create a single drone_data table
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS building_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            date DATE,
            time TIME,
            building TEXT NOT NULL,
            status TEXT,
            image_path TEXT,
            vlm_description TEXT,
            manual_inspection_needed BOOLEAN DEFAULT 0,
            updated_at TEXT
        )
        ''')

        conn.commit()
        conn.close()

    def add_building_event(self, building, status=None, image_data=None,
                           vlm_description=None, manual_inspection=False):
        """Add a building event - can be a status update, detection, or both.

        Args:
            building: Building identifier (A, B, C, etc.)
            status: Status of the building (sent, en_route, arrived, completed)
            image_data: Binary image data (optional)
            vlm_description: clm Description of any issue found (optional)
            manual_inspection: Flag if manual inspection is needed (default: False)

        Returns:
            id: The ID of the newly inserted record
        """
        now = datetime.now()
        date_str = now.strftime('%Y-%m-%d')
        time_str = now.strftime('%H:%M:%S')
        timestamp = now.strftime('%Y-%m-%d %H:%M:%S')

        # Save image if provided
        image_path = None
        if image_data:
            # Create date-based directory for images
            date_dir = os.path.join(self.image_dir, date_str)
            os.makedirs(date_dir, exist_ok=True)

            # Save image to file
            image_filename = f"{now.strftime('%Y%m%d_%H%M%S')}_building_{building}.jpg"
            image_path = os.path.join(date_dir, image_filename)

            with open(image_path, 'wb') as f:
                f.write(image_data)

        # Save record to database
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute('''
        INSERT INTO building_events
            (date, time, building, status, image_path, 
             vlm_description, manual_inspection_needed)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (
            date_str, time_str, building, status, image_path,
            vlm_description, 1 if manual_inspection else 0
        ))

        last_id = cursor.lastrowid
        conn.commit()
        conn.close()

        return last_id

    def update_event(self, event_id, status=None, vlm_description=None,
                     manual_inspection=None, image_data=None):
        """Update an existing building event.

        Args:
            event_id: ID of the event to update
            status: Updated status (optional)
            vlm_description: Updated vlm_description (optional)
            manual_inspection: Updated inspection flag (optional)
            image_data: New image data (optional)

        Returns:
            bool: Success status
        """
        now = datetime.now()
        timestamp = now.strftime('%Y-%m-%d %H:%M:%S')

        # Get current event data
        current = self.get_event(event_id)
        if not current:
            return False

        # Prepare update values
        update_values = []
        update_params = []

        # Update timestamp
        update_values.append("updated_at = ?")
        update_params.append(timestamp)

        # Handle optional updates
        if status is not None:
            update_values.append("status = ?")
            update_params.append(status)

        if vlm_description is not None:
            update_values.append("vlm_description = ?")
            update_params.append(vlm_description)

        if manual_inspection is not None:
            update_values.append("manual_inspection_needed = ?")
            update_params.append(1 if manual_inspection else 0)

        # Handle image update if provided
        if image_data:
            # Create date-based directory for images
            date_str = now.strftime('%Y-%m-%d')
            date_dir = os.path.join(self.image_dir, date_str)
            os.makedirs(date_dir, exist_ok=True)

            # Save image to file
            image_filename = f"{now.strftime('%Y%m%d_%H%M%S')}_building_{current['building']}.jpg"
            image_path = os.path.join(date_dir, image_filename)

            with open(image_path, 'wb') as f:
                f.write(image_data)

            update_values.append("image_path = ?")
            update_params.append(image_path)

        # Execute update
        if update_values:
            update_params.append(event_id)  # Add event_id for WHERE clause

            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            try:
                query = f"UPDATE building_events SET {', '.join(update_values)} WHERE id = ?"
                cursor.execute(query, update_params)
                conn.commit()
                conn.close()
                return True
            except Exception as e:
                conn.close()
                print(f"Error updating event: {e}")
                return False

        return False  # No updates specified

    def get_latest_event_by_building(self, building):
        """Get the latest event for a specific building.

        Args:
            building: Building identifier

        Returns:
            dict: The latest event for the building or None if not found
        """
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        try:
            cursor.execute('''
            SELECT * FROM building_events 
            WHERE building = ?
            ORDER BY updated_at DESC
            LIMIT 1
            ''', (building,))

            row = cursor.fetchone()
            conn.close()

            if row:
                return dict(row)
            return None
        except Exception as e:
            conn.close()
            print(f"Error getting latest event: {e}")
            return None

    def get_all_events(self):
        """Get all events.

        Returns:
            list: List of all events sorted by most recent first
        """
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        try:
            cursor.execute('''
            SELECT * FROM building_events 
            ORDER BY updated_at DESC
            ''')

            rows = cursor.fetchall()
            result = [dict(row) for row in rows]
            conn.close()
            return result
        except Exception as e:
            conn.close()
            print(f"Error getting events: {e}")
            return []

    def get_building_events(self, building=None):
        """Get events for a specific building.

        Args:
            building: Building identifier (optional)

        Returns:
            list: List of events for the building
        """
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        try:
            if building:
                cursor.execute('''
                SELECT * FROM building_events 
                WHERE building = ?
                ORDER BY updated_at DESC
                ''', (building,))
            else:
                cursor.execute('''
                SELECT * FROM building_events 
                ORDER BY building, updated_at DESC
                ''')

            rows = cursor.fetchall()
            result = [dict(row) for row in rows]
            conn.close()
            return result
        except Exception as e:
            conn.close()
            print(f"Error getting building events: {e}")
            return []

    def get_event(self, event_id):
        """Retrieve a specific event by ID.

        Args:
            event_id: The ID of the event to retrieve

        Returns:
            dict: Event data or None if not found
        """
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute('''
        SELECT * FROM building_events WHERE id = ?
        ''', (event_id,))

        row = cursor.fetchone()
        conn.close()

        if row:
            return dict(row)
        return None