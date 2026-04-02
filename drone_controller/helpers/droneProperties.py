class droneProperties:
    
    
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.is_armed = False
        self.pose = [0.0, 0.0, 0.0]  # (latitude, longitude, altitude)
        self.status = "idle"
        self.last_update = None
        
    def get_id(self):
        """Get the drone's unique identifier."""
        return self.drone_id

    
    def set_armed(self, armed):
        """Set drone armed status."""
        self.is_armed = bool(armed)
    
    def get_armed(self):
        """Get drone armed status."""
        return self.is_armed
    
    def set_position(self, latitude, longitude, altitude):
        """Set drone GPS position and altitude."""
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
    
    def get_position(self):
        """Get drone position as tuple (lat, lon, alt)."""
        return (self.latitude, self.longitude, self.altitude)
    
    def set_orientation(self, heading, speed):
        """Set drone heading and speed."""
        self.heading = heading % 360
        self.speed = max(0, speed)
    
    def get_orientation(self):
        """Get heading and speed as tuple."""
        return (self.heading, self.speed)
    
    def set_status(self, status):
        """Set drone operational status."""
        self.status = status.lower()
    
    def get_status(self):
        """Get drone status."""
        return self.status
    
    def get_info(self):
        """Get all drone information as dictionary."""
        return {
            "drone_id": self.drone_id,
            "battery": self.battery_level,
            "armed": self.is_armed,
            "position": (self.latitude, self.longitude, self.altitude),
            "heading": self.heading,
            "speed": self.speed,
            "status": self.status
        }