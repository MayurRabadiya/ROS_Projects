import threading
import math
import sys

class Location:
    def __init__(self):
        """Initialize Location class with thread lock and default position values."""
        self.m = threading.Lock()
        self.x = None
        self.y = None
        self.t = None
        self.deltaT = 0.05  # Tolerance for heading comparison

    def update_location(self, x, y, t):
        """Update the current location and heading with new values."""
        self.m.acquire()
        self.x = x
        self.y = y
        self.t = t
        self.m.release()

    def current_location(self):
        """Get the current location and heading."""
        self.m.acquire()
        x = self.x
        y = self.y
        t = self.t
        self.m.release()
        return (x, y, t)

    def distance(self, x, y):
        """Calculate the Euclidean distance from the current location to the target (x, y)."""
        x0, y0, _ = self.current_location()
        if x0 is None or y0 is None:
            return sys.maxsize
        return math.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    def facing_point(self, x, y):
        """Check if the robot is facing the point (x, y) within a certain tolerance."""
        cx, cy, current_heading = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        n = necessary_heading(cx, cy, x, y)
        return n - self.deltaT <= current_heading <= n + self.deltaT

    def faster_left(self, x, y):
        """Determine if turning left is faster to face the point (x, y)."""
        cx, cy, current_heading = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        return current_heading - necessary_heading(cx, cy, x, y) < 0

    def global_to_local(self, desired_angle):
        """Convert a global desired angle to a local angle relative to the current heading."""
        _, _, current_heading = self.current_location()
        ans = desired_angle - current_heading
        if ans < -math.pi:
            ans += 2 * math.pi
        return ans

def necessary_heading(cx, cy, tx, ty):
    """Calculate the heading angle from current position (cx, cy) to target position (tx, ty)."""
    return math.atan2(ty - cy, tx - cx)
