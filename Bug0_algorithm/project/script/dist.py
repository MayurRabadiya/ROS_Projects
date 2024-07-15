import threading
import sys

class Dist:
    def __init__(self):
        """Initialize Dist class with thread lock and default sensor values."""
        self.m = threading.Lock()
        self.left = 0
        self.front = 0
        self.raw = []

    def update(self, data):
        """Update the distance measurements from LaserScan data."""
        def getmin(a, b):
            """Get the minimum valid range value between indices a and b."""
            in_rng = lambda x: data.range_min <= x <= data.range_max
            vsp = filter(in_rng, data.ranges[a:b])
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint

        # Extract the minimum distance values for front and left
        newfront = getmin(500, 581)
        newleft = getmin(740, 851)

        # Acquire the lock before updating shared variables
        self.m.acquire()
        self.left = newleft
        self.front = newfront
        self.raw = data
        self.m.release()

    def get(self):
        """Get the current front and left distance values."""
        self.m.acquire()
        l = self.left
        f = self.front
        self.m.release()
        return (f, l)

    def angle_to_index(self, angle):
        """Convert an angle to the corresponding index in the LaserScan ranges."""
        return int((angle - self.raw.angle_min) / self.raw.angle_increment)

    def at(self, angle):
        """Get the minimum valid range value around the specified angle."""
        def getmin(a, b):
            """Get the minimum valid range value between indices a and b."""
            in_rng = lambda x: self.raw.range_min <= x <= self.raw.range_max
            vsp = filter(in_rng, self.raw.ranges[a:b])
            if len(vsp) > 0:
                return min(vsp)
            else:
                return 0

        self.m.acquire()
        i = self.angle_to_index(angle)
        start = max(0, i - 100)
        end = min(len(self.raw.ranges) - 1, i + 100)
        ans = getmin(start, end)
        self.m.release()
        return ans
