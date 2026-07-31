import math

class PIDSteeringController:
    """
    Assigned Task Module: Translates A* waypoints into live motor speed 
    adjustments (left_speed and right_speed) using proportional control and 
    heading normalization.
    """
    def __init__(self, kp=3.0, max_speed=6.0, waypoint_tolerance=0.3):
        self.kp = kp
        self.max_speed = max_speed
        self.waypoint_tolerance = waypoint_tolerance

    def calculate_speeds(self, current_x, current_y, current_heading, current_path):
        """
        Takes current robot pose and the active A* path array.
        Mutates the path array (pops reached waypoints) and returns:
        (left_speed, right_speed)
        """
        # 1. If path is empty, halt the motors
        if not current_path or len(current_path) == 0:
            return 0.0, 0.0

        # 2. Target the next immediate waypoint
        target_x, target_y = current_path[0]

        # 3. Delta distance calculation
        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)

        # 4. Check if waypoint is reached
        if dist < self.waypoint_tolerance:
            current_path.pop(0)  # Remove reached waypoint
            return 0.0, 0.0

        # 5. Heading Error & Normalization (-PI to PI)
        target_angle = math.atan2(dy, dx)
        angle_error = (target_angle - current_heading + math.pi) % (2 * math.pi) - math.pi

        # 6. Proportional Steering Command
        turn_speed = max(-self.max_speed, min(self.max_speed, angle_error * self.kp))

        # 7. Pivot-and-Drive Forward Speed Logic
        # If off-course by more than ~28 degrees, stop forward momentum and pivot in place
        forward_speed = 0.0 if abs(angle_error) > 0.5 else self.max_speed * 0.8

        # 8. Individual Wheel Velocities
        left_speed = forward_speed - turn_speed
        right_speed = forward_speed + turn_speed

        # Final safety limits
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        return left_speed, right_speed