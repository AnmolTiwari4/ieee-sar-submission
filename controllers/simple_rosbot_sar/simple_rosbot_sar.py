"""
Basic Rosbot Controller for Search and Rescue Simulation
This controller implements a simple exploration strategy
"""

from controller import Robot
import json
import random
import math
from typing import Tuple, List


class BasicRosbotController:
    def __init__(self):
        """Initialize the robot controller"""
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Get robot name for identification
        self.robot_id = self.robot.getName()

        # Initialize sensors and actuators
        self._init_devices()

        # Robot state and navigation
        self.last_decision_time = 0
        self.decision_interval = 5.0  # seconds between decisions
        self.action_pending = True
        self.action_fun = self.move_forward

        # Victim detection
        self.victim_confidence_threshold = 0.7

        # Navigation constants
        self.max_speed = 5.0
        self.obstacle_threshold = 0.5  # distance sensor threshold

        print(f"[{self.robot_id}] Initialized - Ready for search and rescue mission")

    def _init_devices(self):
        """Initialize robot sensors and actuators"""
        # Motors
        self.front_left_motor = self.robot.getDevice("fl_wheel_joint")
        self.front_right_motor = self.robot.getDevice("fr_wheel_joint")
        self.rear_left_motor = self.robot.getDevice("rl_wheel_joint")
        self.rear_right_motor = self.robot.getDevice("rr_wheel_joint")
        self.front_left_motor.setPosition(float("inf"))
        self.front_right_motor.setPosition(float("inf"))
        self.rear_left_motor.setPosition(float("inf"))
        self.rear_right_motor.setPosition(float("inf"))
        self.front_left_motor.setVelocity(0)
        self.front_right_motor.setVelocity(0)
        self.rear_left_motor.setVelocity(0)
        self.rear_right_motor.setVelocity(0)

        # Wheel position sensors
        self.front_left_position_sensor = self.robot.getDevice(
            "front left wheel motor sensor"
        )
        self.front_right_position_sensor = self.robot.getDevice(
            "front right wheel motor sensor"
        )
        self.rear_left_position_sensor = self.robot.getDevice(
            "rear left wheel motor sensor"
        )
        self.rear_right_position_sensor = self.robot.getDevice(
            "rear right wheel motor sensor"
        )
        self.front_left_position_sensor.enable(self.timestep)
        self.front_right_position_sensor.enable(self.timestep)
        self.rear_left_position_sensor.enable(self.timestep)
        self.rear_right_position_sensor.enable(self.timestep)

        # RGB camera
        try:
            self.camera_rgb = self.robot.getDevice("camera rgb")
            self.camera_rgb.enable(self.timestep)
        except:
            self.camera_rgb = None
            print(f"[{self.robot_id}] Warning: No RGB camera found")

        # Depth camera
        try:
            self.camera_depth = self.robot.getDevice("camera depth")
            self.camera_depth.enable(self.timestep)
        except:
            self.camera_depth = None
            print(f"[{self.robot_id}] Warning: No depth camera found")

        # Lidar sensor
        try:
            self.lidar = self.robot.getDevice("laser")
            self.lidar.enable(self.timestep)
        except:
            self.lidar = None
            print(f"[{self.robot_id}] Warning: No lidar found")

        # Accelerometer
        try:
            self.accelerometer = self.robot.getDevice("imu accelerometer")
            self.accelerometer.enable(self.timestep)
        except:
            print(f"[{self.robot_id}] Warning: No accelerometer found")
            self.accelerometer = None

        # Gyro
        try:
            self.gyro = self.robot.getDevice("imu gyro")
            self.gyro.enable(self.timestep)
        except:
            print(f"[{self.robot_id}] Warning: No gyro found")
            self.gyro = None

        # Compass
        try:
            self.compass = self.robot.getDevice("imu compass")
            self.compass.enable(self.timestep)
        except:
            print(f"[{self.robot_id}] Warning: No compass found")
            self.compass = None

        # Distance sensors
        self.distance_sensors = []
        sensor_names = ["fl_range", "fr_range", "rl_range", "rr_range"]
        for name in sensor_names:
            try:
                sensor = self.robot.getDevice(name)
                sensor.enable(self.timestep)
                self.distance_sensors.append(sensor)
            except:
                # If sensor doesn't exist, create a dummy
                print(f"[{self.robot_id}] Warning: No {name} sensor found")

        # Communication devices to supervisor
        try:
            self.supervisor_emitter = self.robot.getDevice("supervisor emitter")
        except:
            print(
                f"[{self.robot_id}] Warning: Supervisor communication devices not found"
            )
            self.supervisor_emitter = None

        # Communication devices for robot to robot communication
        try:
            self.squad_receiver = self.robot.getDevice("robot to robot receiver")
            self.squad_receiver.enable(self.timestep)

            self.squad_emitter = self.robot.getDevice("robot to robot emitter")
        except:
            print(
                f"[{self.robot_id}] Warning: Robot to robot communication devices not found"
            )
            self.squad_receiver = None
            self.squad_emitter = None

    def get_orientation(self) -> float:
        """Get current robot orientation in radians"""
        if self.compass:
            north = self.compass.getValues()
            return math.atan2(north[0], north[1])

        return 0.0

    def get_distance_readings(self) -> List[float]:
        """Get distance sensor readings"""
        readings = []
        for sensor in self.distance_sensors:
            readings.append(sensor.getValue())
        return readings

    def detect_obstacles(self) -> Tuple[bool, str]:
        """
        Detect obstacles around the robot
        Returns: (obstacle_detected, description)
        """
        distances = self.get_distance_readings()

        # Missing range sensors should not crash the controller.
        if len(distances) < 2:
            return False, "insufficient sensor data"

        # Check front sensors
        if (
            distances[0] < self.obstacle_threshold
            and distances[1] < self.obstacle_threshold
        ):
            return True, "obstacle ahead"
        elif distances[0] < self.obstacle_threshold:
            return True, "obstacle front left"
        elif distances[1] < self.obstacle_threshold:
            return True, "obstacle front right"

        return False, "clear path"

    def detect_victim(self) -> Tuple[bool, float]:
        """
        Attempt to detect victims using camera and sensors
        Returns: (victim_detected, confidence)
        """

        # Random chance of detecting something (including false positives)
        if random.random() < 0.01:  # small chance per check
            # Generate random confidence
            base_confidence = random.uniform(0.3, 0.95)

            if base_confidence > self.victim_confidence_threshold:
                return True, base_confidence

        return False, 0.0

    def send_victim_found_message(
        self,
        victim_detected: bool = False,
        confidence: float = 0.0,
    ):
        """Send victim found message to marking supervisor"""
        if not self.supervisor_emitter:
            print(f"[{self.robot_id}] Warning: Cannot send request - no emitter")
            return

        request = {
            "timestamp": self.robot.getTime(),
            "robot_id": self.robot_id,
            "position": [
                0.0,
                0.0,
                0.0,
            ],  # Your estimate of the robot's current position
            "victim_found": victim_detected,
            "victim_confidence": confidence,
        }

        message = json.dumps(request)
        self.supervisor_emitter.send(message.encode())

        if victim_detected:
            print(f"[{self.robot_id}] VICTIM ALERT: Confidence {confidence:.1%}")

    def set_wheel_speeds(self, left_speed: float, right_speed: float):
        """Set wheel motor speeds"""
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        self.front_left_motor.setVelocity(left_speed)
        self.rear_left_motor.setVelocity(left_speed)
        self.front_right_motor.setVelocity(right_speed)
        self.rear_right_motor.setVelocity(right_speed)

    def move_forward(self, speed: float = None):
        """Move robot forward"""
        if speed is None:
            speed = self.max_speed * 0.7
        self.set_wheel_speeds(speed, speed)

    def turn_left(self, speed: float = None):
        """Turn robot left"""
        if speed is None:
            speed = self.max_speed * 0.5
        self.set_wheel_speeds(-speed, speed)

    def turn_right(self, speed: float = None):
        """Turn robot right"""
        if speed is None:
            speed = self.max_speed * 0.5
        self.set_wheel_speeds(speed, -speed)

    def stop(self):
        """Stop robot movement"""
        self.set_wheel_speeds(0, 0)

    def set_explore_behavior(self):
        """Main exploration behavior logic"""
        obstacle_detected, obstacle_desc = self.detect_obstacles()
        victim_detected, victim_confidence = self.detect_victim()
        current_time = self.robot.getTime()

        # Priority 1: Investigate potential victims
        if victim_detected:
            self.send_victim_found_message(True, victim_confidence)
            self.last_decision_time = current_time
            self.action_pending = True
            self.action_fun = self.move_forward
            return

        # Priority 2: Handle obstacles
        elif obstacle_detected:
            if "front right" in obstacle_desc:
                self.action_fun = self.turn_left
            else:
                self.action_fun = self.turn_right

            self.action_pending = True
            self.last_decision_time = current_time
            return

        # Priority 3: Exploration
        elif (not self.action_pending) and (
            current_time - self.last_decision_time > self.decision_interval
        ):
            exploration_actions = [
                (0.4, self.move_forward),
                (0.3, self.turn_left),
                (0.3, self.turn_right),
            ]

            # Weighted random selection
            rand_val = random.random()
            cumulative = 0

            for weight, cb in exploration_actions:
                cumulative += weight
                if rand_val <= cumulative:
                    self.action_fun = cb
                    break
            self.action_pending = True
            self.last_decision_time = current_time

    def run(self):
        """Main robot control loop"""
        print(f"[{self.robot_id}] Starting search and rescue mission")

        while self.robot.step(self.timestep) != -1:

            if self.action_pending:
                self.action_fun()
                self.action_pending = False
            else:
                self.set_explore_behavior()


def main():
    """Main function to run the robot controller"""
    controller = BasicRosbotController()
    controller.run()


if __name__ == "__main__":
    main()
