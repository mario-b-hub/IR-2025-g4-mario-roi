import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# This node implements the "Reactive Navigation" architecture.
# It does not use maps (SLAM) but reacts instant-by-instant to LiDAR data.
class MyCustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        
        # 1. PUBLISHER: To send velocity commands
        # We define self.cmd_vel_publisher_ WITHIN __init__
        # This allows the robot to move by sending Twist messages (linear and angular velocity).
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2. SUBSCRIBER: To read the laser
        # Raw data input for the "Sensing Logic".
        # Every time a LiDAR message arrives, 'scan_callback' is executed.
        self.subscriber_ = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )

        # 3. STATE VARIABLES:
        # We create a 'global' variable for the class to store the distance.
        # Initialized with a high value (clear path) for safety.
        # STATE VARIABLES (Virtual Sensors)
        # These variables represent the 3 averaged sectors (Front, Left, Right)
        # instead of using hundreds of individual laser rays.
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        # 0: Moving Forward, 1: Turning Left, 2: Turning Right
        # Variable that controls the Finite State Machine (FSM).
        self.state = 0

        # 4. TIMER (BRAIN):
        # We execute the control logic every 0.2 seconds
        # Defines the decision frequency (5 Hz) mentioned in the specifications.
        self.timer_ = self.create_timer(0.2, self.control_loop)
        
        self.get_logger().info('MyCustomNode started: Autonomous robot ready.')

    def scan_callback(self, msg):
        # Implementation of the "Sector Averaging System".
        # This internal function takes a range of degrees and calculates the average,
        # filtering errors (inf/nan) to avoid false readings.
        # Internal helper function to calculate sector average
        def get_sector_distance(target_angle, window_deg=10):
            # 1. Convert window from degrees to number of indices
            # msg.angle_increment is radians per index
            window_rad = (window_deg * 3.14159) / 180.0
            half_window_indices = int(window_rad / msg.angle_increment / 2)
            
            # 2. Calculate theoretical center index for the desired angle
            # Formula: index = (desired_angle - min_angle) / increment
            center_index = int((target_angle - msg.angle_min) / msg.angle_increment)
            
            # 3. Define array limits (start and end) with overflow protection
            start = max(0, center_index - half_window_indices)
            end = min(len(msg.ranges), center_index + half_window_indices + 1)
            
            # 4. Extract and filter data
            sector = msg.ranges[start:end]
            valid_readings = [r for r in sector if not (r == float('inf') or r == float('nan'))]
            
            if not valid_readings:
                return 10.0 # No valid data, assume clear
            return sum(valid_readings) / len(valid_readings)

        # STATE UPDATES
        # Here we update the "Virtual Sensors" mentioned in the architecture.
        # Front (0 radians) -> Detects immediate collisions.
        self.front_dist = get_sector_distance(0.0)
        
        # Left (approx +45 degrees = +0.78 rad) -> Measures left lateral clearance.
        self.left_dist = get_sector_distance(0.78)
        
        # Right (approx -45 degrees = -0.78 rad) -> Measures right lateral clearance.
        self.right_dist = get_sector_distance(-0.78)
        
        # Debug to verify integrity
        # self.get_logger().info(f'F: {self.front_dist:.2f} | L: {self.left_dist:.2f} | R: {self.right_dist:.2f}')  

    def decide_turn_direction(self):
        """
        Analyzes which side has more free space.
        Returns -1 for Left (default direction) or +1 for Right.
        """
        # Logic to solve T-junctions or Cross-junctions.
        # Mathematically compares which side is larger to make a deterministic decision.
        if self.left_dist > self.right_dist:
            # Free path to the left
            self.get_logger().info(f'Gap detected to LEFT (L:{self.left_dist:.1f} > R:{self.right_dist:.1f})')
            return -1 # Negative sign (Counter-clockwise)
        else:
            # Free path to the right
            self.get_logger().info(f'Gap detected to RIGHT (R:{self.right_dist:.1f} > L:{self.left_dist:.1f})')
            return 1  # Positive sign (Clockwise)

    def control_loop(self):
        # --- PARAMETERS ---
        # Technical specifications and Hysteresis parameters.
        STOP_DIST = 1       # Distance to initiate turn (Enter State 1)
        RESUME_DIST = 2.5   # Distance to finish turn (Exit to State 0) - Prevents "Jittering"
        KP = 0.45           # Proportional Gain for corridor centering
        CORRIDOR_LIMIT = 2.0 

        # --- FINITE STATE MACHINE ---
        # Implementation of the FSM.
        
        if self.state == 0: # STATE: FORWARD (Corridor Centering / "Cruise" Mode)
            if self.front_dist > STOP_DIST:
                # 1. Centering in corridor (Keep what works)
                linear = 0.8
                angular_correction = 0.0

                # Proportional Controller (P-Control) Logic.
                # Calculates error (Left - Right) to stay in the center.
                if self.left_dist < CORRIDOR_LIMIT and self.right_dist < CORRIDOR_LIMIT:
                    error = self.left_dist - self.right_dist
                    angular_correction = KP * error
                    # Limit correction to avoid sharp zig-zag
                    angular_correction = max(-0.4, min(0.4, angular_correction))
                
                self.move_robot(linear, angular_correction)
                
            else:
                # 2. CORNER DETECTED -> CRITICAL DECISION
                # Transition from State 0 to 1.
                # Triggered when front wall is closer than STOP_DIST.
                self.get_logger().info(f'Front wall ({self.front_dist:.2f}m). Analyzing turn...')
                
                # HERE WE CALL THE NEW FUNCTION
                # We decide the direction ONCE and store it ("Latch Decision")
                self.turn_direction = self.decide_turn_direction()
                
                # Change state
                self.state = 1

        elif self.state == 1: # STATE: TURN (Dynamic)
            # Use of Hysteresis.
            # We only resume moving forward if the front is very clear (RESUME_DIST > STOP_DIST).
            # This ensures the robot has fully completed the turn before accelerating.
            if self.front_dist > RESUME_DIST:
                self.state = 0 
                self.get_logger().info('Corridor aligned. Moving forward.')
                self.move_robot(0.9, 0.0)
            else:
                # 3. EXECUTION OF TURN
                # Use a base speed (0.5) multiplied by the decided direction (-1 or +1)
                # This automatically inverts the turn if necessary.
                velocidad_giro = 1 * self.turn_direction
                self.move_robot(0.0, velocidad_giro)


    def move_robot(self, linear, angular):
        # Helper function to publish the message
        # Packages velocities into a standard ROS 2 message (Twist)
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot when closing with Ctrl+C
        node.move_robot(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
