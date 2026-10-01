import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray, Empty
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        
        # /////////// declaring the params \\\\\\\\\\
        self.declare_parameter('bubble_radius', 0.3) # in meters
        self.declare_parameter('preprocess_conv_size', 3) # the window to average values
        self.declare_parameter('max_lidar_range', 3.0) # meters
        self.declare_parameter('speed_max', 2.0)
        self.declare_parameter('steering_gain', 1.0) # proportional gain
        self.declare_parameter('max_steering', 30.0) # proportional gain

        # ////// steering low-pass (dynamic alpha) \\\\\\\\\
        self.declare_parameter('alpha_straight', 0.8) # heavy smoothing when the target angle is small
        self.declare_parameter('alpha_curve', 0.35) # light smoothing when the target angle is large -> reacts fast

        # ////// how far around bends the car can see and react to \\\\\\
        self.declare_parameter('fov_deg', 70.0) # degrees, EACH side of center
        self.declare_parameter('gap_aim_aggressiveness', 0.5) # 0 = always aim at the gap's center | 1 = aim at the furthest visible point

        # ///// straightaway detection (predictive lookahead) \\\\\
        self.declare_parameter('straight_min_forward_frac', 0.75)
        self.declare_parameter('straight_max_angle_deg', 6.0) # steering EMA must be under this to count as "open"
        self.declare_parameter('straight_ema_alpha', 0.2) # smoothing on the steering-magnitude EMA (higher = reacts faster)

        # ///// lap tracking!!!! \\\\\\
        self.declare_parameter('lap_leave_dist', 3.0) # meters: must get this far from start before a lap can arm
        self.declare_parameter('lap_return_dist', 1.5) # meters: within this counts as crossing the line
        self.declare_parameter('lap_min_time', 5.0) # seconds: guards against re-triggering on the same crossing

        #  /////// dashboard lidar view / predicted path \\\\\\\\\\
        self.declare_parameter('wheelbase', 0.25) # meters
        self.declare_parameter('lidar_view_every_n', 10) # publish /lidar_view every Nth scan (throttle)

        # ////////// recovery (backing out after getting stuck) \\\\\\\\
        self.declare_parameter('recovery_straight_time', 0.8) 
        self.declare_parameter('recovery_clear_dist', 1.0) 
        self.declare_parameter('recovery_max_time', 6.0) 

        # /////// dynamic track-scale adaptation \\\\\\\\
        self.declare_parameter('enable_dynamic_scaling', True)
        self.declare_parameter('lidar_hw_range_m', 10.0)
        self.declare_parameter('reference_corridor_m', 2.0) 
        self.declare_parameter('min_track_scale', 0.4) 
        self.declare_parameter('max_track_scale', 1.8) 
        self.declare_parameter('track_scale_ema_alpha', 0.05) 

        # ////////// get parameters \\\\\\\\\
        self.bubble_radius_base = self.get_parameter('bubble_radius').value
        self.bubble_radius = self.bubble_radius_base
        self.preprocess_conv_size = self.get_parameter('preprocess_conv_size').value
        self.max_lidar_range_base = self.get_parameter('max_lidar_range').value
        self.max_lidar_range = self.max_lidar_range_base
        self.speed_max_base = self.get_parameter('speed_max').value
        self.speed_max = self.speed_max_base
        self.steering_gain = self.get_parameter('steering_gain').value
        self.current_speed = 0.0
        self.max_steering = np.deg2rad(self.get_parameter('max_steering').value)
        self.alpha_straight = self.get_parameter('alpha_straight').value
        self.alpha_curve = self.get_parameter('alpha_curve').value
        self.fov_deg_base = self.get_parameter('fov_deg').value
        self.fov = np.deg2rad(self.fov_deg_base) # how much we want the car to see infront of it
        self.gap_aim_aggressiveness = self.get_parameter('gap_aim_aggressiveness').value

        # dynamic track-scale adaptation
        self.enable_dynamic_scaling = self.get_parameter('enable_dynamic_scaling').value
        self.lidar_hw_range_m = self.get_parameter('lidar_hw_range_m').value
        self.reference_corridor_m = self.get_parameter('reference_corridor_m').value
        self.min_track_scale = self.get_parameter('min_track_scale').value
        self.max_track_scale = self.get_parameter('max_track_scale').value
        self.track_scale_ema_alpha = self.get_parameter('track_scale_ema_alpha').value
        self.track_scale = None 
        self.prev_steering = 0.0
        self.steering_smoothing = 0.6
        self.recovery_start_time = None

        # predictive lookahead
        self.straight_min_forward_frac = self.get_parameter('straight_min_forward_frac').value
        self.straight_max_angle = np.deg2rad(self.get_parameter('straight_max_angle_deg').value)
        self.straight_ema_alpha = self.get_parameter('straight_ema_alpha').value
        self.straight_angle_ema = 0.0
        self.in_straight_mode = False

        # lap tracking!!!!!!
        self.lap_leave_dist_base = self.get_parameter('lap_leave_dist').value
        self.lap_leave_dist = self.lap_leave_dist_base
        self.lap_return_dist_base = self.get_parameter('lap_return_dist').value
        self.lap_return_dist = self.lap_return_dist_base
        self.lap_min_time = self.get_parameter('lap_min_time').value
        self.start_pos = None
        self.has_left_start = False
        self.lap_count = 0
        self.lap_start_time = None
        self.last_lap_time = 0.0
        self.best_lap_time = None

        # dashboard stuff
        self.wheelbase = self.get_parameter('wheelbase').value
        self.lidar_view_every_n = self.get_parameter('lidar_view_every_n').value
        self.lidar_view_counter = 0

        # recovery
        self.recovery_straight_time = self.get_parameter('recovery_straight_time').value
        self.recovery_clear_dist_base = self.get_parameter('recovery_clear_dist').value
        self.recovery_clear_dist = self.recovery_clear_dist_base
        self.recovery_max_time = self.get_parameter('recovery_max_time').value
        self.recovery_steering = 0.0
        
        # /////////// FOR ODOM \\\\\\\\\\\\
        self.actual_speed = 0.0
        self.stuck_counter = 0
        self.stuck_threshold = 25 # hz scan rate
        self.is_recovering = False
        self.recovery_counter = 0
        self.post_recovery_bias = 0.0
        self.post_recovery_counter = 0
        
        # /////////// topic names \\\\\\\\\\\\\\
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        # /////// subs \\\\\\\
        # what the lidar produces
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10) # 10 for the refresh rate
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom',self.odom_callback, 10)
        self.reset_sub = self.create_subscription(Empty, '/reactive_reset', self.reset_callback, 10)
        
        # /////// pubs \\\\\\\
        # the commands for driving (steering / speed)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10) # 10 is refresh rate
        self.debug_pub = self.create_publisher(Float32MultiArray, '/car_info_debug', 10)
        self.lap_pub = self.create_publisher(Float32MultiArray, '/lap_info', 10)
        self.lidar_view_pub = self.create_publisher(Float32MultiArray, '/lidar_view', 10)

        # for the tuner
        self.add_on_set_parameters_callback(self.on_parameter_update)

        # ////////// DEBUGGER \\\\\\\\\
        self.get_logger().info("\/"*15)
        self.get_logger().info(f"Bubble: {self.bubble_radius}m")
        self.get_logger().info(f"MAX liDAR Range: {self.max_lidar_range}m")
        self.get_logger().info(f"max speed: {self.speed_max}m/s")
        self.get_logger().info(f"\/"*15)

    
    def odom_callback(self, msg):
        # get the supposed value
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.actual_speed = msg.twist.twist.linear.x

        # get the odom value
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        now = self.get_clock().now()

        if self.start_pos is None:
            self.start_pos = (pos_x, pos_y)
            self.lap_start_time = now
            return

        dist_from_start = np.hypot(pos_x - self.start_pos[0], pos_y - self.start_pos[1]) # calculated distance from the start

        # used as a way to decide how much time has elapsed since you left the start
        if not self.has_left_start:
            if dist_from_start > self.lap_leave_dist:
                self.has_left_start = True
        else:
            if dist_from_start < self.lap_return_dist:
                elapsed = (now - self.lap_start_time).nanoseconds / 1e9
                if elapsed > self.lap_min_time:
                    self.lap_count += 1
                    self.last_lap_time = elapsed
                    if self.best_lap_time is None or elapsed < self.best_lap_time:
                        self.best_lap_time = elapsed
                    self.lap_start_time = now
                    self.has_left_start = False
                    self.get_logger().info(
                        f'LAP {self.lap_count} complete: {elapsed:.2f}s (best: {self.best_lap_time:.2f}s)'
                    )

        self._publish_lap_info(now)

    def _publish_lap_info(self, now): # lap info
        current_lap_elapsed = 0.0
        if self.lap_start_time is not None:
            current_lap_elapsed = (now - self.lap_start_time).nanoseconds / 1e9

        lap_msg = Float32MultiArray()
        lap_msg.data = [
            float(self.lap_count),
            float(current_lap_elapsed),
            float(self.last_lap_time),
            float(self.best_lap_time if self.best_lap_time is not None else 0.0),
        ]
        self.lap_pub.publish(lap_msg)

    def reset_callback(self, msg): #this will reset all values for tracking
        self.prev_steering = 0.0
        self.current_speed = 0.0
        self.stuck_counter = 0
        self.is_recovering = False
        self.post_recovery_counter = 0
        self.post_recovery_bias = 0.0
        self.straight_angle_ema = 0.0
        self.in_straight_mode = False
        self.lap_count = 0
        self.has_left_start = False
        self.lap_start_time = self.get_clock().now()
        self.last_lap_time = 0.0
        self.best_lap_time = None
        self.track_scale = None
        self.get_logger().info('State reset (dashboard restart)')

    def on_parameter_update(self, params): # everything that is updated with my param tuner dashboard
        for p in params:
            name, value = p.name, p.value
            if name == 'bubble_radius':
                self.bubble_radius_base = value
                if not self.enable_dynamic_scaling:
                    self.bubble_radius = value
            elif name == 'preprocess_conv_size':
                self.preprocess_conv_size = value
            elif name == 'max_lidar_range':
                self.max_lidar_range_base = value
                if not self.enable_dynamic_scaling:
                    self.max_lidar_range = value
            elif name == 'speed_max':
                self.speed_max_base = value
                if not self.enable_dynamic_scaling:
                    self.speed_max = value
            elif name == 'steering_gain':
                self.steering_gain = value
            elif name == 'max_steering':
                self.max_steering = np.deg2rad(value)
            elif name == 'alpha_straight':
                self.alpha_straight = value
            elif name == 'alpha_curve':
                self.alpha_curve = value
            elif name == 'fov_deg':
                self.fov_deg_base = value
                if not self.enable_dynamic_scaling:
                    self.fov = np.deg2rad(value)
            elif name == 'gap_aim_aggressiveness':
                self.gap_aim_aggressiveness = value
            elif name == 'straight_min_forward_frac':
                self.straight_min_forward_frac = value
            elif name == 'straight_max_angle_deg':
                self.straight_max_angle = np.deg2rad(value)
            elif name == 'straight_ema_alpha':
                self.straight_ema_alpha = value
            elif name == 'lap_leave_dist':
                self.lap_leave_dist_base = value
                if not self.enable_dynamic_scaling:
                    self.lap_leave_dist = value
            elif name == 'lap_return_dist':
                self.lap_return_dist_base = value
                if not self.enable_dynamic_scaling:
                    self.lap_return_dist = value
            elif name == 'lap_min_time':
                self.lap_min_time = value
            elif name == 'wheelbase':
                self.wheelbase = value
            elif name == 'lidar_view_every_n':
                self.lidar_view_every_n = value
            elif name == 'recovery_straight_time':
                self.recovery_straight_time = value
            elif name == 'recovery_clear_dist':
                self.recovery_clear_dist_base = value
                if not self.enable_dynamic_scaling:
                    self.recovery_clear_dist = value
            elif name == 'recovery_max_time':
                self.recovery_max_time = value
            elif name == 'enable_dynamic_scaling':
                self.enable_dynamic_scaling = value
                if not value:
                    self.bubble_radius = self.bubble_radius_base
                    self.max_lidar_range = self.max_lidar_range_base
                    self.speed_max = self.speed_max_base
                    self.fov = np.deg2rad(self.fov_deg_base)
                    self.recovery_clear_dist = self.recovery_clear_dist_base
                    self.lap_leave_dist = self.lap_leave_dist_base
                    self.lap_return_dist = self.lap_return_dist_base
            elif name == 'lidar_hw_range_m':
                self.lidar_hw_range_m = value
            elif name == 'reference_corridor_m':
                self.reference_corridor_m = value
            elif name == 'min_track_scale':
                self.min_track_scale = value
            elif name == 'max_track_scale':
                self.max_track_scale = value
            elif name == 'track_scale_ema_alpha':
                self.track_scale_ema_alpha = value
        return SetParametersResult(successful=True)

    def _update_dynamic_scaling(self, data): # retunes values based on how large track is
        if not self.enable_dynamic_scaling:
            return

        ranges = np.nan_to_num(np.array(data.ranges), nan=0.0, posinf=0.0, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.lidar_hw_range_m)
        angles = data.angle_min + np.arange(len(ranges)) * data.angle_increment

        # get the window of ranges from the right and left and check if scan if useful
        window = np.abs(angles) <= np.pi / 2
        right = ranges[window & (angles < 0)]
        left = ranges[window & (angles >= 0)]
        right = right[right > 0]
        left = left[left > 0]
        if len(right) == 0 or len(left) == 0:
            return  

        corridor_m = float(np.mean(right) + np.mean(left)) # this is how big the track is on both sides
        if corridor_m < 0.2: # check for sensor glitch
            return  

        raw_scale = float(np.clip(corridor_m / self.reference_corridor_m, 
                                   self.min_track_scale, self.max_track_scale)) # this is how much to scale raw

        if self.track_scale is None: # check if there already is a scale otherwise update the track scale based on an alpha value
            self.track_scale = raw_scale 
        else:
            a = self.track_scale_ema_alpha
            self.track_scale = (1 - a) * self.track_scale + a * raw_scale

        # now finally update lidar range, bubble radius, fov, speed, recovery distance, and lap tracking
        scale = self.track_scale
        self.max_lidar_range = float(np.clip(self.max_lidar_range_base * scale, 1.0, self.lidar_hw_range_m))
        self.bubble_radius = float(np.clip(self.bubble_radius_base * scale, 0.1, 1.5))
        self.fov = float(np.deg2rad(np.clip(self.fov_deg_base * scale, 30.0, 90.0)))
        self.speed_max = float(np.clip(self.speed_max_base * scale, 0.3, self.speed_max_base * self.max_track_scale))
        self.recovery_clear_dist = float(np.clip(self.recovery_clear_dist_base * scale, 0.3, 2.0))
        self.lap_leave_dist = float(np.clip(self.lap_leave_dist_base * scale, 0.5, 20.0))
        self.lap_return_dist = float(np.clip(self.lap_return_dist_base * scale, 0.3, 10.0))

        self.get_logger().info(
            f'[track-scale] corridor~{corridor_m:.2f}m scale={scale:.2f} | '
            f'bubble={self.bubble_radius:.2f}m range={self.max_lidar_range:.2f}m '
            f'fov={np.rad2deg(self.fov):.0f}deg speed_max={self.speed_max:.2f}m/s',
            throttle_duration_sec=2.0)

    def preprocess_lidar(self, ranges): # this is to predict what is ahead of you
        # preprocess the lidar scan array
        
        # 1. set each value to mean over some window
        processed_ranges = np.array(ranges) 
        
        # get rid of all the nan values
        processed_ranges = np.nan_to_num(processed_ranges, nan=0.0)
        
        # mean each value in range -> [inside that window]
        # moving average
        weights = np.repeat(1.0, self.preprocess_conv_size) / self.preprocess_conv_size
        processed_ranges = np.convolve(processed_ranges, weights, 'same') # makes sure that average is only calculated when window fully overlaps
        
        # ----------------------------------------------
        
        # 2. reject high values (> max lidar range)
        
        # for loop to go over every value in the range
        # if statement to check if value is in lidar range
        # if not in range set to set to max range if out of range
        
        # or just use clip
        processed_ranges = np.clip(processed_ranges, 0, self.max_lidar_range) # clip between 
        
        return processed_ranges #TODO: implement preprocessing

    
    def lidar_callback(self, data): # 
        ranges = np.array(data.ranges)
        # ------------------------ {PROCESSING} ------------------------

        # Re-tune bubble_radius / max_lidar_range / fov / speed_max /
        # recovery_clear_dist / lap_leave_dist / lap_return_dist for the
        # track we're actually on, BEFORE anything below reads them.
        self._update_dynamic_scaling(data)

        # step 1: preprocess
        processed_ranges = self.preprocess_lidar(ranges) # we want the values to be between 0 and self.max_lidar_range. 
                                                         # NO nan NO inf NO less than 0 or greater than max_lidar_range
        
        # we also want to keep the car from looking at too much 
        center_index = int((0 - data.angle_min) / data.angle_increment)
        beams_per_side = int(self.fov / data.angle_increment)
        start = max(0, center_index - beams_per_side)
        end = min(len(processed_ranges), center_index+beams_per_side)
        processed_ranges = processed_ranges[start:end]
        
        # find closest points
        
        # whatever value is the smallest
        copied_ranges = processed_ranges.copy() # dont want to make changes to the current ranges
        non_zero = copied_ranges[copied_ranges>0] # we only want the nonzero values
        
        if len(non_zero) == 0:
            self.get_logger().warn('NO VALID LIDAR RANGES')
            self.publish_drive(0.0, 0.0)
            return
        
        smallest_dist = np.min(non_zero) # only care about the non zero valeus to find the smallest
        smallest_dist = smallest_dist if smallest_dist != 0 else 0.1 # small angle approx
        smallest_index = np.argmin(copied_ranges) # fining the index
        
        # eliminate points inside bubble
              
        # we want to know where the bubble will start and where it ends
          
        bubble_indices = self.calc_bubble(data, smallest_dist, smallest_index, len(copied_ranges))
        
        # then set the index to 0 
        copied_ranges[bubble_indices] = 0.0 # this is to set all values within the bubble to 0. MAKE THEM KNOWN THEY ARE CLOSE AND UNSAFE
        # this way when follow the gap procedure is done, everything is excluded within the bubble
        

        # stuck checker
        start_gap_index, end_gap_index = self.find_max_gap(copied_ranges)


        if self.current_speed > 0.3:
            moving_correctly = self.actual_speed > 0.1
        elif self.current_speed < -0.3:
            moving_correctly = self.actual_speed < -0.1  # reversing successfully
        else:
            moving_correctly = True  # not commanding motion, not stuck

        commanded_motion = abs(self.current_speed) > 0.3

        if commanded_motion and not moving_correctly:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)  # decay instead of hard reset

        if self.recovery_logic(processed_ranges): return

        # find the max value in the gap
        best_index = self.max_val_in_gap(copied_ranges, start_gap_index, end_gap_index)
        
        # ------------------------ {STEERING AND SPEED CALCULATIONS} ------------------------
        # convert index to angle
        angle_to_best = data.angle_min + (best_index+start) * data.angle_increment
        
        angle_frac = np.clip(abs(angle_to_best) / self.max_steering, 0.0, 1.0)
        gain = self.steering_gain * (1.0 + 0.5 * angle_frac)
        
        # steering gain
        steering_angle = gain * angle_to_best
        
        # clamp the steering angle to not go out of bounds
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
        # LOW PASS FILTER (used in sensor fusion)
        # allows slow, steady signals to pass while blocking higher frequency noise

        curve_frac = np.clip(abs(steering_angle) / self.max_steering, 0.0, 1.0)
        alpha = self.alpha_straight - (self.alpha_straight - self.alpha_curve) * curve_frac


        old_steering = self.prev_steering

        steering_angle = alpha * self.prev_steering + (1 - alpha) * steering_angle
                
        # take anything infront of the car
        forward_beams = int(np.deg2rad(25) / data.angle_increment)
        mid = len(processed_ranges) // 2
        forward_slice = processed_ranges[mid - forward_beams : mid + forward_beams  ]
        min_forward = np.min(forward_slice[forward_slice > 0]) if np.any(forward_slice > 0) else self.max_lidar_range  # find the minimum nonzero value infront of car
        
        # if min_forward / max(self.current_speed, 0.1)  < 0.5:
        #     self.get_logger().warn('ABOUT TO CRASH')
        #     # left_half = processed_ranges[len(processed_ranges)//2:]
        #     # right_half = processed_ranges[:len(processed_ranges)//2]
            
        #     # left_space = np.sum(left_half)
        #     # right_space = np.sum(right_half)
        #     # self.get_logger().info(f"left_space info: {left_space} | right_space info: {right_space}")

            
        #     # recovery_steering = self.max_steering if left_space >right_space else -self.max_steering
        #     # self.get_logger().info(f"steering this way: {np.rad2deg(recovery_steering)}")
        #     # self.publish_drive(recovery_steering, -0.5) #slow down
        #     return

        # steeper distance scaling, breaking distance grows with speed
        # distance_factor = np.clip((min_forward / self.max_lidar_range) ** 1.5, 0.2, 1.0)
        gap_depth = np.mean(copied_ranges[start_gap_index:end_gap_index+1])
        distance_factor = np.clip(gap_depth / self.max_lidar_range, 0.3, 1.0)
        # more steering -> slower speeds
        # PURE PURSUIT METHOD (speed = max * cos(angle)^2 * distance)

        # straightaway detection: predictive lookahead, continuous blend
        # Since we can run into the problem of being too noisy per frame, we created this. 
        # An EMA of steering magnitude that will smoothen noise. The speed blends continuously towards a speed_max as corridor opens 
        self.straight_angle_ema = (1 - self.straight_ema_alpha) * self.straight_angle_ema + self.straight_ema_alpha * abs(steering_angle)

        forward_threshold = self.straight_min_forward_frac * self.max_lidar_range
        forward_openness = np.clip(min_forward / forward_threshold, 0.0, 1.0) if forward_threshold > 0 else 1.0
        angle_openness = np.clip(1.0 - (self.straight_angle_ema / self.straight_max_angle), 0.0, 1.0)
        openness = forward_openness * angle_openness  # both need to be good — wide open lane at an angle isn't "straight"
        self.in_straight_mode = openness > 0.8  # kept for the brake/boost guard below + future telemetry

        reactive_speed = self.speed_max * (np.cos(steering_angle)**4) * distance_factor # we want to lower the speed based on how close the object infront of us is
        speed = reactive_speed + (self.speed_max - reactive_speed) * openness

        steering_delta = steering_angle - old_steering

        # steering rate limiter ...
        max_steer_rate = np.deg2rad(4.0)  # per lidar update

        delta = steering_angle - self.prev_steering

        delta = np.clip(
            delta,
            -max_steer_rate,
            max_steer_rate
        )

        steering_angle = self.prev_steering + delta

        self.prev_steering = steering_angle
        if not self.in_straight_mode:
            if abs(steering_angle) > 0.2 and steering_delta * steering_angle > 0:
                speed *= 0.6 # entering a turn -> must break
            elif abs(steering_angle) < 0.15 and abs(self.prev_steering) > 0.3:
                speed = min(self.speed_max, speed*1.3) # boost on exit
        
        
        # post recovery steering so that we dont pick the bad gap again
        if self.post_recovery_counter > 0:
            steering_angle = np.clip(steering_angle +0.5 * self.post_recovery_bias, -self.max_steering, self.max_steering)
            self.post_recovery_counter -= 1

        # dashboard lidar view + predicted path — throttled, this is the
        # full scan window (not the per-message debug summary), no need
        # to send it at full scan rate
        self.lidar_view_counter += 1
        if self.lidar_view_counter >= self.lidar_view_every_n:
            self.lidar_view_counter = 0
            window_angle_min = data.angle_min + start * data.angle_increment
            view_msg = Float32MultiArray()
            view_msg.data = [
                float(window_angle_min),
                float(data.angle_increment),
                float(steering_angle),
                float(speed),
                float(self.wheelbase),
                float(self.max_lidar_range),
            ] + processed_ranges.tolist()
            self.lidar_view_pub.publish(view_msg)

        # TODO: pubolish drive message
        self.publish_drive(steering_angle, speed)
        
        # DEBUGGER
        # self.get_logger().info("<"+"-"*20+">", throttle_duration_sec=0.5)
        # self.get_logger().info(f'CURRENT SPEED: {speed}| TURNING ANGLE: {np.rad2deg(steering_angle)}', throttle_duration_sec=0.5)
        # self.get_logger().info(f'GAP: [{start_gap_index, end_gap_index}] | BEST: {best_index}', throttle_duration_sec=0.5)
        debugging_message = Float32MultiArray()
        debugging_message.data = [
            float(speed),
            float(self.actual_speed),
            float(np.rad2deg(steering_angle)),
            float(start_gap_index),
            float(end_gap_index),
            float(best_index),
        ]
                
        self.debug_pub.publish(debugging_message)


    def _side_clearance(self, processed_ranges):
        # higher indices = left, lower indices = right (matches angle
        # increasing counterclockwise with index, same convention used
        # for `mid` elsewhere in this file)
        n = len(processed_ranges)
        right_half = processed_ranges[:n // 2]
        left_half = processed_ranges[n // 2:]
        right_nonzero = right_half[right_half > 0]
        left_nonzero = left_half[left_half > 0]
        right_space = float(np.mean(right_nonzero)) if len(right_nonzero) > 0 else 0.0
        left_space = float(np.mean(left_nonzero)) if len(left_nonzero) > 0 else 0.0
        return left_space, right_space

    def recovery_logic(self, processed_ranges):
        if self.stuck_counter > self.stuck_threshold and not self.is_recovering:
            self.is_recovering = True
            self.recovery_start_time = self.get_clock().now()

            # steer away from whichever side is actually tighter, instead
            # of a fixed hardcoded direction — that was the bug: it always
            # reversed the same way regardless of which side the car
            # actually hit, sometimes steering straight back into the wall
            left_space, right_space = self._side_clearance(processed_ranges)
            self.recovery_steering = -self.max_steering if right_space < left_space else self.max_steering
            self.get_logger().warning(
                f"WE ARE STUCK - reversing {'left' if self.recovery_steering > 0 else 'right'} "
                f"(L:{left_space:.2f}m R:{right_space:.2f}m)"
            )

        if self.is_recovering:
            elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
            reverse_speed = -0.5

            if elapsed < self.recovery_straight_time:
                # phase 1: straighten out and get some distance from the
                # wall before committing to a steered direction
                self._publish_reverse(0.0, reverse_speed)
                return True

            # closed-loop exit check — keep backing out until BOTH sides
            # actually measure clear, not just "some fixed number of
            # seconds passed" (which was letting it exit while still
            # wedged against the wall and immediately re-trigger recovery)
            left_space, right_space = self._side_clearance(processed_ranges)
            both_sides_clear = left_space > self.recovery_clear_dist and right_space > self.recovery_clear_dist

            if both_sides_clear or elapsed > self.recovery_max_time:
                self.is_recovering = False
                self.stuck_counter = -100
                # small nudge away from the side we just recovered from,
                # so normal driving doesn't immediately drift back into it
                self.post_recovery_bias = 0.3 if self.recovery_steering > 0 else -0.3
                self.post_recovery_counter = 40
                reason = 'clear on both sides' if both_sides_clear else 'hit max recovery time'
                self.get_logger().warn(f'done recovering ({reason}) L:{left_space:.2f}m R:{right_space:.2f}m')
                return False

            self._publish_reverse(self.recovery_steering, reverse_speed)
            return True

        return False
                
        
    def calc_bubble(self, data, smallest_dist, smallest_index, array_length):
        dynamic_radius = self.bubble_radius * (1.0 + self.current_speed / self.speed_max) # at full speed it is double the size
        #calc the angle opposite by bubble
        bubble_angle = dynamic_radius / max(smallest_dist,0.5) # prevents bubble from blowing up to infinity when somehting is next to you
        # number of indices 
        bubble_index_range = int(np.ceil(bubble_angle / data.angle_increment)) # get how many lidar beams that angle covers
        start_index = max(0, smallest_index - bubble_index_range)
        end_index = min(array_length - 1, smallest_index + bubble_index_range)
        return np.arange(start_index, end_index +1) # positions in lidar array that fall inside danger bubble
    
    def find_max_gap(self, data):
        # return the start and end index of the max gap
        # a gap is continuous sequence of non-zero values
        
        # find where free space is
        # create a mask to show the nonzero values
        mask = data > 0 
        
        # find each edge index of the gaps
        # set bounds of array
        edges = np.diff(np.concatenate(([0], mask.astype(int), [0])))

        # [0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0]
        # for np.diff it finds changes with 0->1 as 1 and 1->0 as -1
        # [0, 1, 0,-1, 1,-1, 1, 0, 0, -1]
        
        
        # we want the largest gap of non zero values so we check
        # check where edge starts and ends
        start = np.where(edges == 1)[0] # looking or vals of 1
        end = np.where(edges == -1)[0] # looking or vals of -1
        
        
        # make sure the start exists
        if len(start) == 0:
            return 0, 0
        # calc that gap
        gap_length = end - start # this is an array
        
        # keep the longest gap
        largest_gap_index = np.argmax(gap_length)
        
        return start[largest_gap_index], end[largest_gap_index] -1
    
    def max_val_in_gap(self, data, start_index, end_index):
        # find the max val between indices
        # for loop starting 
        if start_index >= end_index or end_index >= len(data):
            return len(data) // 2
        gap_ranges = data[start_index:end_index+1]
        
        gap_width = end_index-start_index
        gap_average_depth = np.mean(gap_ranges)
        gap_min_depth = np.min(gap_ranges)
        
        # plateau aware of furthest point 
        max_val = gap_ranges.max()
        plateau = np.where(gap_ranges >= max_val - 0.05)[0]
        furthest_index = start_index + int(plateau.mean())

        centered_index = (start_index + end_index) // 2
        
        
        # Normalize each factor to roughly [0, 1]
        width_score = gap_width / 200.0           # 200 beams = "wide"
        depth_score = gap_average_depth / self.max_lidar_range
        safety_score = gap_min_depth / self.max_lidar_range  # punishes gaps with close obstacles
        
        # aggression score chooses how to tune the individual weights
        aggression = 0.4 * width_score + 0.4 * depth_score + 0.2 * safety_score

        speed_factor = self.current_speed / self.speed_max # fast == 1, slow == 0
        alpha = np.clip(aggression * 0.5 + speed_factor * 0.5, 0.0, 1.0)

        alpha *= self.gap_aim_aggressiveness

        return int(alpha * furthest_index + (1 - alpha) * centered_index)
    
    def publish_drive(self, steering_angle, speed):
        self.current_speed = speed
        
        drive_msg = AckermannDriveStamped() # how to talk to the f1tenth car by giving steering angle and speed
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        
        self.drive_pub.publish(drive_msg)

    def _publish_reverse(self, steering_angle, speed):
        self.current_speed = speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)

        self.drive_pub.publish(drive_msg)
        
        debugging_message = Float32MultiArray()
        debugging_message.data = [
            float(speed),
            float(self.actual_speed),
            float(np.rad2deg(steering_angle)),
            -1.0,
            -1.0,
            -1.0,
        ]
        self.debug_pub.publish(debugging_message)
        
def main(args=None):
    rclpy.init(args=args)
    
    print("initialized...")
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
                
if __name__ == "__main__":
    main()
        