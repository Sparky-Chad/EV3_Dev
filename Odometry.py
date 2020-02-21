
from ev3dev2.motor import MoveTank
class MoveDifferential(MoveTank):
    """
    MoveDifferential is a child of MoveTank that adds the following capabilities:

    - drive in a straight line for a specified distance

    - rotate in place in a circle (clockwise or counter clockwise) for a
      specified number of degrees

    - drive in an arc (clockwise or counter clockwise) of a specified radius
      for a specified distance

    Odometry can be use to enable driving to specific coordinates and
    rotating to a specific angle.

    New arguments:

    wheel_class - Typically a child class of :class:`ev3dev2.wheel.Wheel`. This is used to
    get the circumference of the wheels of the robot. The circumference is
    needed for several calculations in this class.

    wheel_distance_mm - The distance between the mid point of the two
    wheels of the robot. You may need to do some test drives to find
    the correct value for your robot.  It is not as simple as measuring
    the distance between the midpoints of the two wheels. The weight of
    the robot, center of gravity, etc come into play.

    You can use utils/move_differential.py to call on_arc_left() to do
    some test drives of circles with a radius of 200mm. Adjust your
    wheel_distance_mm until your robot can drive in a perfect circle
    and stop exactly where it started. It does not have to be a circle
    with a radius of 200mm, you can test with any size circle but you do
    not want it to be too small or it will be difficult to test small
    adjustments to wheel_distance_mm.

    Example:

    .. code:: python

        from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveDifferential, SpeedRPM
        from ev3dev2.wheel import EV3Tire

        STUD_MM = 8

        # test with a robot that:
        # - uses the standard wheels known as EV3Tire
        # - wheels are 16 studs apart
        mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 16 * STUD_MM)

        # Rotate 90 degrees clockwise
        mdiff.turn_right(SpeedRPM(40), 90)

        # Drive forward 500 mm
        mdiff.on_for_distance(SpeedRPM(40), 500)

        # Drive in arc to the right along an imaginary circle of radius 150 mm.
        # Drive for 700 mm around this imaginary circle.
        mdiff.on_arc_right(SpeedRPM(80), 150, 700)

        # Enable odometry
        mdiff.odometry_start()

        # Use odometry to drive to specific coordinates
        mdiff.on_to_coordinates(SpeedRPM(40), 300, 300)

        # Use odometry to go back to where we started
        mdiff.on_to_coordinates(SpeedRPM(40), 0, 0)

        # Use odometry to rotate in place to 90 degrees
        mdiff.turn_to_angle(SpeedRPM(40), 90)

        # Disable odometry
        mdiff.odometry_stop()
    """
    def __init__(self,
                 left_motor_port,
                 right_motor_port,
                 wheel_class,
                 wheel_distance_mm,
                 desc=None,
                 motor_class=LargeMotor):

        MoveTank.__init__(self, left_motor_port, right_motor_port, desc, motor_class)
        self.wheel = wheel_class()
        self.wheel_distance_mm = wheel_distance_mm

        # The circumference of the circle made if this robot were to rotate in place
        self.circumference_mm = self.wheel_distance_mm * math.pi

        self.min_circle_radius_mm = self.wheel_distance_mm / 2

        # odometry variables
        self.x_pos_mm = 0.0  # robot X position in mm
        self.y_pos_mm = 0.0  # robot Y position in mm
        self.odometry_thread_run = False
        self.odometry_thread_id = None
        self.theta = 0.0

    def on_for_distance(self, speed, distance_mm, brake=True, block=True):
        """
        Drive distance_mm
        """
        rotations = distance_mm / self.wheel.circumference_mm
        log.debug("%s: on_for_rotations distance_mm %s, rotations %s, speed %s" % (self, distance_mm, rotations, speed))

        MoveTank.on_for_rotations(self, speed, speed, rotations, brake, block)

    def _on_arc(self, speed, radius_mm, distance_mm, brake, block, arc_right):
        """
        Drive in a circle with 'radius' for 'distance'
        """

        if radius_mm < self.min_circle_radius_mm:
            raise ValueError("{}: radius_mm {} is less than min_circle_radius_mm {}".format(
                self, radius_mm, self.min_circle_radius_mm))

        # The circle formed at the halfway point between the two wheels is the
        # circle that must have a radius of radius_mm
        circle_outer_mm = 2 * math.pi * (radius_mm + (self.wheel_distance_mm / 2))
        circle_middle_mm = 2 * math.pi * radius_mm
        circle_inner_mm = 2 * math.pi * (radius_mm - (self.wheel_distance_mm / 2))

        if arc_right:
            # The left wheel is making the larger circle and will move at 'speed'
            # The right wheel is making a smaller circle so its speed will be a fraction of the left motor's speed
            left_speed = speed
            right_speed = float(circle_inner_mm / circle_outer_mm) * left_speed

        else:
            # The right wheel is making the larger circle and will move at 'speed'
            # The left wheel is making a smaller circle so its speed will be a fraction of the right motor's speed
            right_speed = speed
            left_speed = float(circle_inner_mm / circle_outer_mm) * right_speed

        log.debug(
            "%s: arc %s, radius %s, distance %s, left-speed %s, right-speed %s, circle_outer_mm %s, circle_middle_mm %s, circle_inner_mm %s"
            % (self, "right" if arc_right else "left", radius_mm, distance_mm, left_speed, right_speed, circle_outer_mm,
               circle_middle_mm, circle_inner_mm))

        # We know we want the middle circle to be of length distance_mm so
        # calculate the percentage of circle_middle_mm we must travel for the
        # middle of the robot to travel distance_mm.
        circle_middle_percentage = float(distance_mm / circle_middle_mm)

        # Now multiple that percentage by circle_outer_mm to calculate how
        # many mm the outer wheel should travel.
        circle_outer_final_mm = circle_middle_percentage * circle_outer_mm

        outer_wheel_rotations = float(circle_outer_final_mm / self.wheel.circumference_mm)
        outer_wheel_degrees = outer_wheel_rotations * 360

        log.debug(
            "%s: arc %s, circle_middle_percentage %s, circle_outer_final_mm %s, outer_wheel_rotations %s, outer_wheel_degrees %s"
            % (self, "right" if arc_right else "left", circle_middle_percentage, circle_outer_final_mm,
               outer_wheel_rotations, outer_wheel_degrees))

        MoveTank.on_for_degrees(self, left_speed, right_speed, outer_wheel_degrees, brake, block)

    def on_arc_right(self, speed, radius_mm, distance_mm, brake=True, block=True):
        """
        Drive clockwise in a circle with 'radius_mm' for 'distance_mm'
        """
        self._on_arc(speed, radius_mm, distance_mm, brake, block, True)

    def on_arc_left(self, speed, radius_mm, distance_mm, brake=True, block=True):
        """
        Drive counter-clockwise in a circle with 'radius_mm' for 'distance_mm'
        """
        self._on_arc(speed, radius_mm, distance_mm, brake, block, False)

    def _turn(self, speed, degrees, brake=True, block=True):
        """
        Rotate in place 'degrees'. Both wheels must turn at the same speed for us
        to rotate in place.
        """

        # The distance each wheel needs to travel
        distance_mm = (abs(degrees) / 360) * self.circumference_mm

        # The number of rotations to move distance_mm
        rotations = distance_mm / self.wheel.circumference_mm

        log.debug("%s: turn() degrees %s, distance_mm %s, rotations %s, degrees %s" %
                  (self, degrees, distance_mm, rotations, degrees))

        # If degrees is positive rotate clockwise
        if degrees > 0:
            MoveTank.on_for_rotations(self, speed, speed * -1, rotations, brake, block)

        # If degrees is negative rotate counter-clockwise
        else:
            rotations = distance_mm / self.wheel.circumference_mm
            MoveTank.on_for_rotations(self, speed * -1, speed, rotations, brake, block)

    def turn_right(self, speed, degrees, brake=True, block=True):
        """
        Rotate clockwise 'degrees' in place
        """
        self._turn(speed, abs(degrees), brake, block)

    def turn_left(self, speed, degrees, brake=True, block=True):
        """
        Rotate counter-clockwise 'degrees' in place
        """
        self._turn(speed, abs(degrees) * -1, brake, block)

    def odometry_coordinates_log(self):
        log.debug("%s: odometry angle %s at (%d, %d)" % (self, math.degrees(self.theta), self.x_pos_mm, self.y_pos_mm))

    def odometry_start(self, theta_degrees_start=90.0, x_pos_start=0.0, y_pos_start=0.0, sleep_time=0.005):  # 5ms
        """
        Ported from:
        http://seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm

        A thread is started that will run until the user calls odometry_stop()
        which will set odometry_thread_run to False
        """
        def _odometry_monitor():
            left_previous = 0
            right_previous = 0
            self.theta = math.radians(theta_degrees_start)  # robot heading
            self.x_pos_mm = x_pos_start  # robot X position in mm
            self.y_pos_mm = y_pos_start  # robot Y position in mm
            TWO_PI = 2 * math.pi

            while self.odometry_thread_run:

                # sample the left and right encoder counts as close together
                # in time as possible
                left_current = self.left_motor.position
                right_current = self.right_motor.position

                # determine how many ticks since our last sampling
                left_ticks = left_current - left_previous
                right_ticks = right_current - right_previous

                # Have we moved?
                if not left_ticks and not right_ticks:
                    if sleep_time:
                        time.sleep(sleep_time)
                    continue

                # log.debug("%s: left_ticks %s (from %s to %s)" %
                #     (self, left_ticks, left_previous, left_current))
                # log.debug("%s: right_ticks %s (from %s to %s)" %
                #     (self, right_ticks, right_previous, right_current))

                # update _previous for next time
                left_previous = left_current
                right_previous = right_current

                # rotations = distance_mm/self.wheel.circumference_mm
                left_rotations = float(left_ticks / self.left_motor.count_per_rot)
                right_rotations = float(right_ticks / self.right_motor.count_per_rot)

                # convert longs to floats and ticks to mm
                left_mm = float(left_rotations * self.wheel.circumference_mm)
                right_mm = float(right_rotations * self.wheel.circumference_mm)

                # calculate distance we have traveled since last sampling
                mm = (left_mm + right_mm) / 2.0

                # accumulate total rotation around our center
                self.theta += (right_mm - left_mm) / self.wheel_distance_mm

                # and clip the rotation to plus or minus 360 degrees
                self.theta -= float(int(self.theta / TWO_PI) * TWO_PI)

                # now calculate and accumulate our position in mm
                self.x_pos_mm += mm * math.cos(self.theta)
                self.y_pos_mm += mm * math.sin(self.theta)

                if sleep_time:
                    time.sleep(sleep_time)

            self.odometry_thread_id = None

        self.odometry_thread_run = True
        self.odometry_thread_id = _thread.start_new_thread(_odometry_monitor, ())

    def odometry_stop(self):
        """
        Signal the odometry thread to exit and wait for it to exit
        """

        if self.odometry_thread_id:
            self.odometry_thread_run = False

            while self.odometry_thread_id:
                pass

    def turn_to_angle(self, speed, angle_target_degrees, brake=True, block=True):
        """
        Rotate in place to ``angle_target_degrees`` at ``speed``
        """
        assert self.odometry_thread_id, "odometry_start() must be called to track robot coordinates"

        # Make both target and current angles positive numbers between 0 and 360
        if angle_target_degrees < 0:
            angle_target_degrees += 360

        angle_current_degrees = math.degrees(self.theta)

        if angle_current_degrees < 0:
            angle_current_degrees += 360

        # Is it shorter to rotate to the right or left
        # to reach angle_target_degrees?
        if angle_current_degrees > angle_target_degrees:
            turn_right = True
            angle_delta = angle_current_degrees - angle_target_degrees
        else:
            turn_right = False
            angle_delta = angle_target_degrees - angle_current_degrees

        if angle_delta > 180:
            angle_delta = 360 - angle_delta
            turn_right = not turn_right

        log.debug("%s: turn_to_angle %s, current angle %s, delta %s, turn_right %s" %
                  (self, angle_target_degrees, angle_current_degrees, angle_delta, turn_right))
        self.odometry_coordinates_log()

        if turn_right:
            self.turn_right(speed, angle_delta, brake, block)
        else:
            self.turn_left(speed, angle_delta, brake, block)

        self.odometry_coordinates_log()

    def on_to_coordinates(self, speed, x_target_mm, y_target_mm, brake=True, block=True):
        """
        Drive to (``x_target_mm``, ``y_target_mm``) coordinates at ``speed``
        """
        assert self.odometry_thread_id, "odometry_start() must be called to track robot coordinates"

        # stop moving
        self.off(brake='hold')

        # rotate in place so we are pointed straight at our target
        x_delta = x_target_mm - self.x_pos_mm
        y_delta = y_target_mm - self.y_pos_mm
        angle_target_radians = math.atan2(y_delta, x_delta)
        angle_target_degrees = math.degrees(angle_target_radians)
        self.turn_to_angle(speed, angle_target_degrees, brake=True, block=True)

        # drive in a straight line to the target coordinates
        distance_mm = math.sqrt(pow(self.x_pos_mm - x_target_mm, 2) + pow(self.y_pos_mm - y_target_mm, 2))
        self.on_for_distance(speed, distance_mm, brake, block)
