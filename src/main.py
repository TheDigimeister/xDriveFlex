# --------------------------------------------------------------------------------- #
#                                                                                   #
#    Project:           Split Arcade                                                #
#    Module:            main.py                                                     #
#    Author:            VEX                                                         #
#    Created:           Fri Aug 05 2022                                             #
#    Description:       The Left up/down Controller Axis (A) will drive             #
#                       the robot forward and backwards.                            #
#                       The Right left/right Controller Axis (C) will turn          #
#                       the robot left and right.                                   #
#                       The deadband variable prevents drift when                   #
#                       the Controller's joystick is released.                      #
#                                                                                   #
#    Configuration:     Left Motor in Port 1                                        #
#                       Right Motor in Port 6                                       #
#                       Controller                                                  #
#                                                                                   #
# --------------------------------------------------------------------------------- #

# Library imports
from vex import *
import math
# from motions import move_to_target

class PID:
    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.integral = 0
        self.prev_error = 0

    def reset(self):
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kP * error + self.kI * self.integral + self.kD * derivative
        self.prev_error = error
        if abs(error) < 1:
            return 0
        else:
            return output

# PID controllers for lateral and angular velocity
lateral_pid = PID(12.0, 0.0, 0.0)  # Tune these values for your robot
angular_pid = PID(1, 0.0, 0.0)  # Tune these values for your robot

def move_to_target(current_x, current_y, current_heading_deg, target_x, target_y, target_heading_deg, dt):
    # Calculate field-centric error
    dx = target_x - current_x  # x is forward
    dy = target_y - current_y  # y is strafe left
    distance_error = math.sqrt(dx**2 + dy**2)
    angle_to_target = math.atan2(dy, dx)
    heading_rad = current_heading_deg * math.pi / 180.0

    # Transform error to robot-centric frame (standard field-centric math)
    forward_error = dx * math.cos(heading_rad) + dy * math.sin(heading_rad)
    strafe_error = -dx * math.sin(heading_rad) + dy * math.cos(heading_rad)

    # S-curve motion profile (sigmoid)
    def s_curve(error, max_vel=100.0, accel=0.05):
        # accel controls steepness; max_vel is max output
        return max_vel * (2 / (1 + math.exp(-accel * error)) - 1)

    # PID outputs for translation
    forward_cmd = lateral_pid.update(forward_error, dt)
    strafe_cmd = lateral_pid.update(strafe_error, dt)

    # Apply s-curve profile to limit velocity
    forward_cmd = s_curve(forward_cmd)
    strafe_cmd = s_curve(strafe_cmd)

    # PID output for rotation
    heading_error = (target_heading_deg - current_heading_deg + 180) % 360 - 180  # Shortest path
    turn_cmd = angular_pid.update(heading_error, dt)
    turn_cmd = s_curve(turn_cmd)

    return forward_cmd, strafe_cmd, turn_cmd

# Brain should be defined by default
brain = Brain()

# Robot configuration code
brain_inertial = Inertial()  # VEX IQ uses GyroSensor for heading

front_left_motor = Motor(Ports.PORT6, True)
front_right_motor = Motor(Ports.PORT5)
back_left_motor = Motor(Ports.PORT2, True)
back_right_motor = Motor(Ports.PORT4)

left_arm_motor = Motor(Ports.PORT1, True)
right_arm_motor = Motor(Ports.PORT3)

controller = Controller()

front_left_motor.set_stopping(BRAKE)
front_right_motor.set_stopping(BRAKE)
back_left_motor.set_stopping(BRAKE)
back_right_motor.set_stopping(BRAKE)
left_arm_motor.set_stopping(BRAKE)
right_arm_motor.set_stopping(BRAKE)

brain_inertial.calibrate()
while brain_inertial.is_calibrating(): wait(50)

# Begin project code
auton_mode = False  # Set to True for autonomous mode
# Set the deadband variable
dead_band = 5

# Odometry variables
x = 0.0  # Field X position (in)
y = 0.0  # Field Y position (in)

# Tuning constants
wheel_diameter_in = 2.5  # Convert mm to inches
max_rpm = 120  # Change to match your motors
wheel_circumference = math.pi * wheel_diameter_in

# Time step
dt = 0.02  # 20 ms

while True:
    if auton_mode:
        # Autonomous mode: move to target position and heading
        target_x = 7  # Example target X (in)
        target_y = -7  # Example target Y (in)
        target_heading = 180  # Example target heading (deg)

        heading_deg = brain_inertial.heading()
        heading_rad = heading_deg * math.pi / 180.0

        # Odometry update
        fl = front_left_motor.velocity()
        fr = front_right_motor.velocity()
        bl = back_left_motor.velocity()
        br = back_right_motor.velocity()

        # Convert RPM to inches/second for each motor
        fl_in_s = (fl / 60.0) * wheel_circumference
        fr_in_s = (fr / 60.0) * wheel_circumference
        bl_in_s = (bl / 60.0) * wheel_circumference
        br_in_s = (br / 60.0) * wheel_circumference

        # X-drive kinematics: forward and strafe in inches/second
        forward_in_s = (fl_in_s + fr_in_s + bl_in_s + br_in_s) / 4.0
        strafe_in_s = (fl_in_s + br_in_s - fr_in_s - bl_in_s) / 4.0

        # Standard field-centric odometry update (x: forward, y: strafe left)
        x_dot = forward_in_s * math.cos(heading_rad) - strafe_in_s * math.sin(heading_rad)
        y_dot = forward_in_s * math.sin(heading_rad) + strafe_in_s * math.cos(heading_rad)

        x += x_dot * dt  # x is forward
        y += y_dot * dt  # y is strafe left

        # Call motion profile PID
        forward_cmd, strafe_cmd, turn_cmd = move_to_target(x, y, heading_deg, target_x, target_y, target_heading, dt)

        # Cap total translation velocity at 50%
        max_vel = 50.0
        vel_mag = math.sqrt(forward_cmd**2 + strafe_cmd**2)
        if vel_mag > max_vel:
            scale = max_vel / vel_mag
            forward_cmd *= scale
            strafe_cmd *= scale
        # Cap turn velocity to half
        turn_cmd *= 0.5

        # Field-centric to motor speeds
        fl_speed = forward_cmd + strafe_cmd + turn_cmd
        fr_speed = forward_cmd - strafe_cmd - turn_cmd
        bl_speed = forward_cmd - strafe_cmd + turn_cmd
        br_speed = forward_cmd + strafe_cmd - turn_cmd

        front_left_motor.set_velocity(fl_speed)
        front_right_motor.set_velocity(fr_speed)
        back_left_motor.set_velocity(bl_speed)
        back_right_motor.set_velocity(br_speed)

        front_left_motor.spin(FORWARD)
        front_right_motor.spin(FORWARD)
        back_left_motor.spin(FORWARD)
        back_right_motor.spin(FORWARD)

        # Print to brain display
        brain.screen.clear_screen()
        brain.screen.print_at(" X: {:.2f} in".format(x), x=0, y=20)
        brain.screen.print_at(" Y: {:.2f} in".format(y), x=0, y=40)
        brain.screen.print_at(" Heading: {:.1f} deg".format(heading_deg), x=0, y=60)

        wait(int(dt * 1000))
    else:
        # Manual mode
        # ...existing code for manual control and odometry...
        axis_a_pos = controller.axisA.position()  # Left stick up/down
        axis_b_pos = controller.axisB.position()  # Left stick left/right
        axis_c_pos = controller.axisC.position()  # Right stick left/right

        heading_deg = brain_inertial.heading()
        heading_rad = heading_deg * math.pi / 180.0

        field_forward = axis_a_pos * math.cos(heading_rad) + axis_b_pos * math.sin(heading_rad)
        field_strafe  = -axis_a_pos * math.sin(heading_rad) + axis_b_pos * math.cos(heading_rad)

        if abs(field_forward) + abs(field_strafe) + abs(axis_c_pos) > dead_band:
            # Cap total translation velocity at 50%
            max_vel = 50.0
            vel_mag = math.sqrt(field_forward**2 + field_strafe**2)
            if vel_mag > max_vel:
                scale = max_vel / vel_mag
                field_forward *= scale
                field_strafe *= scale
            # Cap turn velocity to half
            axis_c_pos *= 0.5

            fl_speed = field_forward + field_strafe + axis_c_pos
            fr_speed = field_forward - field_strafe - axis_c_pos
            bl_speed = field_forward - field_strafe + axis_c_pos
            br_speed = field_forward + field_strafe - axis_c_pos

            front_left_motor.set_velocity(fl_speed)
            front_right_motor.set_velocity(fr_speed)
            back_left_motor.set_velocity(bl_speed)
            back_right_motor.set_velocity(br_speed)
        else:
            front_left_motor.set_velocity(0)
            front_right_motor.set_velocity(0)
            back_left_motor.set_velocity(0)
            back_right_motor.set_velocity(0)

            front_left_motor.stop()
            front_right_motor.stop()
            back_left_motor.stop()
            back_right_motor.stop()

        front_left_motor.spin(FORWARD)
        front_right_motor.spin(FORWARD)
        back_left_motor.spin(FORWARD)
        back_right_motor.spin(FORWARD)

        # Odometry update (same as autonomous mode)
        fl = front_left_motor.velocity()
        fr = front_right_motor.velocity()
        bl = back_left_motor.velocity()
        br = back_right_motor.velocity()

        # Convert RPM to inches/second for each motor
        fl_in_s = (fl / 60.0) * wheel_circumference
        fr_in_s = (fr / 60.0) * wheel_circumference
        bl_in_s = (bl / 60.0) * wheel_circumference
        br_in_s = (br / 60.0) * wheel_circumference

        # X-drive kinematics: forward and strafe in inches/second
        forward_in_s = (fl_in_s + fr_in_s + bl_in_s + br_in_s) / 4.0
        strafe_in_s = (fl_in_s + br_in_s - fr_in_s - bl_in_s) / 4.0

        # Standard field-centric odometry update
        x_dot = forward_in_s * math.cos(heading_rad) + strafe_in_s * math.sin(heading_rad)
        y_dot = -forward_in_s * math.sin(heading_rad) + strafe_in_s * math.cos(heading_rad)

        x += x_dot * dt  # x is forward
        y += y_dot * dt  # y is strafe left

        # Print to brain display
        brain.screen.clear_screen()
        brain.screen.print_at(" X: {:.2f} in".format(x), x=0, y=20)
        brain.screen.print_at(" Y: {:.2f} in".format(y), x=0, y=40)
        brain.screen.print_at(" Heading: {:.1f} deg".format(heading_deg), x=0, y=60)

        # Arm code

        if controller.buttonRUp.pressing():
            left_arm_motor.spin(REVERSE)
            right_arm_motor.spin(REVERSE)
        elif controller.buttonRDown.pressing():
            left_arm_motor.spin(FORWARD)
            right_arm_motor.spin(FORWARD)
        elif controller.buttonLUp.pressing():
            left_arm_motor.spin(FORWARD)
            right_arm_motor.spin(REVERSE)
        elif controller.buttonLDown.pressing():
            left_arm_motor.spin(REVERSE)
            right_arm_motor.spin(FORWARD)
        else:
            left_arm_motor.stop()
            right_arm_motor.stop()

        wait(int(dt * 1000))
