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
        return output

# PID controllers for lateral and angular velocity
lateral_pid = PID(0.1, 0.0, 0.0)  # Tune these values for your robot
angular_pid = PID(1, 0.0, 0.0)  # Tune these values for your robot

def move_to_target(current_x, current_y, current_heading_deg, target_x, target_y, target_heading_deg, dt):
    # Calculate field-centric error
    dx = target_x - current_x
    dy = target_y - current_y
    distance_error = math.sqrt(dx**2 + dy**2)
    angle_to_target = math.atan2(dx, dy)
    heading_rad = current_heading_deg * math.pi / 180.0
    # Transform error to robot-centric frame
    forward_error = distance_error * math.cos(angle_to_target - heading_rad)
    strafe_error = distance_error * math.sin(angle_to_target - heading_rad)

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
    turn_cmd = s_curve(turn_cmd, max_vel=100.0, accel=0.05)

    return forward_cmd, strafe_cmd, turn_cmd

# Brain should be defined by default
brain = Brain()

# Robot configuration code
brain_inertial = Inertial()  # VEX IQ uses GyroSensor for heading
front_left_motor = Motor(Ports.PORT1)
front_right_motor = Motor(Ports.PORT6, True)
back_left_motor = Motor(Ports.PORT7)
back_right_motor = Motor(Ports.PORT12, True)
controller = Controller()

brain_inertial.calibrate()
while brain_inertial.is_calibrating(): wait(50)

# Begin project code
auton_mode = True  # Set to True for autonomous mode
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
        target_x = 24  # Example target X (in)
        target_y = 0  # Example target Y (in)
        target_heading = 90  # Example target heading (deg)

        heading_deg = brain_inertial.heading()
        heading_rad = heading_deg * math.pi / 180.0

        # Odometry update
        fl = front_left_motor.velocity()
        fr = front_right_motor.velocity()
        bl = back_left_motor.velocity()
        br = back_right_motor.velocity()

        max_speed_in_s = (max_rpm / 60.0) * wheel_circumference
        forward = (fl + fr + bl + br) * 0.75 / 60.0 * wheel_circumference
        strafe = (fl + br - fr - bl) * 0.75 / 60.0 * wheel_circumference
        forward_in_s = forward / 100.0 * max_speed_in_s
        strafe_in_s = strafe / 100.0 * max_speed_in_s

        dx = forward * math.cos(heading_rad) + strafe * math.sin(heading_rad)
        dy = -forward * math.sin(heading_rad) + strafe * math.cos(heading_rad)

        x += dx * dt
        y += dy * dt

        # Call motion profile PID
        forward_cmd, strafe_cmd, turn_cmd = move_to_target(x, y, heading_deg, target_x, target_y, target_heading, dt)

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

        front_left_motor.spin(FORWARD)
        front_right_motor.spin(FORWARD)
        back_left_motor.spin(FORWARD)
        back_right_motor.spin(FORWARD)

        # Odometry update
        fl = front_left_motor.velocity()
        fr = front_right_motor.velocity()
        bl = back_left_motor.velocity()
        br = back_right_motor.velocity()

        max_speed_in_s = (max_rpm / 60.0) * wheel_circumference
        forward = (fl + fr + bl + br) * 0.75 / 60.0 * wheel_circumference
        strafe = (fl + br - fr - bl) * 0.75 / 60.0 * wheel_circumference
        forward_in_s = forward / 100.0 * max_speed_in_s
        strafe_in_s = strafe / 100.0 * max_speed_in_s

        dx = forward * math.cos(heading_rad) + strafe * math.sin(heading_rad)
        dy = -forward * math.sin(heading_rad) + strafe * math.cos(heading_rad)

        x += dx * dt
        y += dy * dt

        # Print to brain display
        brain.screen.clear_screen()
        brain.screen.print_at(" X: {:.2f} in".format(x), x=0, y=20)
        brain.screen.print_at(" Y: {:.2f} in".format(y), x=0, y=40)
        brain.screen.print_at(" Heading: {:.1f} deg".format(heading_deg), x=0, y=60)

        wait(int(dt * 1000))
