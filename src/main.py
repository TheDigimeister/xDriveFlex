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

# Brain should be defined by default
brain = Brain()

# Robot configuration code
brain_inertial = Inertial()  # VEX IQ uses GyroSensor for heading
front_left_motor = Motor(Ports.PORT1, True)
front_right_motor = Motor(Ports.PORT6)
back_left_motor = Motor(Ports.PORT7, True)
back_right_motor = Motor(Ports.PORT12)
controller = Controller()

# Begin project code
auton_mode = False  # Set to True for autonomous mode
# Set the deadband variable
dead_band = 5

# Odometry variables
x = 0.0  # Field X position (mm)
y = 0.0  # Field Y position (mm)

# Tuning constants
wheel_diameter_mm = 100  # Change to match your wheels
max_rpm = 200  # Change to match your motors
wheel_circumference = math.pi * wheel_diameter_mm

# Time step
dt = 0.02  # 20 ms

while True:
    if auton_mode:
        pass
        # # Autonomous mode: move to target position and heading
        # target_x = 1000  # Example target X (mm)
        # target_y = 1000  # Example target Y (mm)
        # target_heading = 90  # Example target heading (deg)

        # heading_deg = brain_inertial.heading()
        # heading_rad = heading_deg * math.pi / 180.0

        # # Odometry update
        # fl = front_left_motor.velocity()
        # fr = front_right_motor.velocity()
        # bl = back_left_motor.velocity()
        # br = back_right_motor.velocity()

        # max_speed_mm_s = (max_rpm / 60.0) * wheel_circumference
        # forward = (fl + fr + bl + br) / 4.0
        # strafe = (fl + br - fr - bl) / 4.0
        # forward_mm_s = forward / 100.0 * max_speed_mm_s
        # strafe_mm_s = strafe / 100.0 * max_speed_mm_s

        # dx = forward_mm_s * math.cos(heading_rad) + strafe_mm_s * math.sin(heading_rad)
        # dy = -forward_mm_s * math.sin(heading_rad) + strafe_mm_s * math.cos(heading_rad)

        # x += dx * dt
        # y += dy * dt

        # # Call motion profile PID
        # forward_cmd, strafe_cmd, turn_cmd = move_to_target(x, y, heading_deg, target_x, target_y, target_heading, dt)

        # # Field-centric to motor speeds
        # fl_speed = forward_cmd + strafe_cmd + turn_cmd
        # fr_speed = forward_cmd - strafe_cmd - turn_cmd
        # bl_speed = forward_cmd - strafe_cmd + turn_cmd
        # br_speed = forward_cmd + strafe_cmd - turn_cmd

        # front_left_motor.set_velocity(fl_speed)
        # front_right_motor.set_velocity(fr_speed)
        # back_left_motor.set_velocity(bl_speed)
        # back_right_motor.set_velocity(br_speed)

        # front_left_motor.spin(FORWARD)
        # front_right_motor.spin(FORWARD)
        # back_left_motor.spin(FORWARD)
        # back_right_motor.spin(FORWARD)

        # # Print to brain display
        # brain.screen.clear_screen()
        # brain.screen.print("X: {:.1f} mm".format(x))
        # brain.screen.print(" Y: {:.1f} mm".format(y))
        # brain.screen.print(" Heading: {:.1f} deg".format(heading_deg))

        # wait(int(dt * 1000))
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

        max_speed_mm_s = (max_rpm / 60.0) * wheel_circumference
        forward = (fl + fr + bl + br) / 4.0
        strafe = (fl + br - fr - bl) / 4.0
        forward_mm_s = forward / 100.0 * max_speed_mm_s
        strafe_mm_s = strafe / 100.0 * max_speed_mm_s

        dx = forward_mm_s * math.cos(heading_rad) + strafe_mm_s * math.sin(heading_rad)
        dy = -forward_mm_s * math.sin(heading_rad) + strafe_mm_s * math.cos(heading_rad)

        x += dx * dt
        y += dy * dt

        # Print to brain display
        brain.screen.clear_screen()
        brain.screen.print_at(" X: {:.1f} mm".format(x), x=0, y=20)
        brain.screen.print_at(" Y: {:.1f} mm".format(y), x=0, y=40)
        brain.screen.print_at(" Heading: {:.1f} deg".format(heading_deg), x=0, y=60)

        wait(int(dt * 1000))
