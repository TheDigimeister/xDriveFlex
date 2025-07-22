from vex import *
import math

# Assume motors and inertial sensor are initialized in main.py and imported here
# You may need to adjust these imports if using classes or different structure
brain = Brain()
front_left_motor = Motor(Ports.PORT1, False)
front_right_motor = Motor(Ports.PORT6, True)
back_left_motor = Motor(Ports.PORT7, False)
back_right_motor = Motor(Ports.PORT8, True)
brain_inertial = Inertial()

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
    # Get heading in radians
    heading_deg = brain_inertial.rotation()
    heading_rad = heading_deg * math.pi / 180.0

    # Get average velocities (percent)
    fl = front_left_motor.velocity(PERCENT)
    fr = front_right_motor.velocity(PERCENT)
    bl = back_left_motor.velocity(PERCENT)
    br = back_right_motor.velocity(PERCENT)

    # Convert percent velocity to mm/s
    avg_percent = (abs(fl) + abs(fr) + abs(bl) + abs(br)) / 4.0
    max_speed_mm_s = (max_rpm / 60.0) * wheel_circumference
    speed_mm_s = avg_percent / 100.0 * max_speed_mm_s

    # Estimate robot movement in local frame
    # For X-drive, forward = average of all motors, strafe = (FL + BR) - (FR + BL)
    forward = (fl + fr + bl + br) / 4.0
    strafe = (fl + br - fr - bl) / 4.0
    forward_mm_s = forward / 100.0 * max_speed_mm_s
    strafe_mm_s = strafe / 100.0 * max_speed_mm_s

    # Rotate to field frame
    dx = forward_mm_s * math.cos(heading_rad) + strafe_mm_s * math.sin(heading_rad)
    dy = -forward_mm_s * math.sin(heading_rad) + strafe_mm_s * math.cos(heading_rad)

    # Integrate position
    x += dx * dt
    y += dy * dt

    # Print to brain display
    brain.screen.clear_screen()
    brain.screen.print("X: {:.1f} mm".format(x))
    brain.screen.new_line()
    brain.screen.print("Y: {:.1f} mm".format(y))
    brain.screen.new_line()
    brain.screen.print("Heading: {:.1f} deg".format(heading_deg))

    wait(int(dt * 1000), MSEC)
