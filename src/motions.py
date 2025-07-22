from vex import *
import math

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
lateral_pid = PID(0.5, 0.0, 0.1)  # Tune these values for your robot
angular_pid = PID(1.0, 0.0, 0.2)  # Tune these values for your robot

def move_to_target(current_x, current_y, current_heading_deg, target_x, target_y, target_heading_deg, dt):
    # Calculate field-centric error
    dx = target_x - current_x
    dy = target_y - current_y
    distance_error = math.sqrt(dx**2 + dy**2)
    angle_to_target = math.atan2(dy, dx)
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

# Example usage:
# forward, strafe, turn = move_to_target(x, y, heading_deg, target_x, target_y, target_heading_deg, dt)
# Use these outputs in your drive code
