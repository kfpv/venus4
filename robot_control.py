from st2 import Stepper
import time
import math
import termios
import tty
import sys
import threading
import os
import signal
import select
RELEASE_THRESHOLD = 0.6
class RobotController:
    WHEEL_RADIUS_CM = 4
    WHEEL_BASE_CM = 12.5
    STEPS_PER_REV = 1600
    RAMP_STEPS = 0
    def __init__(self, wheel_radius_cm=None, wheel_base_cm=None,
                 turn_ccw_correction_factor=None, turn_cw_correction_factor=None,
                 distance_correction_factor=None, left_understep_percent=None,
                 speed_multiplier=None):
        self.stepper = Stepper()
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.left_step_count = 0
        self.right_step_count = 0
        self.moving = False
        self.WHEEL_RADIUS_CM = wheel_radius_cm if wheel_radius_cm is not None else 4.0
        self.WHEEL_BASE_CM = wheel_base_cm if wheel_base_cm is not None else 12.5
        self.STEPS_PER_REV = 1600
        self.CM_PER_STEP = 2 * math.pi * self.WHEEL_RADIUS_CM / self.STEPS_PER_REV
        self.RAMP_STEPS = 0
        self.TURN_CORRECTION_FACTOR_CCW = turn_ccw_correction_factor if turn_ccw_correction_factor is not None else 1.0159
        self.TURN_CORRECTION_FACTOR_CW = turn_cw_correction_factor if turn_cw_correction_factor is not None else 1.0154
        self.DISTANCE_CORRECTION_FACTOR = distance_correction_factor if distance_correction_factor is not None else 1.02
        self.left_understep_percent = left_understep_percent if left_understep_percent is not None else -0.5
        self.speed_period = Stepper.MAX_PERIOD
        self.speed_multiplier = 1.0
        final_speed_multiplier = speed_multiplier if speed_multiplier is not None else 1.0
        self.set_speed_multiplier(final_speed_multiplier)
        self.current_direction = None
        max_turn_factor = max(self.TURN_CORRECTION_FACTOR_CCW, self.TURN_CORRECTION_FACTOR_CW)
        self.MIN_TURN_ANGLE_RAD = (2 * self.CM_PER_STEP) / self.WHEEL_BASE_CM / max_turn_factor
        self.MIN_TURN_ANGLE_DEG = math.degrees(self.MIN_TURN_ANGLE_RAD)
        print(f"RobotController initialized with:")
        print(f"  Wheel radius: {self.WHEEL_RADIUS_CM:.2f}cm")
        print(f"  Wheel base: {self.WHEEL_BASE_CM:.2f}cm")
        print(f"  Speed multiplier: {self.speed_multiplier:.2f}")
        print(f"  Turn correction factors: CCW={self.TURN_CORRECTION_FACTOR_CCW:.4f}, CW={self.TURN_CORRECTION_FACTOR_CW:.4f}")
        print(f"  Distance correction factor: {self.DISTANCE_CORRECTION_FACTOR:.4f}")
        print(f"  Left understep: {self.left_understep_percent:.2f}%")
        print(f"  MIN_TURN_ANGLE: {self.MIN_TURN_ANGLE_DEG:.3f}°")
    def update_physical_parameters(self, wheel_radius_cm, wheel_base_cm):
        old_wheel_radius = self.WHEEL_RADIUS_CM
        old_wheel_base = self.WHEEL_BASE_CM
        old_cm_per_step = self.CM_PER_STEP
        self.WHEEL_RADIUS_CM = wheel_radius_cm
        self.WHEEL_BASE_CM = wheel_base_cm
        self.CM_PER_STEP = 2 * math.pi * self.WHEEL_RADIUS_CM / self.STEPS_PER_REV
        self.MIN_TURN_ANGLE_RAD = (2 * self.CM_PER_STEP) / self.WHEEL_BASE_CM * self.TURN_CORRECTION_FACTOR
        self.MIN_TURN_ANGLE_DEG = math.degrees(self.MIN_TURN_ANGLE_RAD)
        print(f"Physical parameters updated:")
        print(f"  Wheel radius: {old_wheel_radius:.2f}cm -> {wheel_radius_cm:.2f}cm")
        print(f"  Wheel base: {old_wheel_base:.2f}cm -> {wheel_base_cm:.2f}cm")
        print(f"  CM_PER_STEP: {old_cm_per_step:.6f} -> {self.CM_PER_STEP:.6f}")
        print(f"  New MIN_TURN_ANGLE: {self.MIN_TURN_ANGLE_RAD:.4f}rad, {self.MIN_TURN_ANGLE_DEG:.4f}°")
    def set_speed_multiplier(self, multiplier):
        multiplier = max(1.0, min(100.0, multiplier))
        old_multiplier = self.speed_multiplier
        self.speed_multiplier = multiplier
        self.speed_period = int(Stepper.MAX_PERIOD / multiplier)
        self.speed_period = max(Stepper.MIN_PERIOD, self.speed_period)
        self.stepper.set_speed(self.speed_period, self.speed_period)
        print(f"Speed multiplier updated: {old_multiplier:.2f} -> {multiplier:.2f}")
        print(f"Speed period: {self.speed_period} (MIN={Stepper.MIN_PERIOD}, MAX={Stepper.MAX_PERIOD})")
    def update_correction_factors(self, turn_factor_ccw=None, turn_factor_cw=None, distance_factor=None, left_understep_percent=None):
        old_turn_ccw = self.TURN_CORRECTION_FACTOR_CCW
        old_turn_cw = self.TURN_CORRECTION_FACTOR_CW
        old_distance = self.DISTANCE_CORRECTION_FACTOR
        old_understep = self.left_understep_percent
        if turn_factor_ccw is not None:
            self.TURN_CORRECTION_FACTOR_CCW = turn_factor_ccw
        if turn_factor_cw is not None:
            self.TURN_CORRECTION_FACTOR_CW = turn_factor_cw
        if distance_factor is not None:
            self.DISTANCE_CORRECTION_FACTOR = distance_factor
        if left_understep_percent is not None:
            self.left_understep_percent = left_understep_percent
        max_turn_factor = max(self.TURN_CORRECTION_FACTOR_CCW, self.TURN_CORRECTION_FACTOR_CW)
        self.MIN_TURN_ANGLE_RAD = (2 * self.CM_PER_STEP) / self.WHEEL_BASE_CM / max_turn_factor
        self.MIN_TURN_ANGLE_DEG = math.degrees(self.MIN_TURN_ANGLE_RAD)
        print(f"Correction factors updated:")
        if turn_factor_ccw is not None:
            print(f"  Turn CCW: {old_turn_ccw:.4f} -> {turn_factor_ccw:.4f}")
        if turn_factor_cw is not None:
            print(f"  Turn CW: {old_turn_cw:.4f} -> {turn_factor_cw:.4f}")
        if distance_factor is not None:
            print(f"  Distance: {old_distance:.4f} -> {distance_factor:.4f}")
        if left_understep_percent is not None:
            print(f"  Left understep: {old_understep:.2f}% -> {left_understep_percent:.2f}%")
        print(f"  New MIN_TURN_ANGLE: {self.MIN_TURN_ANGLE_RAD:.4f}rad, {self.MIN_TURN_ANGLE_DEG:.4f}°")
    def reset_position(self):
        if self.moving:
            self.stop()
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.left_step_count = 0
        self.right_step_count = 0
        print("Robot position reset to origin (0, 0, 0°)")
    def start(self):
        self.stepper.enable()
        print("Robot controller initialized")
        print("Position: ({:.2f}, {:.2f}) cm, Heading: {:.2f}°".format(
            self.x, self.y, self.heading))
        print("\nControls:")
        print("W - Move forward")
        print("S - Move backward")
        print("A - Turn left")
        print("D - Turn right")
        print("Space - Stop")
        print("Q - Quit")
        print("+ - Increase speed")
        print("- - Decrease speed")
    def stop(self, force_reset=True):
        if self.moving:
            rem_left, rem_right = self.stepper.get_remaining_steps()
            left_steps_moved = self.left_step_count - rem_left
            right_steps_moved = self.right_step_count - rem_right
            self._update_position(left_steps_moved, right_steps_moved)
            if force_reset:
                self.stepper.reset()
                self.stepper.enable()
                print("Robot stopped (real stop - forced)")
            else:
                print("Robot stopped (virtual stop - movement completed)")
            self.left_step_count = 0
            self.right_step_count = 0
            self.moving = False
            print("Position: ({:.2f}, {:.2f}) cm, Heading: {:.2f}°".format(
                self.x, self.y, self.heading))
    def move(self, direction):
        if self.moving:
            self.stop()
        if direction == 'forward':
            self.left_step_count = 10000
            self.right_step_count = 10000
            self.stepper.set_steps(self.left_step_count, self.right_step_count)
        elif direction == 'backward':
            self.left_step_count = -10000
            self.right_step_count = -10000
            self.stepper.set_steps(self.left_step_count, self.right_step_count)
        elif direction == 'left':
            self.left_step_count = -5000
            self.right_step_count = 5000
            self.stepper.set_steps(self.left_step_count, self.right_step_count)
        elif direction == 'right':
            self.left_step_count = 5000
            self.right_step_count = -5000
            self.stepper.set_steps(self.left_step_count, self.right_step_count)
        self.moving = True
        self.current_direction = direction
    def _update_position(self, left_steps, right_steps):
        left_dist = left_steps * self.CM_PER_STEP / self.DISTANCE_CORRECTION_FACTOR
        right_dist = right_steps * self.CM_PER_STEP / self.DISTANCE_CORRECTION_FACTOR
        wheel_diff = left_steps - right_steps
        understep_flag=0
        if abs(wheel_diff) > 0.1:
            if wheel_diff > 0:
                turn_correction = self.TURN_CORRECTION_FACTOR_CW
            else:
                turn_correction = self.TURN_CORRECTION_FACTOR_CCW
        else:
            turn_correction = (self.TURN_CORRECTION_FACTOR_CCW + self.TURN_CORRECTION_FACTOR_CW) / 2
            left_dist=left_dist*(1.0-self.left_understep_percent/100.0)
            print("multiplied by ",1.0-self.left_understep_percent/100.0)
            understep_flag=1
        if understep_flag==0:
            left_dist_h = left_steps * self.CM_PER_STEP / turn_correction
        else:
            left_dist_h = left_steps * self.CM_PER_STEP / turn_correction*(1.0-self.left_understep_percent/100.0)
        right_dist_h = right_steps * self.CM_PER_STEP / turn_correction
        center_dist = (left_dist + right_dist) / 2
        heading_change_rad = (left_dist_h - right_dist_h) / self.WHEEL_BASE_CM
        old_heading_rad = math.radians(self.heading)
        new_heading_rad = old_heading_rad + heading_change_rad
        self.heading = math.degrees(new_heading_rad) % 360
        avg_heading_rad = old_heading_rad + heading_change_rad / 2
        self.x += center_dist * math.cos(avg_heading_rad)
        self.y += center_dist * math.sin(avg_heading_rad)
    def set_speed(self, faster=True):
        if faster:
            new_multiplier = max(0.1, self.speed_multiplier * 0.8)
            print(f"Speed increased (multiplier: {self.speed_multiplier:.2f} -> {new_multiplier:.2f})")
        else:
            new_multiplier = min(10.0, self.speed_multiplier * 1.2)
            print(f"Speed decreased (multiplier: {self.speed_multiplier:.2f} -> {new_multiplier:.2f})")
        self.set_speed_multiplier(new_multiplier)
    def shutdown(self):
        try:
            self.stop()
            if hasattr(self, 'stepper') and self.stepper._initialised:
                self.stepper.disable()
                self.stepper.destroy()
            print("Robot controller shutdown")
        except Exception as e:
            print(f"Warning: Error during shutdown: {e}")
    def move_distance(self, distance_cm):
        if self.moving:
            self.stop(force_reset=True)
        steps_needed = int(abs(distance_cm) / self.CM_PER_STEP* self.DISTANCE_CORRECTION_FACTOR)
        original_speed = self.speed_period
        if distance_cm >= 0:
            self.left_step_count = steps_needed
            self.right_step_count = steps_needed
            print(f"Moving forward {distance_cm:.2f}cm ({steps_needed} steps)")
        else:
            self.left_step_count = -steps_needed
            self.right_step_count = -steps_needed
            print(f"Moving backward {abs(distance_cm):.2f}cm ({steps_needed} steps)")
        self.moving = True
        self.current_direction = "distance_move"
        self.stepper.set_steps(self.left_step_count, self.right_step_count)
        while not self.stepper.steps_done() and self.moving:
            rem_left, rem_right = self.stepper.get_remaining_steps()
            remaining = min(abs(rem_left), abs(rem_right))
            self._apply_speed_ramp(remaining)
            time.sleep(0.1)
        self.stepper.set_speed(int(original_speed), int(original_speed))
        if self.moving:
            self.stop(force_reset=False)
            print(f"Distance move complete. New position: ({self.x:.2f}, {self.y:.2f}) cm")
        else:
            print("Distance move interrupted")
    def _apply_speed_ramp(self, remaining_steps):
        return False
        if remaining_steps <= self.RAMP_STEPS:
            ramp_factor = remaining_steps / self.RAMP_STEPS
            ramped_period = int(self.speed_period + (Stepper.MAX_PERIOD - self.speed_period) * (1 - ramp_factor))
            self.stepper.set_speed(ramped_period, ramped_period)
            print(f"Speed ramped to {ramped_period} steps")
            return True
        return False
    def turn_angle(self, angle_degrees):
        if self.moving:
            self.stop(force_reset=True)
        if abs(angle_degrees) < self.MIN_TURN_ANGLE_DEG:
            print(f"Turn angle {angle_degrees:.3f}° too small (min: {self.MIN_TURN_ANGLE_DEG:.3f}°), skipping turn.")
            return
        target_heading = (self.heading + angle_degrees) % 360
        if angle_degrees > 0:
            turn_direction = "CW (right)"
            turn_correction_factor = self.TURN_CORRECTION_FACTOR_CW
        else:
            turn_direction = "CCW (left)"
            turn_correction_factor = self.TURN_CORRECTION_FACTOR_CCW
        print(f"Turning {angle_degrees:.2f}° {turn_direction} from {self.heading:.2f}° to {target_heading:.2f}°")
        print(f"Using turn correction factor: {turn_correction_factor:.3f}")
        steps_per_full_turn = (self.WHEEL_BASE_CM / (2 * self.WHEEL_RADIUS_CM)) * self.STEPS_PER_REV * turn_correction_factor
        steps_per_degree = steps_per_full_turn / 360
        turn_steps = int(abs(angle_degrees) * steps_per_degree)
        turn_steps = max(1, turn_steps)
        original_speed = self.speed_period
        if angle_degrees > 0:
            self.left_step_count = turn_steps
            self.right_step_count = -turn_steps
            print(f"Turning right {turn_steps} steps")
        else:
            self.left_step_count = -turn_steps
            self.right_step_count = turn_steps
            print(f"Turning left {turn_steps} steps")
        self.moving = True
        self.current_direction = "angle_turn"
        self.stepper.set_steps(self.left_step_count, self.right_step_count)
        while not self.stepper.steps_done() and self.moving:
            rem_left, rem_right = self.stepper.get_remaining_steps()
            remaining = min(abs(rem_left), abs(rem_right))
            self._apply_speed_ramp(remaining)
            time.sleep(0.1)
        self.stepper.set_speed(int(original_speed), int(original_speed))
        if self.moving:
            self.stop(force_reset=False)
            print(f"Turn complete. New heading: {self.heading:.2f}°")
        else:
            print("Turn interrupted")
    def go_to_origin(self):
        if self.moving:
            self.stop()
        distance_to_origin = math.sqrt(self.x**2 + self.y**2)
        if distance_to_origin < 0.5:
            print("Already at origin.")
            if abs(self.heading) > 1:
                self._turn_to_heading(0)
            return
        angle_to_origin = math.degrees(math.atan2(-self.y, -self.x)) % 360
        print(f"Current position: ({self.x:.2f}, {self.y:.2f}), Heading: {self.heading:.2f}°")
        print(f"Distance to origin: {distance_to_origin:.2f} cm")
        print(f"Angle to origin: {angle_to_origin:.2f}°")
        self._turn_to_heading(angle_to_origin)
        print(f"Moving {distance_to_origin:.2f}cm toward origin")
        self.move_distance(distance_to_origin)
        self._turn_to_heading(0)
        print(f"Navigation complete. Final position: ({self.x:.2f}, {self.y:.2f}), Heading: {self.heading:.2f}°")
    def test_single_wheel_rotation_left(self):
        if self.moving:
            self.stop(force_reset=True)
        print("Starting single left wheel 360° rotation test...")
        print("Robot will rotate around its right wheel (left wheel moves, right wheel stationary)")
        rotation_circumference = 2 * math.pi * self.WHEEL_BASE_CM
        corrected_distance = rotation_circumference * self.TURN_CORRECTION_FACTOR_CW
        steps_needed = int(corrected_distance / self.CM_PER_STEP)
        print(f"Left wheel will move {corrected_distance:.2f}cm ({steps_needed} steps)")
        print(f"Expected rotation: 360° clockwise around right wheel")
        original_speed = self.speed_period
        test_speed = int(Stepper.MAX_PERIOD / 1.2)
        self.stepper.set_speed(test_speed, test_speed)
        self.left_step_count = steps_needed
        self.right_step_count = 0
        self.moving = True
        self.current_direction = "test_left_wheel_rotation"
        self.stepper.set_steps(self.left_step_count, self.right_step_count)
        print("Left wheel rotation test in progress...")
        while not self.stepper.steps_done() and self.moving:
            time.sleep(0.1)
        self.stepper.set_speed(original_speed, original_speed)
        if self.moving:
            self.stop(force_reset=False)
            print(f"Left wheel rotation test complete!")
            print(f"Final position: ({self.x:.2f}, {self.y:.2f}) cm, Heading: {self.heading:.2f}°")
            print(f"Expected: Should have rotated ~360° clockwise from starting position")
        else:
            print("Left wheel rotation test interrupted")
    def test_single_wheel_rotation_right(self):
        if self.moving:
            self.stop(force_reset=True)
        print("Starting single right wheel 360° rotation test...")
        print("Robot will rotate around its left wheel (right wheel moves, left wheel stationary)")
        rotation_circumference = 2 * math.pi * self.WHEEL_BASE_CM
        corrected_distance = rotation_circumference * self.TURN_CORRECTION_FACTOR_CCW
        steps_needed = int(corrected_distance / self.CM_PER_STEP)
        print(f"Right wheel will move {corrected_distance:.2f}cm ({steps_needed} steps)")
        print(f"Expected rotation: 360° counter-clockwise around left wheel")
        original_speed = self.speed_period
        test_speed = int(Stepper.MAX_PERIOD / 1.2)
        self.stepper.set_speed(test_speed, test_speed)
        self.left_step_count = 0
        self.right_step_count = steps_needed
        self.moving = True
        self.current_direction = "test_right_wheel_rotation"
        self.stepper.set_steps(self.left_step_count, self.right_step_count)
        print("Right wheel rotation test in progress...")
        while not self.stepper.steps_done() and self.moving:
            time.sleep(0.1)
        self.stepper.set_speed(original_speed, original_speed)
        if self.moving:
            time.sleep(0.3)
            self.stop(force_reset=False)
            print(f"Right wheel rotation test complete!")
            print(f"Final position: ({self.x:.2f}, {self.y:.2f}) cm, Heading: {self.heading:.2f}°")
            print(f"Expected: Should have rotated ~360° counter-clockwise from starting position")
        else:
            print("Right wheel rotation test interrupted")
    def _turn_to_heading(self, target_heading):
        turn_angle = target_heading - self.heading
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360
        if turn_angle > 0:
            turn_direction = "CW (right)"
            turn_correction_factor = self.TURN_CORRECTION_FACTOR_CW
        else:
            turn_direction = "CCW (left)"
            turn_correction_factor = self.TURN_CORRECTION_FACTOR_CCW
        print(f"Turning {turn_angle:.2f}° {turn_direction} to heading {target_heading:.2f}°")
        print(f"Using turn correction factor: {turn_correction_factor:.3f}")
        if abs(turn_angle) < self.MIN_TURN_ANGLE_DEG:
            print(f"Turn angle {turn_angle:.3f}° too small (min: {self.MIN_TURN_ANGLE_DEG:.3f}°), skipping turn.")
            return
        steps_per_full_turn = (self.WHEEL_BASE_CM / (2 * self.WHEEL_RADIUS_CM)) * self.STEPS_PER_REV * turn_correction_factor
        steps_per_degree = steps_per_full_turn / 360
        turn_steps = int(abs(turn_angle) * steps_per_degree)
        turn_steps = max(1, turn_steps)
        if turn_angle > 0:
            self.left_step_count = turn_steps
            self.right_step_count = -turn_steps
            print(f"Turning right {turn_steps} steps")
        else:
            self.left_step_count = -turn_steps
            self.right_step_count = turn_steps
            print(f"Turning left {turn_steps} steps")
        self.moving = True
        self.current_direction = "turning"
        self.stepper.set_steps(self.left_step_count, self.right_step_count)
        while not self.stepper.steps_done() and self.moving:
            time.sleep(0.1)
        if self.moving:
            self.stop(force_reset=False)
            print(f"Turn complete. New heading: {self.heading:.2f}°")
def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None
def key_listener(robot):
    print("Press Q to exit")
    print("Press WASD keys to move, Space to stop")
    print("Use + to increase speed, - to decrease speed")
    print("Press H to return to home (0,0) position")
    print("Press L to test left wheel 360° rotation")
    print("Press R to test right wheel 360° rotation")
    print("Note: Over SSH connections, held keys are properly detected")
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key_data = {}
        while True:
            key = get_key()
            current_time = time.time()
            if key is not None:
                direction = None
                if key.lower() == 'w':
                    direction = 'forward'
                elif key.lower() == 's':
                    direction = 'backward'
                elif key.lower() == 'a':
                    direction = 'left'
                elif key.lower() == 'd':
                    direction = 'right'
                if direction:
                    if direction not in key_data:
                        key_data[direction] = {
                            'pressed': True,
                            'last_press': current_time,
                            'last_seen': current_time,
                            'hold_reported': False
                        }
                        robot.move(direction)
                        print(f"Starting to move: {direction}")
                    else:
                        key_data[direction]['last_seen'] = current_time
                        if not key_data[direction]['pressed']:
                            key_data[direction]['pressed'] = True
                            key_data[direction]['last_press'] = current_time
                            key_data[direction]['hold_reported'] = False
                            robot.move(direction)
                            print(f"Starting to move: {direction}")
                elif key == ' ':
                    robot.stop(force_reset=True)
                    for k in key_data:
                        key_data[k]['pressed'] = False
                elif key.lower() == 'h':
                    robot.go_to_origin()
                elif key.lower() == 'l':
                    robot.test_single_wheel_rotation_left()
                elif key.lower() == 'r':
                    robot.test_single_wheel_rotation_right()
                elif key.lower() == 'q':
                    break
                elif key == '+':
                    robot.set_speed(faster=True)
                elif key == '-':
                    robot.set_speed(faster=False)
            for k in list(key_data.keys()):
                if key_data[k]['pressed'] and (current_time - key_data[k]['last_seen'] > RELEASE_THRESHOLD):
                    key_data[k]['pressed'] = False
                    print(f"Key {k} released")
                    if robot.current_direction == k:
                        robot.stop(force_reset=True)
            time.sleep(0.01)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        robot.shutdown()
        os.kill(os.getpid(), signal.SIGINT)
def main():
    print("Starting robot controller")
    print("Make sure you have root permission for MMIO access")
    robot = RobotController()
    try:
        robot.start()
        print("H - Return to home (0,0) position")
        input_thread = threading.Thread(target=key_listener, args=(robot,))
        input_thread.daemon = True
        input_thread.start()
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Shutting down...")
        robot.shutdown()
if __name__ == "__main__":
    main()