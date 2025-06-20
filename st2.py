from pynq import MMIO
import time
import warnings
import os
import subprocess
STEPPER_BASE_ADDR = 0x43C90000
STEPPER_ADDR_RANGE = 4096
class Stepper:
    REG_CONFIG = 0 * 4
    REG_STEPS = 1 * 4
    REG_CUR_STEPS = 1 * 4
    REG_PERIOD = 2 * 4
    REG_CUR_PERIOD = 2 * 4
    REG_DUTY = 3 * 4
    REG_CUR_DUTY = 3 * 4
    MIN_PULSE = 0x10
    MIN_PERIOD = 1000
    MAX_PERIOD = 65535
    MAX_STEPS_PER_CMD = (1 << 15) - 1
    def __init__(self, base_address=STEPPER_BASE_ADDR, address_range=STEPPER_ADDR_RANGE):
        try:
            subprocess.run(['./main'], check=True)
            pass
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to run ./main: {e}")
        except FileNotFoundError:
            raise RuntimeError("Could not find ./main executable in current directory")
        if not os.path.exists('/dev/mem'):
             raise RuntimeError("Cannot access /dev/mem. MMIO requires root privileges "
                                "and device tree configuration for /dev/mem.")
        try:
            self.mmio = MMIO(base_address, address_range)
            self._initialised = True
            print(f"Stepper initialised at address 0x{base_address:08X}")
        except Exception as e:
            raise RuntimeError(f"Failed to initialise MMIO at 0x{base_address:08X}: {e}")
        duty_val = self._pack_pwm(self.MIN_PULSE, self.MIN_PULSE)
        self.mmio.write(self.REG_DUTY, duty_val)
        default_period = self.MAX_PERIOD
        period_val = self._pack_pwm(default_period, default_period)
        self.mmio.write(self.REG_PERIOD, period_val)
        self.disable()
    def _check_init(self):
        if not hasattr(self, '_initialised') or not self._initialised:
            raise RuntimeError("Stepper driver has not been properly initialised.")
    def _pack_steps(self, left: int, right: int) -> int:
        if abs(left) > self.MAX_STEPS_PER_CMD or abs(right) > self.MAX_STEPS_PER_CMD:
            raise ValueError(f"Steps must be between -{self.MAX_STEPS_PER_CMD} and +{self.MAX_STEPS_PER_CMD}")
        dir_l = 1 if right >= 0 else 0
        step_l = abs(right)
        dir_r = 1 if left >= 0 else 0
        step_r = abs(left)
        val = (dir_r << 31) | (step_r << 16) | (dir_l << 15) | step_l
        return val
    def _unpack_steps(self, val: int) -> tuple[int, int, int, int]:
        step_l = val & 0x7FFF
        dir_l = (val >> 15) & 0x1
        step_r = (val >> 16) & 0x7FFF
        dir_r = (val >> 31) & 0x1
        return step_r, dir_r, step_l, dir_l
    def _pack_pwm(self, left: int, right: int) -> int:
        if not (0 <= left <= 0xFFFF and 0 <= right <= 0xFFFF):
             raise ValueError("PWM values (period/duty) must be 16-bit unsigned.")
        val = (left << 16) | right
        return val
    def _unpack_pwm(self, val: int) -> tuple[int, int]:
        left = val & 0xFFFF
        right = (val >> 16) & 0xFFFF
        return right, left
    def enable(self):
        self._check_init()
        self.mmio.write(self.REG_CONFIG, 0x1)
        print("Stepper enabled.")
    def disable(self):
        self._check_init()
        self.mmio.write(self.REG_CONFIG, 0x0)
        print("Stepper disabled.")
    def reset(self):
        self._check_init()
        self.mmio.write(self.REG_CONFIG, 0x2)
        time.sleep(0.01)
        self.mmio.write(self.REG_CONFIG, 0x0)
        print("Stepper reset.")
    def steps_done(self) -> bool:
        self._check_init()
        current_steps_val = self.mmio.read(self.REG_CUR_STEPS)
        step_l, _, step_r, _ = self._unpack_steps(current_steps_val)
        return step_l == 0 and step_r == 0
    def wait_for_steps(self, timeout: float = 10.0):
        self._check_init()
        while not self.steps_done():
            time.sleep(0.001)
    def set_steps(self, left: int, right: int):
        self._check_init()
        packed_value = self._pack_steps(left, right)
        self.mmio.write(self.REG_STEPS, packed_value)
    def set_speed(self, left_period: int, right_period: int):
        self._check_init()
        left_period = min(max(left_period, self.MIN_PERIOD), self.MAX_PERIOD)
        right_period = min(max(right_period, self.MIN_PERIOD), self.MAX_PERIOD)
        if left_period < self.MIN_PERIOD * 2 or right_period < self.MIN_PERIOD * 2:
            warnings.warn(f"Stepper period is very low. This will result in very fast movement.", UserWarning)
        packed_value = self._pack_pwm(left_period, right_period)
        self.mmio.write(self.REG_PERIOD, packed_value)
    def get_remaining_steps(self) -> tuple[int, int]:
        self._check_init()
        current_steps_val = self.mmio.read(self.REG_CUR_STEPS)
        step_l, dir_l, step_r, dir_r = self._unpack_steps(current_steps_val)
        remaining_left = step_l if dir_l == 1 else -step_l
        remaining_right = step_r if dir_r == 1 else -step_r
        return remaining_left, remaining_right
    def get_current_period(self) -> tuple[int, int]:
        self._check_init()
        period_val = self.mmio.read(self.REG_CUR_PERIOD)
        return self._unpack_pwm(period_val)
    def get_current_duty(self) -> tuple[int, int]:
        self._check_init()
        duty_val = self.mmio.read(self.REG_CUR_DUTY)
        return self._unpack_pwm(duty_val)
    def destroy(self):
        if hasattr(self, '_initialised') and self._initialised:
            try:
                self.disable()
            except Exception as e:
                print(f"Warning: Error during disable on destroy: {e}")
            finally:
                self._initialised = False
                print("Stepper driver resources released (disabled motors).")
        self._initialised = False
    def __del__(self):
        try:
            self.destroy()
        except:
            pass
if __name__ == "__main__":
    stepper = None
    try:
        print("Creating Stepper object using direct MMIO...")
        stepper = Stepper(base_address=STEPPER_BASE_ADDR, address_range=STEPPER_ADDR_RANGE)
        print("\nStarting stepper test sequence...")
        speed_period = Stepper.MIN_PERIOD
        print(f"Setting speed period to: {speed_period}")
        stepper.set_speed(speed_period, speed_period)
        l_p, r_p = stepper.get_current_period()
        print(f"Read back period - Left: {l_p}, Right: {r_p}")
        stepper.enable()
        time.sleep(0.5)
        steps_to_move = 2000
        print(f"\nMoving forward {steps_to_move} steps...")
        stepper.set_steps(steps_to_move, steps_to_move)
        stepper.wait_for_steps(timeout=5.0)
        print("Forward move complete.")
        rem_l, rem_r = stepper.get_remaining_steps()
        print(f"Remaining steps: Left={rem_l}, Right={rem_r}")
        time.sleep(0.5)
        print(f"\nMoving backward {steps_to_move} steps...")
        stepper.set_steps(-steps_to_move, -steps_to_move)
        stepper.wait_for_steps(timeout=5.0)
        print("Backward move complete.")
        rem_l, rem_r = stepper.get_remaining_steps()
        print(f"Remaining steps: Left={rem_l}, Right={rem_r}")
        time.sleep(0.5)
        turn_steps = 1000
        print(f"\nTurning right {turn_steps} steps...")
        stepper.set_steps(turn_steps, -turn_steps)
        stepper.wait_for_steps(timeout=3.0)
        print("Turn complete.")
        rem_l, rem_r = stepper.get_remaining_steps()
        print(f"Remaining steps: Left={rem_l}, Right={rem_r}")
        time.sleep(0.5)
        print(f"\nTurning left {turn_steps} steps...")
        stepper.set_steps(-turn_steps, turn_steps)
        stepper.wait_for_steps(timeout=3.0)
        print("Turn complete.")
        rem_l, rem_r = stepper.get_remaining_steps()
        print(f"Remaining steps: Left={rem_l}, Right={rem_r}")
        time.sleep(0.5)
        print("\nTest sequence finished.")
    except RuntimeError as e:
        print(f"Runtime Error: {e}")
    except ValueError as e:
        print(f"Value Error: {e}")
    except TimeoutError as e:
        print(f"Timeout Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if stepper is not None and isinstance(stepper, Stepper):
            print("\nCleaning up...")
            stepper.destroy()