#!/usr/bin/env python3
"""Pygame-based keyboard teleoperation node for the ROV.

Publishes all command fields in one rov_msgs/msg/ROVControl message on
/rov/control/keyboard. Movement commands ramp by 10 PWM every 0.2 seconds.
"""

import time

import rclpy
from rclpy.node import Node
from simul_msgs.msg import ROVControl

try:
    import pygame
except ImportError as exc:
    raise RuntimeError(
        "Missing dependency 'pygame'. Install it with: sudo apt install python3-pygame"
    ) from exc

NEUTRAL_PWM = 1500
MIN_PWM = 1100
MAX_PWM = 1900
PWM_STEP = 10
RAMP_PERIOD_SEC = 0.2
PUBLISH_PERIOD_SEC = 0.02  # 50 Hz command heartbeat; control watchdog is 0.5 s.
COMMAND_TOPIC = "/rov/control/keyboard"

# Each mapping is (axis name, direction). Extra bindings Q/E and F/R are
# provided for roll/pitch because ROVControl contains those fields.
MOVEMENT_KEYS = {
    pygame.K_w: ("surge", +1),
    pygame.K_s: ("surge", -1),
    pygame.K_d: ("sway", +1),
    pygame.K_a: ("sway", -1),
    pygame.K_UP: ("heave", +1),
    pygame.K_DOWN: ("heave", -1),
    pygame.K_RIGHT: ("yaw", +1),
    pygame.K_LEFT: ("yaw", -1),
    pygame.K_e: ("roll", +1),
    pygame.K_q: ("roll", -1),
    pygame.K_r: ("pitch", +1),
    pygame.K_f: ("pitch", -1),
}
AXES = ("surge", "sway", "heave", "yaw", "roll", "pitch")


class ROVKeyboardNode(Node):
    def __init__(self):
        super().__init__("rov_keyboard_node")

        pygame.init()
        pygame.display.set_caption("ROV Keyboard Control - click this window to focus")
        self._screen = pygame.display.set_mode((660, 250))
        self._font = pygame.font.Font(None, 25)
        self._small_font = pygame.font.Font(None, 20)
        self._pygame_clock = pygame.time.Clock()
        self._window_open = True
        self._focused = True

        self.command_pub = self.create_publisher(ROVControl, COMMAND_TOPIC, 10)
        self._pressed_keys = set()
        self._pwm = {axis: NEUTRAL_PWM for axis in AXES}
        self._arm_pulse = False
        self._alt_hold_pulse = False
        self._last_ramp_time = time.monotonic()
        self._last_focus_warning = 0.0

        self._timer = self.create_timer(PUBLISH_PERIOD_SEC, self._timer_callback)
        self.get_logger().info(f"Publishing keyboard commands on {COMMAND_TOPIC}")
        self.get_logger().info(
            "W/S surge | A/D sway | Up/Down heave | Left/Right yaw | "
            "Q/E roll | F/R pitch | J arm toggle | K mode toggle"
        )
        self.get_logger().warn(
            "Click the Pygame window to focus it. Test initially with thrusters disabled."
        )
        self._draw_window()

    def _timer_callback(self):
        if not self._process_events():
            # Closing the keyboard UI terminates this node; the controller's
            # command watchdog will then engage its failsafe.
            self.get_logger().warn("Pygame window closed; stopping keyboard command publisher.")
            self._timer.cancel()
            if rclpy.ok():
                rclpy.shutdown()
            return

        now = time.monotonic()
        if now - self._last_ramp_time >= RAMP_PERIOD_SEC:
            # Keep the ramp cadence stable even if a callback was delayed.
            self._last_ramp_time = now
            for axis in AXES:
                target = self._target_for_axis(axis) if self._focused else NEUTRAL_PWM
                current = self._pwm[axis]
                if current < target:
                    current = min(current + PWM_STEP, target)
                elif current > target:
                    current = max(current - PWM_STEP, target)
                self._pwm[axis] = current

        # A focus loss clears held-key state immediately. PWM still ramps toward
        # neutral rather than jumping, as requested.
        msg = ROVControl()
        msg.surge = self._normalized(self._pwm["surge"])
        msg.sway = self._normalized(self._pwm["sway"])
        msg.heave = self._normalized(self._pwm["heave"])
        msg.roll = self._normalized(self._pwm["roll"])
        msg.pitch = self._normalized(self._pwm["pitch"])
        msg.yaw = self._normalized(self._pwm["yaw"])
        msg.arm = bool(self._arm_pulse)
        msg.alt_hold = bool(self._alt_hold_pulse)
        msg.grip = 0.0
        self._arm_pulse = False
        self._alt_hold_pulse = False
        self.command_pub.publish(msg)
        self._draw_window()

    def _process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._window_open = False
                self._pressed_keys.clear()
                return False

            if event.type == pygame.WINDOWFOCUSLOST:
                self._focused = False
                self._pressed_keys.clear()
            elif event.type == pygame.WINDOWFOCUSGAINED:
                self._focused = True
            elif event.type == pygame.KEYDOWN and self._focused:
                key = event.key
                # pygame key repeat is ignored if the key is already held.
                if key in self._pressed_keys:
                    continue
                self._pressed_keys.add(key)
                if key == pygame.K_j:
                    self._arm_pulse = True
                elif key == pygame.K_k:
                    self._alt_hold_pulse = True
            elif event.type == pygame.KEYUP:
                self._pressed_keys.discard(event.key)

        # Some window managers do not deliver focus events consistently.
        focused_now = pygame.key.get_focused()
        if not focused_now and self._focused:
            self._focused = False
            self._pressed_keys.clear()
        elif focused_now and not self._focused:
            self._focused = True
        if not self._focused:
            now = time.monotonic()
            if now - self._last_focus_warning > 5.0:
                self.get_logger().warn("Pygame window is not focused; movement keys are ignored.")
                self._last_focus_warning = now
        return self._window_open

    def _target_for_axis(self, axis):
        positive_down = any(
            mapped_axis == axis and direction > 0 and key in self._pressed_keys
            for key, (mapped_axis, direction) in MOVEMENT_KEYS.items()
        )
        negative_down = any(
            mapped_axis == axis and direction < 0 and key in self._pressed_keys
            for key, (mapped_axis, direction) in MOVEMENT_KEYS.items()
        )
        # Opposite-direction keys pressed together cancel out to neutral.
        if positive_down == negative_down:
            return NEUTRAL_PWM
        return MAX_PWM if positive_down else MIN_PWM

    @staticmethod
    def _normalized(pwm_value):
        return max(-1.0, min(1.0, (float(pwm_value) - NEUTRAL_PWM) / 400.0))

    def _draw_window(self):
        self._screen.fill((24, 31, 42))
        title = self._font.render("ROV Keyboard Control", True, (240, 240, 240))
        self._screen.blit(title, (20, 15))
        lines = [
            "W/S: surge     A/D: sway     UP/DOWN: heave",
            "LEFT/RIGHT: yaw     Q/E: roll     F/R: pitch",
            "J: arm/disarm toggle     K: ALT_HOLD/MANUAL toggle",
            "Close this window or stop this node to trigger controller failsafe.",
        ]
        y = 55
        for line in lines:
            text_surface = self._small_font.render(line, True, (210, 218, 230))
            self._screen.blit(text_surface, (20, y))
            y += 31
        focus_text = "FOCUSED" if self._focused else "NOT FOCUSED - movement keys ignored"
        focus_color = (90, 220, 120) if self._focused else (255, 180, 80)
        status = self._small_font.render(focus_text, True, focus_color)
        self._screen.blit(status, (20, 190))
        pygame.display.flip()

    def destroy_node(self):
        try:
            pygame.quit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ROVKeyboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
