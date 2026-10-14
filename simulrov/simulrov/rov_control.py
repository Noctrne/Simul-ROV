import time
import rclpy
from rclpy.node import Node

from simul_msgs.msg import ROVAttitude
from simul_msgs.msg import ROVDepth
from simul_msgs.msg import ROVControl

from pymavlink import mavutil

class ROVControlNode(Node):

    def __init__(self):

        super().__init__(

            'rov_control_node'

        )

        # ============================================================

        # MAVLink / Pixhawk 6C

        # ============================================================

        self.pix_address = '127.0.0.1:14551'

        self.pix_baud = 57600

        self.master = mavutil.mavlink_connection(

            self.pix_address,

            baud=self.pix_baud

        )

        self.get_logger().info(

            'Menunggu heartbeat Pixhawk 6C...'

        )

        heartbeat = self.master.wait_heartbeat()

        self.target_system = (

            heartbeat.get_srcSystem()

        )

        self.target_component = (

            heartbeat.get_srcComponent()

        )

        self.get_logger().info(

            f'Connected to Pixhawk 6C '

            f'(system={self.target_system}, '

            f'component={self.target_component})'

        )

        # ============================================================

        # PIXHAWK STATUS

        # ============================================================

        self.armed = False

        self.current_mode = 'UNKNOWN'

        self.update_pixhawk_status(

            heartbeat

        )

        # ============================================================

        # SENSOR DATA

        # ============================================================

        self.roll = 0.0

        self.pitch = 0.0

        self.yaw = 0.0

        self.depth = 0.0

        self.pressure = 0.0

        # ============================================================

        # KEYBOARD COMMAND DATA

        # ============================================================

        self.surge = 0.0

        self.sway = 0.0

        self.heave = 0.0

        self.yaw_cmd = 0.0
        self.roll_cmd = 0.0
        self.pitch_cmd = 0.0
        self.grip = 0.0

        self.arm = 0

        self.alt_hold = 0

        # Rising-edge detection

        self.arm_previous = 0

        self.alt_hold_previous = 0

        # ============================================================

        # CURRENT PWM

        # ============================================================

        self.rc1_pwm = 1500

        self.rc2_pwm = 1500

        self.rc3_pwm = 1500

        self.rc4_pwm = 1500

        self.rc5_pwm = 1500

        self.rc6_pwm = 1500

        self.rc7_pwm = 1500

        # ============================================================

        # KEYBOARD COMMAND FAILSAFE

        # ============================================================

        # Sender = 50 Hz

        # Timeout control = 0.5 s

        self.keyboard_timeout = 0.5

        self.last_keyboard_command_time = None

        self.keyboard_failsafe = True

        # Agar command DISARM/MANUAL tidak dikirim setiap 10 ms

        self.last_failsafe_command_time = 0.0

        self.failsafe_command_interval = 0.2

        # ============================================================

        # ROS SUBSCRIBERS

        # ============================================================

        self.att_sub = self.create_subscription(

            ROVAttitude,

            'rov/sensor/attitude',

            self.attitude_callback,

            10

        )

        self.depth_sub = self.create_subscription(

            ROVDepth,

            'rov/sensor/depth',

            self.depth_callback,

            10

        )

        # One custom ROVControl message carries every command axis and button.
        self.keyboard_sub = self.create_subscription(
            ROVControl,
            '/rov/control/keyboard',
            self.keyboard_command_callback,
            10
        )

        self.get_logger().info(

            'SUBSCRIBING: '

            '/rov/sensor/attitude, '

            '/rov/sensor/depth, '

            '/rov/control/keyboard'

        )

        # ============================================================

        # CONTROL LOOP - 100 Hz

        # ============================================================

        self.control_timer = self.create_timer(

            0.01,

            self.control_callback

        )

        # ============================================================

        # PIXHAWK STATUS - 10 Hz

        # ============================================================

        self.pixhawk_status_timer = self.create_timer(

            0.1,

            self.pixhawk_status_callback

        )

        # ============================================================

        # STATUS LOGGER - 1 Hz

        # ============================================================

        self.status_timer = self.create_timer(

            1.0,

            self.status_callback

        )

        self.get_logger().info(

            'ROV Control Node READY'

        )

        # ============================================================

        # STARTUP FAILSAFE

        # ============================================================

        self.get_logger().warn(

            'Keyboard command belum terdeteksi '

            '-> FAILSAFE aktif'

        )

        self.enforce_failsafe(

            force=True

        )

    # ================================================================

    # SENSOR CALLBACK

    # ================================================================

    def attitude_callback(

        self,

        msg: ROVAttitude

    ):

        self.roll = float(msg.roll)

        self.pitch = float(msg.pitch)

        self.yaw = float(msg.yaw)

    # ----------------------------------------------------------------

    def depth_callback(

        self,

        msg: ROVDepth

    ):

        self.depth = float(msg.depth)

        self.pressure = float(msg.pressure)

    # ================================================================

    # KEYBOARD COMMAND CALLBACK

    # ================================================================

    def keyboard_command_callback(

        self,

        msg: ROVControl

    ):

        # ------------------------------------------------------------

        # Movement

        # ------------------------------------------------------------

        self.surge = self.clamp(

            msg.surge,

            -1.0,

            1.0

        )

        self.sway = self.clamp(

            msg.sway,

            -1.0,

            1.0

        )

        self.heave = self.clamp(

            msg.heave,

            -1.0,

            1.0

        )

        self.yaw_cmd = self.clamp(

            msg.yaw,

            -1.0,

            1.0

        )

        self.roll_cmd = self.clamp(
            msg.roll,
            -1.0,
            1.0
        )

        self.pitch_cmd = self.clamp(
            msg.pitch,
            -1.0,
            1.0
        )

        # ------------------------------------------------------------

        # Button

        # ------------------------------------------------------------

        self.arm = int(msg.arm)

        self.alt_hold = int(

            msg.alt_hold

        )

        # ------------------------------------------------------------

        # Grip

        # ------------------------------------------------------------

        self.grip = self.clamp(

            msg.grip,

            -1.0,

            1.0

        )

        # ------------------------------------------------------------

        # Update keyboard command timestamp

        # ------------------------------------------------------------

        self.last_keyboard_command_time = (

            self.get_clock().now()

        )

        # ------------------------------------------------------------

        # KEYBOARD COMMAND KEMBALI

        #

        # Tidak otomatis ARM

        # Tidak otomatis ALT_HOLD

        # ------------------------------------------------------------

        if self.keyboard_failsafe:

            self.keyboard_failsafe = False

            self.get_logger().info(

                'Keyboard command terdeteksi kembali '

                '-> FAILSAFE dilepas'

            )

            # Reset previous button.

            # Tombol yang sedang ditekan saat reconnect

            # tidak dianggap sebagai command baru.

            self.arm_previous = self.arm

            self.alt_hold_previous = self.alt_hold

            return

        # ------------------------------------------------------------

        # ARM / DISARM TOGGLE

        # ------------------------------------------------------------

        if (

            self.arm == 1

            and self.arm_previous == 0

        ):

            if self.armed:

                self.arm_disarm(False)

            else:

                self.arm_disarm(True)

        self.arm_previous = self.arm

        # ------------------------------------------------------------

        # ALT_HOLD / MANUAL TOGGLE

        # ------------------------------------------------------------

        if (

            self.alt_hold == 1

            and self.alt_hold_previous == 0

        ):

            self.toggle_alt_hold()

        self.alt_hold_previous = (

            self.alt_hold

        )

    # ================================================================

    # PIXHAWK STATUS

    # ================================================================

    def pixhawk_status_callback(self):

        try:

            while True:

                heartbeat = self.master.recv_match(

                    type='HEARTBEAT',

                    blocking=False

                )

                if heartbeat is None:

                    break

                self.update_pixhawk_status(

                    heartbeat

                )

        except Exception as e:

            self.get_logger().error(

                f'Gagal membaca status Pixhawk: {e}'

            )

    # ----------------------------------------------------------------

    def update_pixhawk_status(

        self,

        heartbeat

    ):

        self.armed = bool(

            heartbeat.base_mode

            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        )

        mode = mavutil.mode_string_v10(

            heartbeat

        )

        if mode:

            self.current_mode = mode

        else:

            self.current_mode = 'UNKNOWN'

    # ================================================================

    # MAIN CONTROL LOOP - 100 Hz

    # ================================================================

    def control_callback(self):

        # ------------------------------------------------------------

        # Belum pernah menerima keyboard command

        # ------------------------------------------------------------

        if self.last_keyboard_command_time is None:

            if not self.keyboard_failsafe:

                self.keyboard_failsafe = True

            self.enforce_failsafe()

            return

        # ------------------------------------------------------------

        # Hitung umur keyboard command

        # ------------------------------------------------------------

        now = self.get_clock().now()

        elapsed = (

            now - self.last_keyboard_command_time

        ).nanoseconds / 1e9

        # ------------------------------------------------------------

        # TIMEOUT

        # ------------------------------------------------------------

        if elapsed > self.keyboard_timeout:

            if not self.keyboard_failsafe:

                self.keyboard_failsafe = True

                self.get_logger().error(

                    f'KEYBOARD COMMAND TIMEOUT '

                    f'({elapsed:.2f} s) '

                    f'-> FAILSAFE AKTIF'

                )

                # Kirim segera ketika pertama kali masuk failsafe

                self.enforce_failsafe(

                    force=True

                )

            else:

                self.enforce_failsafe()

            return

        # ------------------------------------------------------------

        # NORMAL CONTROL

        # ------------------------------------------------------------

        self.keyboard_failsafe = False

        # ------------------------------------------------------------

        # Command sumbu [-1, +1]

        # -> PWM [1100, 1900]

        # ------------------------------------------------------------

        # RC1 = Roll

        # RC2 = Pitch

        # RC3 = Heave

        # RC4 = Yaw

        # RC5 = Surge

        # RC6 = Sway

        # RC7 = Gripper

        self.rc1_pwm = self.normalized_to_pwm(self.roll_cmd)
        self.rc2_pwm = self.normalized_to_pwm(self.pitch_cmd)



        self.rc3_pwm = self.normalized_to_pwm(

            self.heave

        )

        self.rc4_pwm = self.normalized_to_pwm(

            self.yaw_cmd

        )

        self.rc5_pwm = self.normalized_to_pwm(

            self.surge

        )

        self.rc6_pwm = self.normalized_to_pwm(

            self.sway

        )

        # Gripper:

        # -1 -> 1000

        #  0 -> 1500

        # +1 -> 2000

        self.rc7_pwm = self.grip_to_pwm(

            self.grip

        )

        self.send_rc_override(

            self.rc1_pwm,

            self.rc2_pwm,

            self.rc3_pwm,

            self.rc4_pwm,

            self.rc5_pwm,

            self.rc6_pwm,

            self.rc7_pwm

        )

    # ================================================================

    # FAILSAFE

    # ================================================================

    def enforce_failsafe(

        self,

        force=False

    ):

        # ------------------------------------------------------------

        # 1. Semua PWM NEUTRAL

        # ------------------------------------------------------------

        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.yaw_cmd = 0.0
        self.roll_cmd = 0.0
        self.pitch_cmd = 0.0
        self.grip = 0.0
        self.send_neutral()

        # ------------------------------------------------------------

        # 2. DISARM + MANUAL

        # ------------------------------------------------------------

        now = time.monotonic()

        if (

            not force

            and (

                now

                - self.last_failsafe_command_time

            )

            < self.failsafe_command_interval

        ):

            return

        self.last_failsafe_command_time = now

        # ------------------------------------------------------------

        # DISARM

        # ------------------------------------------------------------

        self.arm_disarm(False)

        # ------------------------------------------------------------

        # MANUAL

        # ------------------------------------------------------------

        self.set_mode('MANUAL')

    # ================================================================

    # MAVLINK RC OVERRIDE

    # ================================================================

    def send_rc_override(

        self,

        rc1,

        rc2,

        rc3,

        rc4,

        rc5,

        rc6,

        rc7

    ):

        channels = [

            rc1,

            rc2,

            rc3,

            rc4,

            rc5,

            rc6,

            rc7,

            65535,

            65535,

            65535,

            65535,

            65535,

            65535,

            65535,

            65535,

            65535,

            65535,

            65535

        ]

        self.master.mav.rc_channels_override_send(

            self.target_system,

            self.target_component,

            *channels

        )

    # ================================================================

    # NEUTRAL

    # ================================================================

    def send_neutral(self):

        self.rc1_pwm = 1500

        self.rc2_pwm = 1500

        self.rc3_pwm = 1500

        self.rc4_pwm = 1500

        self.rc5_pwm = 1500

        self.rc6_pwm = 1500

        self.rc7_pwm = 1500

        self.send_rc_override(

            self.rc1_pwm,

            self.rc2_pwm,

            self.rc3_pwm,

            self.rc4_pwm,

            self.rc5_pwm,

            self.rc6_pwm,

            self.rc7_pwm

        )

    # ================================================================

    # ARM / DISARM

    # ================================================================

    def arm_disarm(

        self,

        arm: bool

    ):

        value = 1.0 if arm else 0.0

        self.master.mav.command_long_send(

            self.target_system,

            self.target_component,

            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

            0,

            value,

            0,

            0,

            0,

            0,

            0,

            0

        )

        if arm:

            self.get_logger().warn(

                'ARM command dikirim ke Pixhawk'

            )

        else:

            self.get_logger().warn(

                'DISARM command dikirim ke Pixhawk'

            )

    # ================================================================

    # ALT_HOLD / MANUAL

    # ================================================================

    def toggle_alt_hold(self):

        try:

            if self.current_mode == 'ALT_HOLD':

                self.set_mode('MANUAL')

            else:

                self.set_mode('ALT_HOLD')

        except Exception as e:

            self.get_logger().error(

                f'Gagal mengubah mode: {e}'

            )

    # ================================================================

    # SET MODE

    # ================================================================

    def set_mode(

        self,

        mode_name

    ):

        mode_mapping = (

            self.master.mode_mapping()

        )

        if mode_mapping is None:

            self.get_logger().error(

                'Mode mapping Pixhawk '

                'tidak tersedia'

            )

            return False

        if mode_name not in mode_mapping:

            self.get_logger().error(

                f'Mode {mode_name} '

                f'tidak tersedia pada Pixhawk'

            )

            return False

        mode_id = mode_mapping[

            mode_name

        ]

        self.master.mav.command_long_send(

            self.target_system,

            self.target_component,

            mavutil.mavlink.MAV_CMD_DO_SET_MODE,

            0,

            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,

            mode_id,

            0,

            0,

            0,

            0,

            0

        )

        self.get_logger().info(

            f'MAVLink SET_MODE -> '

            f'{mode_name}'

        )

        return True

    # ================================================================

    # STATUS LOGGER

    # ================================================================

    def status_callback(self):

        arm_status = (

            'ARMED'

            if self.armed

            else 'DISARMED'

        )

        failsafe_status = (

            'ACTIVE'

            if self.keyboard_failsafe

            else 'INACTIVE'

        )

        self.get_logger().info(

            f'[ROV STATUS] '

            f'MODE={self.current_mode} | '

            f'ARM={arm_status} | '

            f'FAILSAFE={failsafe_status} | '

            f'PWM: '

            f'R={self.rc1_pwm} '

            f'P={self.rc2_pwm} '

            f'H={self.rc3_pwm} '

            f'Y={self.rc4_pwm} '

            f'S={self.rc5_pwm} '

            f'W={self.rc6_pwm} '

            f'GRIP={self.rc7_pwm}'

        )

    # ================================================================

    # HELPER

    # ================================================================

    @staticmethod

    def clamp(

        value,

        minimum,

        maximum

    ):

        return max(

            minimum,

            min(

                maximum,

                float(value)

            )

        )

    # ----------------------------------------------------------------

    @staticmethod

    def normalized_to_pwm(

        value

    ):

        """

        -1.0 -> 1100

         0.0 -> 1500

        +1.0 -> 1900

        """

        value = max(

            -1.0,

            min(

                1.0,

                float(value)

            )

        )

        return int(

            round(

                1500

                + (400 * value)

            )

        )

    # ----------------------------------------------------------------

    @staticmethod

    def grip_to_pwm(

        value

    ):

        """

        -1.0 -> 1000

         0.0 -> 1500

        +1.0 -> 2000

        """

        value = max(

            -1.0,

            min(

                1.0,

                float(value)

            )

        )

        return int(

            round(

                1500

                + (500 * value)

            )

        )

    # ================================================================

    # SHUTDOWN

    # ================================================================

    def destroy_node(self):

        self.get_logger().warn(

            'ROV Control Node shutdown '

            '-> DISARM + MANUAL + NEUTRAL'

        )

        try:

            self.send_neutral()

            self.arm_disarm(False)

            self.set_mode('MANUAL')

        except Exception:

            pass

        super().destroy_node()

# ====================================================================

# MAIN

# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ROVControlNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':

    main()
