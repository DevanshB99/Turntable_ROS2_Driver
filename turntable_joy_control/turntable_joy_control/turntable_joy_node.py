"""Turntable jogging control node using turntable_forward_position_controller - Xbox controller input."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController

TWO_PI = 2.0 * math.pi


class TurntableJoyNode(Node):
    def __init__(self):
        super().__init__('turntable_joy_node')

        # Declare parameters
        self.declare_parameter('jog_speed_deg_per_sec', 30.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cw_button', 5)   # RB
        self.declare_parameter('ccw_button', 4)  # LB
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_topic',
                               '/turntable_forward_position_controller/commands')
        self.declare_parameter('switch_controller_service',
                               '/controller_manager/switch_controller')
        self.declare_parameter('position_controller',
                               'turntable_forward_position_controller')
        self.declare_parameter('trajectory_controller',
                               'turntable_trajectory_controller')
        self.declare_parameter('joint_name', 'disc_joint')

        # Read parameters
        self.jog_speed_rad = math.radians(
            self.get_parameter('jog_speed_deg_per_sec').value
        )
        self.publish_rate = self.get_parameter('publish_rate').value
        self.cw_button = self.get_parameter('cw_button').value
        self.ccw_button = self.get_parameter('ccw_button').value
        joy_topic = self.get_parameter('joy_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        command_topic = self.get_parameter('command_topic').value
        switch_controller_service = self.get_parameter('switch_controller_service').value
        self.position_controller = self.get_parameter('position_controller').value
        self.trajectory_controller = self.get_parameter('trajectory_controller').value
        self.joint_name = self.get_parameter('joint_name').value

        self.dt = 1.0 / self.publish_rate

        # State
        self.target_position = None
        self.position_seeded = False
        self.cw_pressed = False
        self.ccw_pressed = False

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            command_topic,
            qos_profile
        )

        # Subscriptions
        self.joint_states_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_states_cb, qos_profile
        )
        self.create_subscription(
            Joy, joy_topic, self.joy_cb, qos_profile
        )

        # Switch controllers on startup
        self.switch_client = self.create_client(
            SwitchController, switch_controller_service
        )
        self.switch_controllers(
            activate=[self.position_controller],
            deactivate=[self.trajectory_controller]
        )

        # Register shutdown callback
        self.context.on_shutdown(self.shutdown)

        # Main timer
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f'Turntable joy node started '
            f'(speed={math.degrees(self.jog_speed_rad):.1f} deg/s)'
        )

    def switch_controllers(self, activate, deactivate):
        """Call switch_controller service asynchronously."""
        if not self.switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'switch_controller service not available, continuing anyway'
            )
            return
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self.switch_client.call_async(req)
        future.add_done_callback(self._switch_cb)

    def _switch_cb(self, future):
        try:
            result = future.result()
            if result.ok:
                self.get_logger().info('Controller switch successful')
            else:
                self.get_logger().warn('Controller switch returned not ok')
        except Exception as e:
            self.get_logger().error(f'Controller switch failed: {e}')

    def joint_states_cb(self, msg: JointState):
        if self.position_seeded:
            return
        try:
            idx = msg.name.index(self.joint_name)
            self.target_position = msg.position[idx]
            self.position_seeded = True
            self.get_logger().info(
                f'Seeded target position: {math.degrees(self.target_position):.2f} deg'
            )
            self.destroy_subscription(self.joint_states_sub)
        except ValueError:
            pass

    def joy_cb(self, msg: Joy):
        if len(msg.buttons) <= max(self.cw_button, self.ccw_button):
            self.get_logger().warn(
                f'Joy message has only {len(msg.buttons)} buttons, '
                f'need at least {max(self.cw_button, self.ccw_button) + 1}'
            )
            return
        self.cw_pressed = bool(msg.buttons[self.cw_button])
        self.ccw_pressed = bool(msg.buttons[self.ccw_button])

    def timer_cb(self):
        if not self.position_seeded:
            return

        # Update target
        if self.cw_pressed:
            self.target_position += self.jog_speed_rad * self.dt
        elif self.ccw_pressed:
            self.target_position -= self.jog_speed_rad * self.dt

        # Wrap to [0, 2π)
        self.target_position = self.target_position % TWO_PI

        # Publish command
        msg = Float64MultiArray()
        msg.data = [self.target_position]
        self.cmd_pub.publish(msg)

    def shutdown(self):
        """Restore controllers."""
        self.switch_controllers(
            activate=[self.trajectory_controller],
            deactivate=[self.position_controller]
        )
        self.get_logger().info('Shutting down, restoring controllers')


def main(args=None):
    rclpy.init(args=args)
    node = TurntableJoyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
