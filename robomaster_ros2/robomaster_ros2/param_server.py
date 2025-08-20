import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class param_server(Node):

    def __init__(self):
        super().__init__('param_server',start_parameter_services=True)

        self.get_logger().info('param_server node started')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_of_drones', Parameter.Type.INTEGER),
                ('num_of_eps1', Parameter.Type.INTEGER),
            ])
        num_of_drones = self.get_parameter('num_of_drones').value
        num_of_eps1 = self.get_parameter('num_of_eps1').value

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rmtt.mled_display_num', Parameter.Type.BOOL),
                ('rmtt.external_position', Parameter.Type.BOOL),
                ('rmtt.drone_name_list', Parameter.Type.STRING_ARRAY),
                ('rmtt.pub_front_tof', Parameter.Type.BOOL),
                ('rmtt.pub_bottom_tof', Parameter.Type.BOOL),
                ('rmtt.pub_attitude', Parameter.Type.BOOL),
                ('rmtt.pub_barometer', Parameter.Type.BOOL),
                ('rmtt.pub_imu', Parameter.Type.BOOL),
                ('rmtt.pub_mpad', Parameter.Type.BOOL),
            ])
        drone_name_list = self.get_parameter('rmtt.drone_name_list').value
        
        for i in range(num_of_drones):
            self.declare_parameters(
                namespace='',
                parameters=[
                    (f'rmtt.{drone_name_list[i]}.ip', Parameter.Type.STRING),
                    (f'rmtt.{drone_name_list[i]}.pub_cam', Parameter.Type.BOOL),
                    (f'rmtt.{drone_name_list[i]}.yaw_hold', Parameter.Type.BOOL),
                    (f'rmtt.{drone_name_list[i]}.cam_direction', Parameter.Type.INTEGER),
                    (f'rmtt.{drone_name_list[i]}.ext_pose_topic_type', Parameter.Type.STRING),
                    (f'rmtt.{drone_name_list[i]}.ext_pose_topic_name', Parameter.Type.STRING),
                ])
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rmep.led_num', Parameter.Type.BOOL),
                ('rmep.external_position', Parameter.Type.BOOL),
                ('rmep.robot_name_list', Parameter.Type.STRING_ARRAY),
                ('rmep.pub_imu', Parameter.Type.BOOL),
                ('rmep.pub_cam', Parameter.Type.BOOL),
                ('rmep.pub_marker', Parameter.Type.BOOL),
                ('rmep.pub_armpose', Parameter.Type.BOOL),
            ])
        robot_name_list = self.get_parameter('rmep.robot_name_list').value
        for i in range(num_of_eps1):
            self.declare_parameters(
                namespace='',
                parameters=[
                    (f'rmep.{robot_name_list[i]}.ip', Parameter.Type.STRING),
                    (f'rmep.{robot_name_list[i]}.pub_cam', Parameter.Type.BOOL),
                    (f'rmep.{robot_name_list[i]}.ext_pose_topic_type', Parameter.Type.STRING),
                    (f'rmep.{robot_name_list[i]}.ext_pose_topic_name', Parameter.Type.STRING),
                ])


def main(args=None):
    rclpy.init(args=args)

    node = param_server()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
