from uuv_interfaces.srv import GenerateCone
from pose_generator.cone_function import generate_cone
import rclpy
from rclpy.node import Node


class ConeService(Node):

    def __init__(self):
        super().__init__('cone_service')
        self.srv = self.create_service(GenerateCone, 'generate_cone', self.cone_callback)

    def cone_callback(self, request, response):

        response.poses = generate_cone(
            num_rings=request.num_rings,
            clearance=request.clearance,
            cone_height=request.cone_height,
            delta_theta_deg=request.delta_theta_deg,
            mount_angle_deg=request.mount_angle_deg
        )

        self.get_logger().info(
            f'Generated {len(response.poses.poses)} poses '
            f'({request.num_rings} rings, z in [{request.clearance}, {request.cone_height}]m, '
            f'{request.delta_theta_deg}° step)'
        )

        return response


def main():
    rclpy.init()

    cone_service = ConeService()

    rclpy.spin(cone_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
