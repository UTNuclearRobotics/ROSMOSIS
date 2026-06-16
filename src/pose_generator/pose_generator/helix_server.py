from uuv_interfaces.srv import GenerateHelix
from pose_generator.helix_function import generate_helix
import rclpy
from rclpy.node import Node


class HelixService(Node):

    def __init__(self):
        super().__init__('helix_service')
        self.srv = self.create_service(GenerateHelix, 'generate_helix', self.helix_callback)

    def helix_callback(self, request, response):
        
        response.poses = generate_helix(
            r_min=request.r_min,
            radius_multiplier=request.radius_multiplier,
            num_shells=request.num_shells,
            helix_height=request.helix_height,
            clearance=request.clearance,
            psi_max_deg=request.psi_max_deg,
            delta_theta_deg=request.delta_theta_deg,
            mount_angle_deg=request.mount_angle_deg
        )
        
        self.get_logger().info(
            f'Generated {len(response.poses.poses)} poses '
            f'(r_min={request.r_min}m, {request.num_shells} shells, '
            f'{request.delta_theta_deg}° step)'
        )

        
        return response


def main():
    rclpy.init()

    helix_service = HelixService()

    rclpy.spin(helix_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()