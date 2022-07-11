import rclpy

from src.calib_camera_lidar import CameraLidarCalibration


def main(args=None):
    rclpy.init(args=args)
    _main = CameraLidarCalibration()
    rclpy.spin(_main)

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
