"""
aruco_detector.py

ArUco marker tespit modülü — STUB.
TODO (Faz 2): OpenCV ArUco detector implementasyonu

Beklenen davranış:
  1. /camera/image_raw subscribe
  2. cv2.aruco ile marker tespit
  3. Tespit edilen markerları /perception/aruco'ya yayınla
  4. Task Executor tarafından kullanılır
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray


class ArucoDetector(Node):
    """ArUco marker tespit node'u — STUB."""

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('marker_ids', [0, 1, 2, 3, 4])
        self.declare_parameter('approach_dist_m', 1.0)

        # TODO (Faz 2): cv_bridge ve cv2.aruco import'u
        self._det_pub = self.create_publisher(
            PoseArray, '/perception/aruco', 10)
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10)

        self.get_logger().info('ArucoDetector başlatıldı. [STUB]')

    def _image_cb(self, msg: Image) -> None:
        # TODO (Faz 2): cv_bridge ile image dönüştür, ArUco tespit et
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
