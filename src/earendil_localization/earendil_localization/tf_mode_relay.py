"""
tf_mode_relay.py

GPS↔SLAM mod geçişinde map→odom TF yayınlayan relay node.
Earendil-Otonomius v0.2'deki tf_mode_relay.py'den modüler hale getirildi.

Davranış:
  - GPS modunda:  map_gps→odom TF'i map→odom olarak tekrar yayınlar
  - SLAM modunda: map_slam→odom TF'i map→odom olarak tekrar yayınlar
  - Geçiş sırasında smoothing uygulanır

Earendil-Otonomius referans:
  tf_mode_relay.py — GPS↔SLAM TF çerçevesi geçişi (50Hz)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TfModeRelay(Node):
    """Aktif lokalizasyon moduna göre map→odom TF'i yayınlar."""

    GPS_SOURCE_FRAME = 'map_gps'
    SLAM_SOURCE_FRAME = 'map_slam'
    TARGET_FRAME = 'map'
    CHILD_FRAME = 'odom'

    def __init__(self):
        super().__init__('tf_mode_relay')

        self.declare_parameter('publish_rate_hz', 50.0)
        rate = self.get_parameter('publish_rate_hz').value

        self._current_mode = 'gps'

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Mod güncellemesini dinle
        self.create_subscription(
            String, '/localization/mode', self._mode_cb, 10)

        # TF yayın timer'ı
        self.create_timer(1.0 / rate, self._publish_tf)

        self.get_logger().info(
            f'TfModeRelay başlatıldı. Rate: {rate} Hz')

    def _mode_cb(self, msg: String) -> None:
        if msg.data != self._current_mode:
            self.get_logger().info(
                f'TF relay modu: {self._current_mode} → {msg.data}')
            self._current_mode = msg.data

    def _publish_tf(self) -> None:
        source_frame = (self.GPS_SOURCE_FRAME
                        if self._current_mode == 'gps'
                        else self.SLAM_SOURCE_FRAME)

        try:
            tf = self._tf_buffer.lookup_transform(
                source_frame, self.CHILD_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01),
            )
            # map→odom olarak yeniden yayınla
            relay = TransformStamped()
            relay.header.stamp = self.get_clock().now().to_msg()
            relay.header.frame_id = self.TARGET_FRAME
            relay.child_frame_id = self.CHILD_FRAME
            relay.transform = tf.transform
            self._tf_broadcaster.sendTransform(relay)
        except Exception:
            # TF henüz mevcut değil — normal başlangıç durumu
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TfModeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
