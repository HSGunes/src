import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloV8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # RealSense renkli görüntü topic'i (güncellendi)
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/yolov8/detections/image', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # veya kendi modelin
        self.get_logger().info('YOLOv8 Node Başladı!')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)
        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0:  # COCO'da 0: person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # Draw red rectangle
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0,0,255), 2)
                    # Write 'person' label above the box
                    label = 'person'
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.7
                    thickness = 2
                    text_size, _ = cv2.getTextSize(label, font, font_scale, thickness)
                    text_x = x1
                    text_y = y1 - 10 if y1 - 10 > 10 else y1 + 20
                    cv2.putText(cv_image, label, (text_x, text_y), font, font_scale, (0,0,255), thickness, cv2.LINE_AA)
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 