import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import CompressedImage
from PIL import Image
import numpy as np
from datetime import datetime
import cv2
import easyocr
import concurrent.futures
import time
from custom_interfaces.msg import AIres

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.get_logger().info('Image Capture Node has been started.')

        # Subscribe to compressed color images
        self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.easyocr_callback,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.SYSTEM_DEFAULT)
        )
        self.pub_ = self.create_publisher(AIres, '/AI_detect', 10)

        self.r = easyocr.Reader(['en'], gpu=True)
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)
        self.last_ocr_time = 0.0  # Track the last time we submitted an OCR task

    def depth_callback(self, msg):
        pass

    def crop_largest_purple_rectangle(self, pil_image):
        frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_purple = np.array([0, 0, 0])
        upper_purple = np.array([40, 40, 40])
        purple_mask = cv2.inRange(hsv_frame, lower_purple, upper_purple)

        contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return pil_image

        largest_contour = max(contours, key=cv2.contourArea)
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)
        x, y, w, h = cv2.boundingRect(approx)
        cropped_image = frame[y:y+h, x:x+w]
        return Image.fromarray(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))

    def process_image_find_digit(self, image, time):
        
        np_arr = np.frombuffer(image, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None:
            self.get_logger().error('Failed to decode the compressed image.')
            return

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(img_rgb)
        image_np = np.array(pil_image)
        hsv_image = cv2.cvtColor(image_np, cv2.COLOR_RGB2HSV)
        
        lower_purple = np.array([120, 50, 50])
        upper_purple = np.array([160, 255, 255])
        mask = cv2.inRange(hsv_image, lower_purple, upper_purple)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None
        
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cropped_image = image_np[y:y+h, x:x+w]
        result = self.r.readtext(cropped_image, detail=0, allowlist='0123456789')
        res = AIres()
        res.num = str(result)
        res.timestamp = time
        return res

    def easyocr_callback(self, msg: CompressedImage):
        # Check if at least 1 second has passed since last OCR submission
        current_time = time.time()
        if (current_time - self.last_ocr_time) < 1.0:
            # Less than a second since last OCR task, skip this frame
            return

        # Submit the OCR processing to the thread pool
        future = self.thread_pool.submit(self.process_image_find_digit, msg.data, current_time)
        future.add_done_callback(self.on_ocr_done)

        # Update the last_ocr_time since we just launched a new OCR job
        self.last_ocr_time = current_time

    def on_ocr_done(self, future):
        try:
            result = future.result()
            if result == None:
                return
            self.get_logger().info("OCR Result: %s" % str(result))
            self.pub_.publish(result)
        except Exception as e:
            self.get_logger().error(f"OCR processing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
