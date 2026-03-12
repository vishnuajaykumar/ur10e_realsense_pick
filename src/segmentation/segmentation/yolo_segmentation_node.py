#!/usr/bin/env python3
"""
YOLOv8-seg inference node.

Subscribes:
  /camera/color/image_raw        (sensor_msgs/Image)

Publishes:
  /segmentation/masks            (sensor_msgs/Image, mono8, one channel per frame,
                                  pixel value = instance id, 0 = background)
  /segmentation/detections       (vision_msgs/Detection2DArray)
  /segmentation/debug_image      (sensor_msgs/Image, RGB with mask overlay)

Parameters:
  model_path  (str)   : Path to YOLOv8-seg .pt or .onnx model file
  confidence  (float) : Detection confidence threshold (default 0.5)
  target_class (int)  : COCO class id to pick (-1 = all classes)
  device      (str)   : 'cpu' or 'cuda:0'
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
import numpy as np

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class YoloSegmentationNode(Node):
    def __init__(self):
        super().__init__("yolo_segmentation_node")

        # Parameters
        self.declare_parameter("model_path", "yolov8n-seg.pt")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("target_class", -1)   # -1 = all
        self.declare_parameter("device", "cpu")

        model_path = self.get_parameter("model_path").value
        self.conf = self.get_parameter("confidence").value
        self.target_class = self.get_parameter("target_class").value
        device = self.get_parameter("device").value

        # Load model
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error(
                "ultralytics not installed. Run: pip install ultralytics"
            )
            raise SystemExit(1)

        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(f"Loaded YOLOv8-seg model: {model_path} on {device}")

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )

        # Publishers
        self.masks_pub = self.create_publisher(Image, "/segmentation/masks", 10)
        self.detections_pub = self.create_publisher(
            Detection2DArray, "/segmentation/detections", 10
        )
        self.debug_pub = self.create_publisher(
            Image, "/segmentation/debug_image", 10
        )

    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = cv_image.shape[:2]

        # Run inference
        classes = None if self.target_class == -1 else [self.target_class]
        results = self.model(cv_image, conf=self.conf, classes=classes, verbose=False)

        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)

        # Build combined mask image (pixel value = instance index + 1, 0 = background)
        combined_mask = np.zeros((h, w), dtype=np.uint8)
        detections_msg = Detection2DArray(header=header)
        debug_image = cv_image.copy()

        result = results[0]
        if result.masks is not None:
            masks_data = result.masks.data.cpu().numpy()   # (N, H, W) float [0,1]
            boxes = result.boxes

            for idx, (mask, box) in enumerate(zip(masks_data, boxes)):
                instance_id = idx + 1  # 1-indexed, 0 = background

                # Resize mask to original image size
                mask_resized = cv2.resize(
                    mask, (w, h), interpolation=cv2.INTER_NEAREST
                )
                binary_mask = (mask_resized > 0.5).astype(np.uint8)
                combined_mask[binary_mask == 1] = instance_id

                # Detection2D message
                det = Detection2D()
                det.header = header
                xyxy = box.xyxy[0].cpu().numpy()
                cx = float((xyxy[0] + xyxy[2]) / 2.0)
                cy = float((xyxy[1] + xyxy[3]) / 2.0)
                det.bbox.center.x = cx
                det.bbox.center.y = cy
                det.bbox.size_x = float(xyxy[2] - xyxy[0])
                det.bbox.size_y = float(xyxy[3] - xyxy[1])

                hyp = ObjectHypothesisWithPose()
                hyp.id = str(int(box.cls[0].item()))
                hyp.score = float(box.conf[0].item())
                det.results.append(hyp)
                detections_msg.detections.append(det)

                # Debug overlay
                color = self._instance_color(instance_id)
                overlay = debug_image.copy()
                overlay[binary_mask == 1] = color
                debug_image = cv2.addWeighted(debug_image, 0.6, overlay, 0.4, 0)
                cv2.rectangle(debug_image,
                              (int(xyxy[0]), int(xyxy[1])),
                              (int(xyxy[2]), int(xyxy[3])),
                              color, 2)
                cv2.putText(debug_image, f"id:{instance_id} {hyp.score:.2f}",
                            (int(xyxy[0]), int(xyxy[1]) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Publish
        self.masks_pub.publish(
            self.bridge.cv2_to_imgmsg(combined_mask, encoding="mono8",
                                      header=header)
        )
        self.detections_pub.publish(detections_msg)
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8",
                                      header=header)
        )

    @staticmethod
    def _instance_color(instance_id: int):
        """Deterministic color per instance id (BGR)."""
        np.random.seed(instance_id * 37)
        return [int(c) for c in np.random.randint(80, 255, 3)]


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
