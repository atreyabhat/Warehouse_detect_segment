#!/usr/bin/env python3

import os
import argparse
import time
import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class DetSeg(Node):
    def __init__(self, image_topic: str):
        super().__init__('det_seg_node')

        pkg_root   = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        models_dir = os.path.join(pkg_root, 'models')
        det_engine_path = os.path.join(models_dir, 'palletDet.engine')
        seg_engine_path = os.path.join(models_dir, 'groundSeg.engine')

        # load YOLO models
        self.det_model = YOLO(det_engine_path, task='detect')
        self.seg_model = YOLO(seg_engine_path, task='segment')

        self.bridge = CvBridge()

        det_pub_topic = '/pallet_detection'
        seg_pub_topic = '/ground_segmentation'

        #sub and pub
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile=qos)

        self.det_pub = self.create_publisher(Image, det_pub_topic, 10)
        self.seg_pub = self.create_publisher(Image, seg_pub_topic, 10)

        self.get_logger().info(f"Subscribed to {image_topic}")
        self.get_logger().info(f"Publishing detection → {det_pub_topic}")
        self.get_logger().info(f"Publishing segmentation → {seg_pub_topic}")

    def image_callback(self, msg: Image):
        #ros2 to cv2
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')    
        orig_h, orig_w = cv_img.shape[:2]
        
        #Detection
        det_img = cv_img.copy()
        det_results = self.det_model(cv_img)
        for r in det_results:
            boxes, classes, scores = (
                r.boxes.xyxy.cpu().numpy().astype(int),
                r.boxes.cls.cpu().numpy().astype(int),
                r.boxes.conf.cpu().numpy())
            
            for (x1, y1, x2, y2), cls_id, conf in zip(boxes, classes, scores):
                color = (0, 255, 0)
                cv2.rectangle(det_img, (x1, y1), (x2, y2), color, 2)
                label = f"{self.det_model.names[cls_id]} {conf:.2f}"
                cv2.putText(det_img, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        #Segmentation
        seg_img = cv_img.copy()
        alpha = 0.2
        seg_results = self.seg_model(cv_img)
        
        for r in seg_results:
            if r.masks is None:
                continue
                
            masks = r.masks.data.cpu().numpy()  # (N, H, W)
            
            for i, mask in enumerate(masks):
                mask_h, mask_w = mask.shape
                
                # Resize mask to match the original image
                if mask_h != orig_h or mask_w != orig_w:
                    mask = cv2.resize(mask.astype(np.float32), (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
                
                binary_mask = mask > 0.2
                
                overlay = seg_img.copy()
                color_mask = np.zeros_like(seg_img)
                color_mask[:, :] = (0, 255, 0)  # Green
                
                # Apply the mask to the color mask
                masked_color = np.zeros_like(seg_img)
                masked_color[binary_mask] = color_mask[binary_mask]
                
                # Blend the original image with the colored mask
                cv2.addWeighted(overlay, 1 - alpha, masked_color, alpha, 0, seg_img)

        det_msg = self.bridge.cv2_to_imgmsg(det_img, encoding='bgr8')
        seg_msg = self.bridge.cv2_to_imgmsg(seg_img, encoding='bgr8')
        
        det_msg.header = msg.header
        seg_msg.header = msg.header

        self.det_pub.publish(det_msg)
        self.seg_pub.publish(seg_msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image_topic',
        default='/robot1/zed2i/left/image_rect_color',
        help='ROS2 Image topic to subscribe to')
    
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = DetSeg(image_topic=args.image_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()