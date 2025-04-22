# Pallet Detection and Ground Segmentation
Pallet detection and Warehouse floor segmentation pipeline based on ROS2

This repository provides a complete pipeline for:
1. **Real‑time pallet detection** (bounding‑box)  
2. **Ground segmentation** (pixel‑wise mask)  
3. **ROS 2 Humble Python node** that subscribes to a camera topic, runs both YOLO v11 detection and segmentation (via TensorRT engines), and publishes overlayed images.

---

## Installation

1. **Clone the repo**  
   ```bash
   git clone git@github.com:atreyabhat/Warehouse_detect_segment.git
   cd Warehouse_detect_segment
   ```

2. **Install Python dependencies**  
   ```bash
   pip install -r requirements.txt
   ```

3. **Build the ROS 2 workspace**  
   ```bash
   cd detseg_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## How To Run

```bash
ros2 run det_seg_pkg det_seg_node --image_topic "<your_image_topic_here>"
```

- **Default image topic:**  
  ```
  /robot1/zed2i/left/image_rect_color
  ```
- **Published topics:**  
  - `/pallet_detection` (overlayed detection boxes)  
  - `/ground_segmentation` (overlayed segmentation mask)

---

## YOLO Detection Model Results

- **Model:** `yolo11m-seg` (medium) → TensorRT engine trained on ~15000 images with augmentation
- **mAP@0.5:** __%  
- **mAP@[0.5:0.95]:** __%  
- **Inference speed (Nvidia RTX 4070 Mobile (8GB), TensorRT FP16):** ~3.0ms


---

## YOLO Segmentation Model Results

- **Model:** `yolo11s-seg` (small) → TensorRT engine trained on ~1100 images with augmentation
- **Mean IoU:** __%  
- **Pixel Accuracy:** __%  
- **Inference speed (Nvidia RTX 4070 Mobile (8GB), TensorRT FP16):** ~2.0ms   



