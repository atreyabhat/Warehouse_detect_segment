# Pallet Detection and Ground Segmentation
Pallet detection and Warehouse floor segmentation pipeline optimized for Jetson devices and based on ROS2

This repository provides a complete pipeline for:
1. **Real‑time pallet detection** (bounding‑box)  
2. **Ground segmentation** (pixel‑wise mask)  
3. **ROS 2 Humble Python node** that subscribes to a camera topic, runs both YOLO v11 detection and segmentation (via TensorRT engines), and publishes overlayed images.

---

## YOLO Detection Model Results

- **Base Model:** `yolo11m-seg` (medium)
- **Training:** Trained on ~15000 images with augmentation on A100 GPU for 100 epochs
- **mAP@0.5 (bbox):** 76.4%  
- **mAP@[0.5:0.95] (bbox):** 35.5%  (Could have been better)
- **Inference speed (Nvidia RTX 4070 Mobile (8GB), TensorRT FP16):** ~3.0ms


## YOLO Segmentation Model Results

- **Base Model:** `yolo11s-seg` (small) 
- **Training:** Trained on ~1100 images with augmentation on A100 GPU for 100epochs
- **mAP@0.5 (mask):** 69.1%  
- **mAP@[0.5:0.95] (mask):** 59.3%   
- **Inference speed (Nvidia RTX 4070 Mobile (8GB), TensorRT FP16):** ~2.0ms

Download the models from : https://drive.google.com/drive/folders/14uhlcHqhTAzMIwNPTl7g8yIwqOuzZ2Sp?usp=drive_link and place them in src/det_seg_pkg/models


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

## Sample Results

<img src="https://github.com/user-attachments/assets/9faf20f6-3c68-4adb-8858-12fa3695e2b8" width="410"/>
<img src="https://github.com/user-attachments/assets/c773f7ed-2e1b-4f4c-8921-0a99661d58f9" width="410"/>


![image](https://github.com/user-attachments/assets/247fad8e-51c8-410a-89e2-6cba8aab6825)







