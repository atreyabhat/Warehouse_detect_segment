python3 - <<EOF
from ultralytics import YOLO


model = YOLO('det_seg/runs/train/ground_seg_yolo_s/weights/best.pt')

model.export(
    format='engine',
    device = '0',
    half = True,
    dynamic = True,
    batch = 1)
    
EOF
