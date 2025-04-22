from ultralytics import YOLO
import os

def main():
    model = YOLO('yolo11m.pt')  

    model.train(
        data='/data/pallet_detection/annotated/Peer_Pallet/data.yaml',               
        model='yolo11m.pt',            
        epochs=30,                      
        imgsz=640,                      
        batch=64,                       
        device=0,                       
        project='runs/train',           # project directory
        name='pallet_yolov11_m_tb',     # experiment name
        exist_ok=True                   # overwrite existing experiment
    )

if __name__ == '__main__':
    main()