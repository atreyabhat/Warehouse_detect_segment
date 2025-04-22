from ultralytics import YOLO

def main():
    model = YOLO('yolo11s-seg.pt')  

    model.train(
        data='/data/ground_seg/peer_data/data.yaml',               
        model='yolo11s-seg.pt',            
        epochs=50,                      
        imgsz=640,                      
        batch=16,                       
        device=0,                       
        project='runs/train',           # project directory
        name='ground_seg_yolo_s',     # experiment name
        exist_ok=True                   # overwrite existing experiment
    )

if __name__ == '__main__':
    main()