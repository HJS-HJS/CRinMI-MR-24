from ultralytics import YOLO

# Load a model
model = YOLO("yolov8m-seg.pt")  # load a pretrained model (recommended for training)

# Use the model
model.train(
    data="/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/scripts/segment_interface/yolov8/data.yaml",
    epochs=30,
    batch=2,
    imgsz=1280,
)  # train the model
