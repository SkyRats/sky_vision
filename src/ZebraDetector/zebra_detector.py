from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO("../best.pt")

# Export the model to NCNN format
model.export(format="ncnn")  # creates 'best_ncnn_model'

# Load the exported NCNN model
ncnn_model = YOLO("best_ncnn_model")

# Run inference
results = ncnn_model("zebra1.jpg")

