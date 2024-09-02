from ultralytics import YOLO

model = YOLO("best.pt")

model.export(format="ncnn", imgsz=640)

ncnn_model = YOLO("best_ncnn_model")

results = ncnn_model(source="zebracima2.jpg", save=True)
# pode mexer no valor do conf para arrumar a precisão. conf=valor


