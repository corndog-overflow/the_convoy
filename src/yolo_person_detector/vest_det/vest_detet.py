import torch
from ultralytics import YOLO

# Load the existing model
model = YOLO('vest_det/new_weights.pt')

print(model.names)  

results = model(source=0, show = True, conf=.4, save=True)

