##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
##################################################################################################
import numpy as np
import random

# Define the classes and their estimated sizes
object_classes = {
    "person": (1.7, 0.5, 0.5),
    "bicycle": (1.5, 0.6, 1.8),
    "car": (1.5, 1.8, 4.0),
    "motorbike": (1.5, 0.8, 2.0),
    "aeroplane": (10.0, 20.0, 30.0),
    "bus": (3.0, 2.5, 10.0),
    "train": (3.0, 3.0, 30.0),
    "truck": (3.0, 2.5, 12.0),
    "boat": (2.0, 3.0, 10.0),
    "traffic light": (2.0, 0.5, 0.5),
    "fire hydrant": (0.75, 0.5, 0.5),
    "stop sign": (1.0, 0.5, 0.5),
    "parking meter": (1.5, 0.5, 0.5),
    "bench": (0.5, 1.5, 0.5),
    "bird": (0.3, 0.3, 0.3),
    "cat": (0.3, 0.3, 0.6),
    "dog": (0.5, 0.5, 1.0),
    "horse": (1.6, 0.5, 2.0),
    "sheep": (0.9, 0.5, 1.0),
    "cow": (1.5, 1.0, 2.5),
    "elephant": (3.0, 3.0, 6.0),
    "bear": (1.5, 1.0, 2.5),
    "zebra": (1.5, 0.5, 2.0),
    "giraffe": (5.0, 1.5, 3.0),
    "backpack": (0.5, 0.3, 0.2),
    "umbrella": (1.0, 0.5, 0.5),
    "handbag": (0.5, 0.3, 0.2),
    "tie": (0.7, 0.1, 0.05),
    "suitcase": (0.7, 0.3, 0.5),
    "frisbee": (0.3, 0.3, 0.05),
    "skis": (2.0, 0.1, 0.1),
    "snowboard": (1.5, 0.3, 0.1),
    "sports ball": (0.3, 0.3, 0.3),
    "kite": (1.0, 1.0, 0.1),
    "baseball bat": (1.0, 0.1, 0.1),
    "baseball glove": (0.3, 0.3, 0.1),
    "skateboard": (0.5, 0.2, 0.8),
    "surfboard": (2.0, 0.5, 0.1),
    "tennis racket": (0.7, 0.3, 0.1),
    "bottle": (0.3, 0.1, 0.1),
    "wine glass": (0.3, 0.1, 0.1),
    "cup": (0.2, 0.2, 0.2),
    "fork": (0.2, 0.02, 0.02),
    "knife": (0.2, 0.02, 0.02),
    "spoon": (0.2, 0.02, 0.02),
    "bowl": (0.3, 0.3, 0.1),
    "banana": (0.3, 0.1, 0.1),
    "apple": (0.3, 0.3, 0.3),
    "sandwich": (0.3, 0.3, 0.1),
    "orange": (0.3, 0.3, 0.3),
    "broccoli": (0.5, 0.3, 0.3),
    "carrot": (0.3, 0.1, 0.1),
    "hot dog": (0.3, 0.1, 0.1),
    "pizza": (0.5, 0.5, 0.1),
    "donut": (0.3, 0.3, 0.1),
    "cake": (0.5, 0.5, 0.3),
    "chair": (1.0, 0.5, 0.5),
    "sofa": (1.0, 2.0, 0.8),
    "pottedplant": (0.5, 0.5, 0.5),
    "bed": (1.0, 2.0, 0.8),
    "diningtable": (1.0, 2.0, 0.8),
    "toilet": (1.0, 0.5, 0.5),
    "tvmonitor": (0.5, 1.0, 0.1),
    "laptop": (0.5, 0.5, 0.1),
    "mouse": (0.1, 0.1, 0.05),
    "remote": (0.2, 0.1, 0.05),
    "keyboard": (0.5, 0.2, 0.05),
    "cell phone": (0.2, 0.1, 0.05),
    "microwave": (0.5, 0.5, 0.5),
    "oven": (1.0, 0.5, 0.5),
    "toaster": (0.3, 0.2, 0.2),
    "sink": (0.5, 1.0, 0.5),
    "refrigerator": (2.0, 1.0, 1.0),
    "book": (0.3, 0.2, 0.1),
    "clock": (0.3, 0.3, 0.1),
    "vase": (0.5, 0.3, 0.3),
    "scissors": (0.2, 0.05, 0.05),
    "teddy bear": (0.5, 0.3, 0.3),
    "hair drier": (0.3, 0.2, 0.1),
    "toothbrush": (0.2, 0.02, 0.02),
}

# Generate a synthetic label file
def generate_label_file(label_filename):
    labels = []

    for obj_class, (h, w, l) in object_classes.items():
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = random.uniform(0, 5)
        ry = random.uniform(-np.pi, np.pi)
        label = f"{obj_class} 0.0 0 0.0 0 0 0 0 {h} {w} {l} {x} {y} {z} {ry}"
        labels.append(label)

    with open(label_filename, 'w') as f:
        for label in labels:
            f.write(label + '\n')

# Specify the label file path
label_file_path = "labels.txt"

# Generate the label file
generate_label_file(label_file_path)

print(f"Label file generated at {label_file_path}")

