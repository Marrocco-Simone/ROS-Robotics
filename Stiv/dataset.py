import os
import json
from PIL import Image
import pandas as pd
from pathlib import Path
from torchvision.io import read_image


class Dataset:
    def __init__(self):
        self.path = ""
        self.data = []
        self.classes = {}

    def load_legacy(self, path: str):
        self.path = path
        for s in os.listdir(path):
            for f in os.listdir(path + "/" + s):
                file_path = f"{path}/{s}/{f}"
                if f.split(".")[1] == "jpeg" and f.split(".")[0][-2] == "=":
                    boxes = []
                    classes = []
                    with open(file_path.replace("jpeg", "json"), "r") as j:
                        data = json.load(j)
                        for obj_name, obj in data.items():
                            min_x = min_y = 100_000
                            max_x = max_y = 0
                            for p in obj["bbox"]:
                                if p[0] < min_x:
                                    min_x = p[0]
                                if p[1] < min_y:
                                    min_y = p[1]

                                if p[0] > max_x:
                                    max_x = p[0]
                                if p[1] > max_y:
                                    max_y = p[1]
                            boxes.append((min_x, min_y, max_x, max_y))
                            classes.append(obj["y"])
                            if obj["y"] not in self.classes:
                                self.classes[obj["y"]] = len(self.classes)

                    self.data.append({
                        "image": f"{s}_{f}",
                        "boxes": boxes,
                        "classes": classes
                    })

    def load(self, path: str):
        for f in os.listdir(path):
            if f.split(".")[1] == "jpeg" and f.split(".")[0][-2] == "=":
                with open(path + "/" + f.replace(".jpeg", "_yolo.json"), "r") as j:
                    self.data.append(json.load(j))

    def save(self, path: str, format="yolo"):
        if format == "yolo":
            for datum in self.data:
                folder = f"{path}/labels"
                if not os.path.exists(folder):
                    os.makedirs(folder)
                with open(f"{folder}/{datum['image'].replace('jpeg', 'txt')}", "w+") as f:
                    for box, cla in zip(datum["boxes"], datum["classes"]):
                        cx = ((box[0] + box[2]) / 2) / 1024  # center
                        cy = ((box[1] + box[3]) / 2) / 1024
                        dx = (box[2] - box[0]) / 1024  # width
                        dy = (box[3] - box[1]) / 1024  # height

                        if cx > 1 or cx < 0 or cy > 1 or cy < 0 or dx > 1 or dx < 0 or dy > 1 or dy < 0:
                            continue
                        f.write(f"{self.classes[cla]} {cx} {cy} {dx} {dy}\n")

                folder = f"{path}/images"
                if not os.path.exists(folder):
                    os.makedirs(folder)
                with open(f"{folder}/{datum['image']}", "wb+") as f:
                    with open(f"{self.path}/{datum['image'].split('_')[0]}/{datum['image'].split('_')[1]}", "rb") as s:
                        f.write(s.read())

            with open(path + "/dataset.yml", "w+") as f:
                f.write(f"path: {Path(path).resolve()}\n")
                f.write("train: images/\n")
                f.write("val: images/\n")
                f.write(f"nc: {len(self.classes)}\n")
                f.write(f"names: {list(self.classes)}\n")

        elif format == "default":
            for datum in self.data:
                with open(datum["image"].replace(".jpeg", "_yolo.json"), "w+") as f:
                    f.write(json.dumps(datum, indent=4))

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        image = read_image(self.path + "/" + self.data[idx]["image"])
        label = self.data[idx]["classes"]
        if self.transform:
            image = self.transform(image)
        if self.target_transform:
            label = self.target_transform(label)
        return image, label


if __name__ == "__main__":
    dataset = Dataset()
    dataset.load_legacy("/home/stiven/Scrivania/RoboAlb/dataset/assign3")
    dataset.save("/home/stiven/Scrivania/RoboAlb/dataset_yolo/save3")
