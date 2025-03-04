import torch
import os

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageChops
#from einops import rearrange
from torch.utils.data import Dataset
from torchvision.transforms import ToTensor
import argparse


import sys
sys.path.append(".")

ATHOME_DATASET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "data", "athome")
LABELS_SUBSET_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "athome_labels_subset.txt")


class AtHomeImageDataset(Dataset):

    def __init__(self, dataset_dir = ATHOME_DATASET_DIR, is_train=True, transforms=None, label_path=LABELS_SUBSET_PATH, bbox_brightness_margin=0.05):
        self.dataset_dir = dataset_dir
        self.transforms = transforms or ToTensor()
        self.bbox_brightness_margin = bbox_brightness_margin

        with open(os.path.join(dataset_dir, "labels.txt"), "r") as f:
            base_labels = f.read().strip().split("\n")

        with open(label_path, "r") as f:
            self.labels = f.read().strip().split("\n")

        self.image_label_pairs = []

        with open(os.path.join(dataset_dir, "training_split.txt" if is_train else "validation_split.txt"), "r") as f:
            for line in f.read().strip().split("\n"):
                path, base_label_id = line.split("\t")
                image_path = path.replace("/PATH/TO/DATABASE/DIRECTORY/athome", ATHOME_DATASET_DIR)
                label_name = base_labels[int(base_label_id)]
                if label_name in self.labels:
                    label_index = self.labels.index(label_name)
                    self.image_label_pairs.append((image_path, label_index))

    @property
    def n_classes(self):
        return len(self.labels)

    def __getitem__(self, idx):
        image_path, label = self.image_label_pairs[idx]
        image = Image.open(image_path)
        image = self.transforms(image.convert('RGB'))

        return image, torch.tensor(label, dtype=torch.long)

    def __len__(self):
        return len(self.image_label_pairs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", "-i", type=int, default=100)

    args = parser.parse_args()

    dataset = AtHomeImageDataset()
    image, target = dataset[args.index]
    print(target)
    #plt.imshow(rearrange(image, "c h w -> h w c"))
    plt.imshow(np.moveaxis(image,0,-1))
    plt.show()
