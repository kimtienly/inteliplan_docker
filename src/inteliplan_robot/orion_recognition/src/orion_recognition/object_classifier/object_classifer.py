#!/usr/bin/env python
# coding: utf-8

import os
import torch
from object_classifier.utils import load_finetuned_resnet
from pathlib import Path

dir_path = os.path.join(Path(__file__).resolve().parent, "checkpoints")

class ObjectClassifer(torch.nn.Module):
    def __init__(self, device: torch.device):
        super(ObjectClassifer, self).__init__()
        self.device = device
        model_path = os.path.join(dir_path, "resnet_epoch_999.pth")
        self.model = load_finetuned_resnet(model_path, 100, eval=True)
        self.model.to(self.device).float()

    def forward(self, img):
        assert len(img.size()) == 3, "Assumes a single image"
        # img = transforms.functional.resize(img, size=[224])
        # print("Image shape: {}".format(img.shape))
        output = self.model(img.unsqueeze(0))
        probabilities = torch.softmax(output[0], dim=0)
        return torch.argmax(probabilities).item(), torch.max(probabilities).item()
