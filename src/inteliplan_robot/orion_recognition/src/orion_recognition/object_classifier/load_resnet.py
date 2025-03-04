import argparse
import os

from pathlib import Path
from utils import load_finetuned_resnet

dir_path = os.path.join(Path(__file__).resolve().parent, "checkpoints")

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", help="path to checkpoint", default=os.path.join(dir_path, "resnet_epoch_16.pth"))
parser.add_argument("--n_classes", help="number of classes", type=int, default=180)

args = parser.parse_args()

model = load_finetuned_resnet(args.checkpoint, args.n_classes, eval=True)

print(model)

# make sure that the image input is a Torch.Tensor and
# output = model(image.unsqueeze(0))
