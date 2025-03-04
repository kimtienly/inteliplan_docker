import argparse
import datetime
import os
import pytorch_lightning as pl

from pytorch_lightning.loggers import TensorBoardLogger
from torch.utils.data import DataLoader
from pathlib import Path

from object_classifier.datasets.athome_dataset import AtHomeImageDataset
from object_classifier.utils import get_transforms, collate_fn
from object_classifier.pl_module import ResNetModule

import sys

sys.path.append(".")
sys.path.append("./src")

dir_path = Path(__file__).resolve().parent

parser = argparse.ArgumentParser()
parser.add_argument("--name", type=str, default="")
parser.add_argument("--batch_size", type=int, default=128)
parser.add_argument("--n_workers", type=int, default=4)
parser.add_argument("--lr", type=float, default=1e-4)
parser.add_argument("--save_path", default=os.path.join(dir_path,"model"))
parser.add_argument("--checkpoints", default=os.path.join(dir_path, "checkpoints"))
parser.add_argument("--input_size", '-sz', type=int, default=224)

args = parser.parse_args()
train_tf, val_tf = get_transforms(args.input_size)

train_dataset = AtHomeImageDataset(is_train=True, transforms=train_tf)
val_dataset = AtHomeImageDataset(is_train=False, transforms=val_tf)

train_loader = DataLoader(train_dataset, shuffle=True, batch_size=args.batch_size, drop_last=True, num_workers=args.n_workers, collate_fn=collate_fn)
val_loader = DataLoader(val_dataset, shuffle=False, batch_size=args.batch_size, drop_last=False, num_workers=args.n_workers, collate_fn=collate_fn)

version_name = f"{datetime.datetime.now().strftime('%m%d_%H%M%S')}{'_' + args.name if args.name else ''}"

module = ResNetModule(train_dataset.n_classes, save_path=os.path.join(args.save_path, version_name), lr=args.lr)

tb_logger = TensorBoardLogger(save_dir=".", version=version_name)
trainer = pl.Trainer(accelerator="gpu", logger=tb_logger, gpus=1)
trainer.fit(module, train_loader, val_loader, ckpt_path=args.checkpoint)
