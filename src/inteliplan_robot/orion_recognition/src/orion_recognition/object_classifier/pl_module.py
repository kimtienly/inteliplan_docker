import os

import pytorch_lightning as pl
import torch
from torch.nn import functional as F
from torch.optim import Adam

from object_classifier.utils import load_resnet


class ResNetModule(pl.LightningModule):
    def __init__(self, num_classes, save_path=None, lr=1e-4):
        super().__init__()
        self.model = load_resnet(num_classes)
        self.save_path = save_path
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        self.save_hyperparameters()

    def _step_common(self, batch):
        images, labels = batch
        outputs = self.model(images)
        loss = F.cross_entropy(outputs, labels)
        acc = (outputs.argmax(dim=1) == labels).float().mean()
        return loss, acc

    def training_step(self, batch, batch_idx):
        loss, acc = self._step_common(batch)
        self.log("train_loss", loss, on_epoch=True)
        self.log("train_acc", acc, on_epoch=True)
        return {"loss": loss, "acc": acc}

    def validation_step(self, batch, batch_idx):
        loss, acc = self._step_common(batch)
        self.log("val_loss", loss, on_epoch=True)
        self.log("val_acc", acc, on_epoch=True)
        return {"loss": loss, "acc": acc}

    def predict_step(self, batch, batch_idx, dataloader_idx: int = 0):
        return self.infer_step(batch, batch_idx, "pred")

    def validation_epoch_end(self, outputs):
        acc = torch.stack([o['acc'] for o in outputs]).mean().item()
        print(acc)

    def on_train_epoch_end(self):
        torch.save(self.model.state_dict(), os.path.join(self.save_path, f"resnet_epoch_{self.current_epoch}.pth"))

    def configure_optimizers(self):
        return Adam(self.parameters(), lr=self.hparams.lr)
