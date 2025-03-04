import torch
import torch.nn as nn
from torch.nn import Identity
from torchvision import models, transforms
from torchvision.transforms import RandomChoice, CenterCrop, RandomCrop, Resize, ToTensor, Compose, RandomRotation, \
    RandomAffine, ColorJitter


def load_resnet(n_classes):
    model_ft = models.resnet18(pretrained=True)
    for param in model_ft.parameters():
        param.requires_grad = False
    for param in model_ft.layer4.parameters():
        param.requires_grad = True
    num_ftrs = model_ft.fc.in_features
    model_ft.fc = nn.Linear(num_ftrs, n_classes)
    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                     std=[0.229, 0.224, 0.225])
    return nn.Sequential(normalize, model_ft)


def load_finetuned_resnet(finetuned_path, n_classes, eval=True):
    model = load_resnet(n_classes)
    model.load_state_dict(torch.load(finetuned_path, map_location=torch.device("cpu")))
    if eval:
        model.eval()
    return model


def get_transforms(input_size=224, rotation=15):
    train_tf_list = [
        Resize(int(input_size * 1.2)),
        RandomChoice([
            RandomRotation(rotation),
            RandomAffine(rotation,
                         scale=(0.9, 1.1),
                         translate=(0.1, 0.1),
                         shear=10),
            Identity()
        ]),
        RandomChoice([
            RandomCrop(input_size),
            CenterCrop(input_size)
        ]),
        ColorJitter(brightness=0.4, contrast=0.4,
                    saturation=0.4, hue=0.125),
        ToTensor()
    ]

    val_tf_list = [
        Resize((input_size, input_size)),
        ToTensor()
    ]

    return Compose(train_tf_list), Compose(val_tf_list)


def collate_fn(batch):
    return tuple(map(torch.stack, zip(*batch)))
