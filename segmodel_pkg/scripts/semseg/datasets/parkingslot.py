import torch 
import numpy as np
from torch import Tensor
from torch.utils.data import Dataset
from torchvision import io
from pathlib import Path
from typing import Tuple
import pandas as pd


class ParkingSlot(Dataset):
    
    PALETTE = torch.tensor([
        (255,255,255), (192,192,0), (113,193,46), (123,64,132), (77,128,255), (255,255,0), (34,134,136), (0,0,0)
    ])
    def __init__(self, root: str, split: str = 'train', transform = None) -> None:
        super().__init__()
        assert split in ['train', 'val', 'test']
        self.transform = transform
        self.n_classes = 8
        self.ignore_label = 255

        self.files = pd.read_csv(root + '/' + split + '.csv', header=None, names=["image","label"])
        self.images = self.files["image"].values[1:]
        self.labels = self.files["label"].values[1:]

        if  len(self.files) < 1:
            raise Exception(f"No images found in img_path")
        print(f"Found {len(self.files)} {split} images.")

    def __len__(self) -> int:
        return len(self.images)
    
    def __getitem__(self, index: int) -> Tuple[Tensor, Tensor]:
        img_path = self.images[index]
        lbl_path = self.labels[index].replace('labels', 'masks')

        image = io.read_image(img_path)
        label = io.read_image(lbl_path)
        
        if self.transform:
            image, label = self.transform(image, label)
        return image, label.squeeze().long()




