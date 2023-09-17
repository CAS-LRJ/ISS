from typing import Dict

import torch
import torch.nn as nn

class torchScriptWrapper(nn.Module):
    def __init__(self, path:str=None):
        super().__init__()
        self.path = path
        self.model = None

    def load_model(self):
        assert self.path is not None
        self.model = torch.jit.load(self.path)
        self.model.cuda()
        self.model.eval()
        
    def forward(self, data_dict:Dict[str, torch.Tensor]):
        assert self.model is not None
        return self.model(data_dict)
