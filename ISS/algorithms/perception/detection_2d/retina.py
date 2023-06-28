import torch
from ISS.algorithms.perception.detection_2d.base import detector_2d_base

def Retina(ckpt_file='resources/models/retinanet.pt'):
    model = torch.load(ckpt_file)
    return detector_2d_base(model)