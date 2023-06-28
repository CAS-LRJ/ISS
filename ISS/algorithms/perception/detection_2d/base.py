from torchvision import transforms
import torch

class detector_2d_base(object):

    def __init__(self, model):
        self.model = model        
    
    def detect(self, detection_2d_input):  
        with torch.no_grad():
            to_tensor = transforms.ToTensor()
            x = to_tensor(detection_2d_input.resize_image)
            x = x.unsqueeze(0).cuda()
            return self.model(x)    