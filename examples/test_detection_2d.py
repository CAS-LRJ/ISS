from ISS.algorithms.utils.dataexchange.detection_2d import Detection2DOutput, Detection2DInput
from ISS.algorithms.perception.detection_2d.retina import Retina
from ISS.algorithms.perception.detection_2d.yolov3 import YoloV3
from ISS.algorithms.perception.detection_2d.ssd import SSD
from ISS.algorithms.sensors.camera import Camera
import os
import time
from beamngpy import BeamNGpy, Scenario, Vehicle

USERPATH = fr'{os.getenv("LOCALAPPDATA")}/BeamNG.tech'   
beamng = BeamNGpy('localhost', 64256, user=USERPATH)
bng = beamng.open(launch=True, extensions=['util/roadData'])
scenario = Scenario('italy', 'Map Mod Test')
vehicle = Vehicle('ego_vehicle', model='vivace', license='AI', extensions=['mapPath'], partConfig='vehicles/vivace/tograc_qE_Unbreakable.pc')
scenario.add_vehicle(vehicle, pos=(-709, -1342, 141), rot_quat=(0, 0, -0.82, 0.55))
scenario.make(bng)
bng.settings.set_deterministic(60) # Set simulator to 60hz temporal resolution
bng.scenario.load(scenario)
bng.scenario.start()

model = Retina('resources/models/detection_2d/retinanet.pt')
# model = YoloV3('resources/models/detection_2d/yolov3.pt')
# model = SSD('resources/models/detection_2d/ssd.pt')
camera = Camera()
camera.from_beamng(bng, vehicle, resolution=(640, 480))

names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
         'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
         'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 
         'skis', 'snowboard','sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 
         'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 
         'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 
         'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
         'microwave', 'oven', 'toaster', 'sink', 'refrigerator,book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
det_in = Detection2DInput(resize=320, stride=320)
det_out = Detection2DOutput(names, [], 0.5, 'logs/2d_det_demo.mp4', True, True)


while True:
    
    det_in.from_camera(camera)    
    # det_in.resize_image.show()
    # print(model.detect(det_in))
    # start = time.time()
    model_out = model.detect(det_in)
    # print(time.time()-start)
    det_out.from_output(det_in, model_out)    