import numpy as np
import open3d
cimport numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

from ISS.algorithms.sensors.carla_lidar import CarlaLiDAR
from ISS.algorithms.utils.visutils.open3d_vis_utils import open3d_vis_utils

class Detection3DInput(object):
    
    def __init__(self, dataset_cfg=None):
        self.dimension = 4
        self.pcs = None
        self.dataset_cfg = dataset_cfg

    def from_point_cloud(self, pcs):
        self.pcs = pcs

    def from_carla_lidar(self, lidar:CarlaLiDAR):
        sensor_data = lidar.queue.get(True, 1.0)
        # print(sensor_data)
        if sensor_data is not None:
            lidar_output = lidar.realtime_data(sensor_data)
            self.pcs = lidar_output.points
            

    def preview_scene(self, vis=None):
        if vis is None:
            vis = open3d.visualization.Visualizer()
            vis.create_window()

        if self.pcs is not None:
            open3d_vis_utils.draw_scenes(vis,
                    points=self.pcs[:, 0:3], ref_boxes=None,
                    ref_scores=None, ref_labels=None, confidence=None, tracks=None
                )

        return vis

    
    def padding_resize(self):
        pass



class Detection3DOutput(object):

    def __init__(self, class_names, filtered_names, confidence_threshold, output_file=None, display=False, video_log=False):
        self.class_names = class_names
        self.filtered_names = filtered_names
        self.confidence_threshold = confidence_threshold
        self.display = display
        self.output_file = output_file
        self.video_log = video_log
        self.video_writer = None
        self.det_boxs = None
        self.det_labels = None                

    def from_output(self, det_input, model_output):
        original_image = det_input.image        
        top = det_input.padding_top
        bottom = det_input.padding_bottom
        left = det_input.padding_left
        right = det_input.padding_right
        scale_x = (det_input.resize - left - right) / det_input.resolution_x
        scale_y = (det_input.resize - top - bottom) / det_input.resolution_y
        self.det_boxes = model_output[0].tolist()[0]
        self.det_labels = model_output[1].tolist()[0]
        j = 0
        self.det_boxes_scaled = []
        self.det_labels_scaled = []
        for box in self.det_boxes:
            if box[4] >= self.confidence_threshold:                
                clas = self.class_names[self.det_labels[j]]
                x_min = (box[0] - left) / scale_x
                y_min = (box[1] - top) / scale_y
                x_max = (box[2] - left) / scale_x
                y_max = (box[3] - top) / scale_y
                self.det_boxes_scaled.append([x_min, y_min, x_max, y_max])
                self.det_labels_scaled.append(self.det_labels[j])
                # 创建可编辑图像
                # print(x_min, y_min)
                j += 1
                draw = ImageDraw.Draw(original_image)
                font = ImageFont.truetype("arial.ttf", 36)
                # 绘制矩形框
                draw.rectangle(xy=[(x_min, y_min), (x_max, y_max)], outline='red', width=3)
                draw.text((x_min, y_min), clas, font=font)
        self.annotated_image = original_image
        self.display_image()
    
    def display_image(self):
        # 将 PIL 图像转换为 OpenCV 图像
        cv_image = cv2.cvtColor(np.array(self.annotated_image), cv2.COLOR_RGB2BGR)        

        if self.display:
            # 在窗口中显示图像
            cv2.imshow('Realtime Image Display', cv_image)
            cv2.waitKey(1)  # 保持图像显示的时间间隔，单位为毫秒

        if self.video_log:
            # 如果视频写入器未初始化，则根据第一张图像的大小创建写入器
            if self.video_writer is None:
                height, width, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 可根据需要修改编码器
                self.video_writer = cv2.VideoWriter(self.output_file, fourcc, 30.0, (width, height))
            # 将图像写入视频文件
            self.video_writer.write(cv_image)