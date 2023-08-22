import numpy as np
cimport numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont
from time import sleep

class CameraOutput(object):

    def __init__(self, intrinsic=None, image=None):
        self.intrinsic = intrinsic
        self.image = image

    def visualize(self):
        if self.image == None:
            print("No image to visualize")
            return
        cv_image = cv2.cvtColor(np.array(self.annotated_image), cv2.COLOR_RGB2BGR)        
        # 在窗口中显示图像
        cv2.imshow('Realtime Image Display', cv_image)
        cv2.waitKey(10000)  # 保持图像显示的时间间隔，单位为毫秒
        print("Image displayed")
        sleep(1)

    


        # if self.video_log:
        #     # 如果视频写入器未初始化，则根据第一张图像的大小创建写入器
        #     if self.video_writer is None:
        #         height, width, _ = cv_image.shape
        #         fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 可根据需要修改编码器
        #         self.video_writer = cv2.VideoWriter(self.output_file, fourcc, 30.0, (width, height))
        #     # 将图像写入视频文件
        #     self.video_writer.write(cv_image)
    
    # def from_beamng_camera(self, camera):
    #     camera.poll_data()
    #     if camera.rgb_image != None:
    #         self.image = camera.rgb_image
    #         self.resolution_x, self.resolution_y = self.image.size
    #         self.resize_image = self.padding_resize()
    
    # def padding_resize(self):
    #     if self.image != None:
    #         image_numpy = np.array(self.image)
    #         shape = image_numpy.shape[:2]
    #         new_shape = (self.resize, self.resize)

    #         # Scale ratio (new / old)
    #         r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    #         # Compute padding
    #         ratio = r, r  # width, height ratios
    #         new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))

    #         dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    #         dw, dh = np.mod(dw, self.stride), np.mod(dh, self.stride)  # wh padding
    #         dw /= 2  # divide padding into 2 sides
    #         dh /= 2
    #         # print(dw, dh)
    #         if shape[::-1] != new_unpad:  # resize
    #             im = cv2.resize(image_numpy, new_unpad, interpolation=cv2.INTER_LINEAR)
    #         top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    #         left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    #         self.padding_top = top
    #         self.padding_bottom = bottom
    #         self.padding_left = left
    #         self.padding_right = right
    #         im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=self.colour)  # add border
    #         return Image.fromarray(im.astype('uint8')).convert('RGB')


# class Detection2DInput(object):
    
#     def __init__(self):
#         self.resize = 320        
#         self.stride = 320
#         self.colour = (114, 114, 114)
#         self.image = None
    
#     def __init__(self, resize, stride, colour=(114, 114, 114)):
#         self.resize = resize        
#         self.stride = stride
#         self.colour = colour
#         self.image = None        

#     def from_image(self, image):
#         self.image = image
#         self.resolution_x, self.resolution_y = image.size
#         self.resize_image = self.padding_resize()



# class Detection2DOutput(object):

#     def __init__(self, class_names, filtered_names, confidence_threshold, output_file=None, display=False, video_log=False):
#         self.class_names = class_names
#         self.filtered_names = filtered_names
#         self.confidence_threshold = confidence_threshold
#         self.display = display
#         self.output_file = output_file
#         self.video_log = video_log
#         self.video_writer = None
#         self.det_boxs = None
#         self.det_labels = None                

#     def from_output(self, det_input, model_output):
#         original_image = det_input.image        
#         top = det_input.padding_top
#         bottom = det_input.padding_bottom
#         left = det_input.padding_left
#         right = det_input.padding_right
#         scale_x = (det_input.resize - left - right) / det_input.resolution_x
#         scale_y = (det_input.resize - top - bottom) / det_input.resolution_y
#         self.det_boxes = model_output[0].tolist()[0]
#         self.det_labels = model_output[1].tolist()[0]
#         j = 0
#         self.det_boxes_scaled = []
#         self.det_labels_scaled = []
#         for box in self.det_boxes:
#             if box[4] >= self.confidence_threshold:                
#                 clas = self.class_names[self.det_labels[j]]
#                 x_min = (box[0] - left) / scale_x
#                 y_min = (box[1] - top) / scale_y
#                 x_max = (box[2] - left) / scale_x
#                 y_max = (box[3] - top) / scale_y
#                 self.det_boxes_scaled.append([x_min, y_min, x_max, y_max])
#                 self.det_labels_scaled.append(self.det_labels[j])
#                 # 创建可编辑图像
#                 # print(x_min, y_min)
#                 j += 1
#                 draw = ImageDraw.Draw(original_image)
#                 font = ImageFont.truetype("arial.ttf", 36)
#                 # 绘制矩形框
#                 draw.rectangle(xy=[(x_min, y_min), (x_max, y_max)], outline='red', width=3)
#                 draw.text((x_min, y_min), clas, font=font)
#         self.annotated_image = original_image
#         self.display_image()
    
#     def display_image(self):
#         # 将 PIL 图像转换为 OpenCV 图像
#         cv_image = cv2.cvtColor(np.array(self.annotated_image), cv2.COLOR_RGB2BGR)        

#         if self.display:
#             # 在窗口中显示图像
#             cv2.imshow('Realtime Image Display', cv_image)
#             cv2.waitKey(1)  # 保持图像显示的时间间隔，单位为毫秒

#         if self.video_log:
#             # 如果视频写入器未初始化，则根据第一张图像的大小创建写入器
#             if self.video_writer is None:
#                 height, width, _ = cv_image.shape
#                 fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 可根据需要修改编码器
#                 self.video_writer = cv2.VideoWriter(self.output_file, fourcc, 30.0, (width, height))
#             # 将图像写入视频文件
#             self.video_writer.write(cv_image)