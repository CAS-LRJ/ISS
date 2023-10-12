import numpy as np
import open3d
cimport numpy as np
import cv2
import torch
from functools import partial
from collections import defaultdict
from PIL import Image, ImageDraw, ImageFont

from ISS.algorithms.sensors.carla_lidar import CarlaLiDAR
from ISS.algorithms.utils.spconvutils.VoxelGeneratorWrapper import VoxelGeneratorWrapper
from ISS.algorithms.utils.visutils.open3d_vis_utils import open3d_vis_utils
from ISS.algorithms.utils.postprocessingutils.iou3dwrapper import iou3dWrapper
from ISS.algorithms.perception.torchscriptwrapper import torchScriptWrapper

class Detection3DInput(object):
    
    def __init__(self, dataset_cfg=None):
        self.dimension = 4
        self.points = None
        self.input_dict = None
        self.dataset_cfg = dataset_cfg
        self.mode = "test"

        self.point_cloud_range = np.array(self.dataset_cfg.POINT_CLOUD_RANGE, dtype=np.float32)
        self.num_point_features = self.dataset_cfg.NUM_POINT_FEATURES
        
        self.grid_size = self.voxel_size = None
        self.voxel_generator = None

    def transform_points_to_voxels(self, data_dict=None, config=None):
        if data_dict is None:
            grid_size = (self.point_cloud_range[3:6] - self.point_cloud_range[0:3]) / np.array(config.VOXEL_SIZE)
            self.grid_size = np.round(grid_size).astype(np.int64)
            self.voxel_size = config.VOXEL_SIZE
            # just bind the config, we will create the VoxelGeneratorWrapper later,
            # to avoid pickling issues in multiprocess spawn
            return partial(self.transform_points_to_voxels, config=config)

        if self.voxel_generator is None:
            self.voxel_generator = VoxelGeneratorWrapper(
                vsize_xyz=config.VOXEL_SIZE,
                coors_range_xyz=self.point_cloud_range,
                num_point_features=self.num_point_features,
                max_num_points_per_voxel=config.MAX_POINTS_PER_VOXEL,
                max_num_voxels=config.MAX_NUMBER_OF_VOXELS[self.mode],
            )

        points = data_dict['points']
        voxel_output = self.voxel_generator.generate(points)
        voxels, coordinates, num_points = voxel_output

        if not data_dict['use_lead_xyz']:
            voxels = voxels[..., 3:]  # remove xyz in voxels(N, 3)

        if config.get('DOUBLE_FLIP', False):
            voxels_list, voxel_coords_list, voxel_num_points_list = [voxels], [coordinates], [num_points]
            points_yflip, points_xflip, points_xyflip = self.double_flip(points)
            points_list = [points_yflip, points_xflip, points_xyflip]
            keys = ['yflip', 'xflip', 'xyflip']
            for i, key in enumerate(keys):
                voxel_output = self.voxel_generator.generate(points_list[i])
                voxels, coordinates, num_points = voxel_output

                if not data_dict['use_lead_xyz']:
                    voxels = voxels[..., 3:]
                voxels_list.append(voxels)
                voxel_coords_list.append(coordinates)
                voxel_num_points_list.append(num_points)

            data_dict['voxels'] = voxels_list
            data_dict['voxel_coords'] = voxel_coords_list
            data_dict['voxel_num_points'] = voxel_num_points_list
        else:
            data_dict['voxels'] = voxels
            data_dict['voxel_coords'] = coordinates
            data_dict['voxel_num_points'] = num_points
        return data_dict
    

    def mask_points_by_range(self, points, limit_range):
        mask = (points[:, 0] >= limit_range[0]) & (points[:, 0] <= limit_range[3]) \
            & (points[:, 1] >= limit_range[1]) & (points[:, 1] <= limit_range[4])
        return mask

    @staticmethod
    def load_data_to_gpu(batch_dict):
        for key, val in batch_dict.items():
            if not isinstance(val, np.ndarray):
                batch_dict[key] = torch.from_numpy(np.array([val])).cuda()
                continue
            elif key in ['frame_id', 'metadata', 'calib']:
                batch_dict[key] = torch.from_numpy(val).cuda()
            # elif key in ['images']:
            #     batch_dict[key] = kornia.image_to_tensor(val).float().cuda().contiguous()
            elif key in ['image_shape']:
                batch_dict[key] = torch.from_numpy(val).int().cuda()
            else:
                batch_dict[key] = torch.from_numpy(val).float().cuda()

    @staticmethod
    def collate_batch(batch_list, _unused=False):
        data_dict = defaultdict(list)
        for cur_sample in batch_list:
            for key, val in cur_sample.items():
                data_dict[key].append(val)
        batch_size = len(batch_list)
        ret = {}
        batch_size_ratio = 1

        for key, val in data_dict.items():
            try:
                if key in ['voxels', 'voxel_num_points']:
                    if isinstance(val[0], list):
                        batch_size_ratio = len(val[0])
                        val = [i for item in val for i in item]
                    ret[key] = np.concatenate(val, axis=0)
                elif key in ['points', 'voxel_coords']:
                    coors = []
                    if isinstance(val[0], list):
                        val =  [i for item in val for i in item]
                    for i, coor in enumerate(val):
                        coor_pad = np.pad(coor, ((0, 0), (1, 0)), mode='constant', constant_values=i)
                        coors.append(coor_pad)
                    ret[key] = np.concatenate(coors, axis=0)
                elif key in ['gt_boxes']:
                    max_gt = max([len(x) for x in val])
                    batch_gt_boxes3d = np.zeros((batch_size, max_gt, val[0].shape[-1]), dtype=np.float32)
                    for k in range(batch_size):
                        batch_gt_boxes3d[k, :val[k].__len__(), :] = val[k]
                    ret[key] = batch_gt_boxes3d

                elif key in ['roi_boxes']:
                    max_gt = max([x.shape[1] for x in val])
                    batch_gt_boxes3d = np.zeros((batch_size, val[0].shape[0], max_gt, val[0].shape[-1]), dtype=np.float32)
                    for k in range(batch_size):
                        batch_gt_boxes3d[k,:, :val[k].shape[1], :] = val[k]
                    ret[key] = batch_gt_boxes3d

                elif key in ['roi_scores', 'roi_labels']:
                    max_gt = max([x.shape[1] for x in val])
                    batch_gt_boxes3d = np.zeros((batch_size, val[0].shape[0], max_gt), dtype=np.float32)
                    for k in range(batch_size):
                        batch_gt_boxes3d[k,:, :val[k].shape[1]] = val[k]
                    ret[key] = batch_gt_boxes3d

                elif key in ['gt_boxes2d']:
                    max_boxes = 0
                    max_boxes = max([len(x) for x in val])
                    batch_boxes2d = np.zeros((batch_size, max_boxes, val[0].shape[-1]), dtype=np.float32)
                    for k in range(batch_size):
                        if val[k].size > 0:
                            batch_boxes2d[k, :val[k].__len__(), :] = val[k]
                    ret[key] = batch_boxes2d
                elif key in ["images", "depth_maps"]:
                    raise NotImplementedError
                    # Get largest image size (H, W)
                    max_h = 0
                    max_w = 0
                    for image in val:
                        max_h = max(max_h, image.shape[0])
                        max_w = max(max_w, image.shape[1])

                    # Change size of images
                    images = []
                    for image in val:
                        pad_h = common_utils.get_pad_params(desired_size=max_h, cur_size=image.shape[0])
                        pad_w = common_utils.get_pad_params(desired_size=max_w, cur_size=image.shape[1])
                        pad_width = (pad_h, pad_w)
                        pad_value = 0

                        if key == "images":
                            pad_width = (pad_h, pad_w, (0, 0))
                        elif key == "depth_maps":
                            pad_width = (pad_h, pad_w)

                        image_pad = np.pad(image,
                                           pad_width=pad_width,
                                           mode='constant',
                                           constant_values=pad_value)

                        images.append(image_pad)
                    ret[key] = np.stack(images, axis=0)
                elif key in ['calib']:
                    ret[key] = val
                elif key in ["points_2d"]:
                    max_len = max([len(_val) for _val in val])
                    pad_value = 0
                    points = []
                    for _points in val:
                        pad_width = ((0, max_len-len(_points)), (0,0))
                        points_pad = np.pad(_points,
                                pad_width=pad_width,
                                mode='constant',
                                constant_values=pad_value)
                        points.append(points_pad)
                    ret[key] = np.stack(points, axis=0)
                else:
                    ret[key] = np.stack(val, axis=0)
            except:
                print('Error in collate_batch: key=%s' % key)
                raise TypeError

        ret['batch_size'] = batch_size * batch_size_ratio
        return ret
    
    def prepare_data(self, data_dict):
        
        if data_dict.get('points', None) is not None:
            mask = self.mask_points_by_range(data_dict['points'], self.point_cloud_range)
            data_dict['points'] = data_dict['points'][mask]
            data_dict['use_lead_xyz'] = True

        for process in self.dataset_cfg.DATA_PROCESSOR:
            if process["NAME"] is None:
                continue
            
            process_method = getattr(self, process["NAME"])

            if process_method is not None and callable(process_method):
                data_dict = process_method(data_dict, process)
            else:
                raise NotImplementedError
        
        return data_dict

    def from_point_cloud(self, points):
        self.points = points

    def from_carla_lidar(self, lidar:CarlaLiDAR):
        sensor_data = lidar.queue.get(True, 1.0)
        # print(sensor_data)
        if sensor_data is not None:
            lidar_output = lidar.realtime_data(sensor_data)
            self.points = lidar_output.points
            input_dict = {
            'points': None,
            'intensity': None,
            }

            if self.points is not None:
                points = self.points.reshape(-1, 4)
                input_dict['intensity'] = points[:, 3]
                points[:, 3] = 0
                input_dict['points'] = points

                input_dict = self.prepare_data(input_dict)
            
            self.data_dict = input_dict
            

    def preview_scene(self, vis=None):
        if vis is None:
            vis = open3d.visualization.Visualizer()
            vis.create_window()

        if self.data_dict is not None:
            open3d_vis_utils.draw_scenes(vis,
                    points=self.data_dict['points'][:, 0:3], ref_boxes=None,
                    ref_scores=None, ref_labels=None, confidence=None, tracks=None
                )

        return vis




class Detection3DOutput(object):

    def __init__(self, class_names, confidence_threshold):
        self.class_names = class_names
        self.confidence_threshold = confidence_threshold
        self.final_scores = None
        self.final_labels = None
        self.final_boxes = None

    def from_output(self, det_input, model):
        self.data_dict = det_input.data_dict
        with torch.no_grad():
            if self.data_dict["points"] is not None:
                self.data_dict = det_input.collate_batch([self.data_dict])
                det_input.load_data_to_gpu(self.data_dict)
                # print(self.data_dict["voxels"].size())
                # print(self.data_dict["points"].size())

                pred_dicts, _ = model.forward(self.data_dict)
                
                cls_preds = pred_dicts["pred_scores"]
                box_preds = pred_dicts["pred_boxes"]
                label_preds = pred_dicts["pred_labels"]

                selected, selected_scores = iou3dWrapper.class_agnostic_nms(
                                    box_scores=cls_preds, box_preds=box_preds,
                                    score_thresh=0.01
                                )

                self.final_scores = selected_scores
                self.final_labels = label_preds[selected]
                self.final_boxes = box_preds[selected]

                for i in range(self.class_names.__len__()):
                    class_selected = self.final_labels != i
                    score_selected = self.final_scores >= self.confidence_threshold[i]
                    self.final_scores = self.final_scores[class_selected | score_selected]
                    self.final_labels = self.final_labels[class_selected | score_selected]
                    self.final_boxes = self.final_boxes[class_selected | score_selected]
    
    def preview_scene(self, vis=None):
        if vis is None:
            vis = open3d.visualization.Visualizer()
            vis.create_window()

        if self.data_dict is not None:
            open3d_vis_utils.draw_scenes(vis,
                    points=self.data_dict['points'][:, 1:], ref_boxes=self.final_boxes,
                    ref_scores=self.final_scores, ref_labels=self.final_labels, confidence=None, tracks=None
                )

        return vis