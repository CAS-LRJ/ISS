import torch
from .iou3d import iou3d_nms_cuda

class iou3dWrapper:
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def nms_gpu(boxes, scores, thresh, pre_maxsize=None, **kwargs):
        """
        :param boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
        :param scores: (N)
        :param thresh:
        :return:
        """
        assert boxes.shape[1] == 7
        order = scores.sort(0, descending=True)[1]
        if pre_maxsize is not None:
            order = order[:pre_maxsize]

        boxes = boxes[order].contiguous()
        # keep = torch.IntTensor(boxes.size(0))
        keep = torch.zeros(boxes.size(0)).int()

        num_out = iou3d_nms_cuda.nms_gpu(boxes, keep, thresh)

        return order[keep[:num_out].long().cuda()].contiguous(), None

    @staticmethod
    def class_agnostic_nms(box_scores, box_preds, score_thresh=None):
        src_box_scores = box_scores
        if score_thresh is not None:
            scores_mask = (box_scores >= score_thresh)
            box_scores = box_scores[scores_mask]
            box_preds = box_preds[scores_mask]

        selected = []
        if box_scores.shape[0] > 0:
            box_scores_nms, indices = torch.topk(box_scores, k=min(4096, box_scores.shape[0]))
            boxes_for_nms = box_preds[indices]
            keep_idx, selected_scores = iou3dWrapper.nms_gpu(
                    boxes_for_nms[:, 0:7], box_scores_nms, 0.01, 4096
            )
            selected = indices[keep_idx[:500]]

        if score_thresh is not None:
            original_idxs = scores_mask.nonzero().view(-1)
            selected = original_idxs[selected]
        return selected, src_box_scores[selected]