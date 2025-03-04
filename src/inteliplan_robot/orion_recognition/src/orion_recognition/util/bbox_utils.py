import torch
from collections import defaultdict
from typing import Any, List, Tuple
import torchvision.ops as ops


# When performing non-maximum suppression, the intersection-over-union threshold defines
# the proportion of intersection a bounding box must cover before it is determined to be
# part of the same object.
iou_threshold_general = 0.9        # Generally be conservative for bbox overlap for applying general NMS
iou_threshold_per_label = 0.1      # If the bboxes are for the same label, apply NMS even for a slight overlap

labelwise_nms_only = True


def non_max_supp(bbox_tuples: List[Tuple[tuple, str, float, Any]]):
    if not bbox_tuples:
        return []

    bbox_tuples_per_label = defaultdict(list)

    if labelwise_nms_only:
        for box, label, score, info in bbox_tuples:
            bbox_tuples_per_label[label].append((box, score, info))
    else:
        # Perform non-maximum suppression on boxes according to their intersection over union (IoU)
        bboxes, labels, scores, infos = list(zip(*bbox_tuples))
        keep_indices = ops.nms(torch.as_tensor(bboxes), torch.as_tensor(scores, dtype=torch.float), iou_threshold_general)

        for i in keep_indices:
            box, label, score, info = bboxes[i], labels[i], scores[i], infos[i]
            bbox_tuples_per_label[label].append((box, score, info))

    # Perform non-maximum suppression on boxes withaccording to their intersection over union (IoU)
    clean_bbox_tuples = []
    for label, bbox_tuples in bbox_tuples_per_label.items():
        bboxes, scores, infos = list(zip(*bbox_tuples))
        keep_indices = ops.nms(torch.as_tensor(bboxes), torch.as_tensor(scores, dtype=torch.float), iou_threshold_per_label)
        for i in keep_indices:
            box, score, info = bboxes[i], scores[i], infos[i]
            clean_bbox_tuples.append((box, label, score, info))

    return clean_bbox_tuples
