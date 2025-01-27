import numpy as np
import torch


class Yolov3Matcher(object):
    def __init__(self, num_classes, num_anchors, anchor_size, iou_thresh):
        self.num_classes = num_classes
        self.num_anchors = num_anchors
        self.iou_thresh = iou_thresh
        self.anchor_boxes = np.array(
            [[0., 0., anchor[0], anchor[1]]
            for anchor in anchor_size]
            )  # [KA, 4]

    def compute_iou(self, anchor_boxes, gt_box):
        """
            anchor_boxes: (numpy.array) -> [KA, 4] (cx, cy, bw, bh).
            gt_box:       (numpy.array) -> [1, 4] (cx, cy, bw, bh).
        """
        # anchors: [KA, 4]
        anchors = np.zeros_like(anchor_boxes)
        anchors[..., :2] = anchor_boxes[..., :2] - anchor_boxes[..., 2:] * 0.5  # x1y1
        anchors[..., 2:] = anchor_boxes[..., :2] + anchor_boxes[..., 2:] * 0.5  # x2y2
        anchors_area = anchor_boxes[..., 2] * anchor_boxes[..., 3]
        
        # gt_box: [1, 4] -> [KA, 4]
        gt_box = np.array(gt_box).reshape(-1, 4)
        gt_box = np.repeat(gt_box, anchors.shape[0], axis=0)
        gt_box_ = np.zeros_like(gt_box)
        gt_box_[..., :2] = gt_box[..., :2] - gt_box[..., 2:] * 0.5  # x1y1
        gt_box_[..., 2:] = gt_box[..., :2] + gt_box[..., 2:] * 0.5  # x2y2
        gt_box_area = np.prod(gt_box[..., 2:] - gt_box[..., :2], axis=1)

        # intersection
        inter_w = np.minimum(anchors[:, 2], gt_box_[:, 2]) - \
                  np.maximum(anchors[:, 0], gt_box_[:, 0])
        inter_h = np.minimum(anchors[:, 3], gt_box_[:, 3]) - \
                  np.maximum(anchors[:, 1], gt_box_[:, 1])
        inter_area = inter_w * inter_h
        
        # union
        union_area = anchors_area + gt_box_area - inter_area

        # iou
        iou = inter_area / union_area
        iou = np.clip(iou, a_min=1e-10, a_max=1.0)
        
        return iou

    @torch.no_grad()
    def __call__(self, fmp_sizes, fpn_strides, targets):
        """
        输入参数的解释:
            fmp_sizes:   (List[List[int, int], ...]) 多尺度特征图的尺寸
            fpn_strides: (List[Int, ...]) 多尺度特征图的输出步长
            targets:     (List[Dict]) 为List类型，包含一批数据的标签，每一个数据标签为Dict类型，其主要的数据结构为：
                             dict{'boxes':  (torch.Tensor) [N, 4], 一张图像中的N个目标边界框坐标
                                  'labels': (torch.Tensor) [N,], 一张图像中的N个目标类别标签
                                  ...}
        """
        assert len(fmp_sizes) == len(fpn_strides)
        # 准备后续处理会用到的变量
        bs = len(targets)
        gt_objectness = [
            torch.zeros([bs, fmp_h, fmp_w, self.num_anchors, 1]) 
            for (fmp_h, fmp_w) in fmp_sizes
            ]
        gt_classes = [
            torch.zeros([bs, fmp_h, fmp_w, self.num_anchors, self.num_classes]) 
            for (fmp_h, fmp_w) in fmp_sizes
            ]
        gt_bboxes = [
            torch.zeros([bs, fmp_h, fmp_w, self.num_anchors, 4]) 
            for (fmp_h, fmp_w) in fmp_sizes
            ]

        # 第一层for循环遍历每一张图像的标签
        for batch_index in range(bs):
            targets_per_image = targets[batch_index]
            # [N,]
            tgt_cls = targets_per_image["labels"].numpy()
            # [N, 4]
            tgt_box = targets_per_image['boxes'].numpy()

            # 第二层for循环遍历该张图像的每一个目标的标签
            for gt_box, gt_label in zip(tgt_box, tgt_cls):
                # 获得该目标的边界框坐标
                x1, y1, x2, y2 = gt_box.tolist()

                # 计算目标框的中心点坐标和宽高
                xc, yc = (x2 + x1) * 0.5, (y2 + y1) * 0.5
                bw, bh = x2 - x1, y2 - y1
                gt_box = [0, 0, bw, bh]

                # 检查该目标边界框是否有效
                if bw < 1. or bh < 1.:
                    continue    

                # 计算目标框和所有先验框之间的交并比
                iou = self.compute_iou(self.anchor_boxes, gt_box)
                iou_mask = (iou > self.iou_thresh)

                # 根据IoU结果，确定正样本的标记
                label_assignment_results = []
                if iou_mask.sum() == 0:
                    # 情况1，如果的先验框与目标框的iou值都较低，
                    # 此时，我们将iou最高的先验框标记为正样本
                    iou_ind = np.argmax(iou)

                    # 先验框所对应的特征金字塔的尺度(level)的标记
                    level = iou_ind // self.num_anchors              # pyramid level
                    # 先验框的索引
                    anchor_idx = iou_ind - level * self.num_anchors  # anchor index

                    # 对应尺度的输出步长
                    stride = fpn_strides[level]

                    # 计算网格坐标
                    xc_s = xc / stride
                    yc_s = yc / stride
                    grid_x = int(xc_s)
                    grid_y = int(yc_s)

                    label_assignment_results.append([grid_x, grid_y, level, anchor_idx])
                else:
                    # 情况2&3，至少有一个先验框和目标框的IoU大于给定的阈值
                    for iou_ind, iou_m in enumerate(iou_mask):
                        if iou_m:
                            # 先验框所对应的特征金字塔的尺度(level)的标记
                            level = iou_ind // self.num_anchors              # pyramid level
                            # 先验框的索引
                            anchor_idx = iou_ind - level * self.num_anchors  # anchor index

                            # 对应尺度的输出步长
                            stride = fpn_strides[level]

                            # 计算网格坐标
                            xc_s = xc / stride
                            yc_s = yc / stride
                            grid_x = int(xc_s)
                            grid_y = int(yc_s)

                            label_assignment_results.append([grid_x, grid_y, level, anchor_idx])

                # 依据上述的先验框的标记，开始标记正样本的位置
                for result in label_assignment_results:
                    grid_x, grid_y, level, anchor_idx = result
                    fmp_h, fmp_w = fmp_sizes[level]

                    if grid_x < fmp_w and grid_y < fmp_h:
                        # 标记objectness标签，即此处的网格有物体，对应一个正样本
                        gt_objectness[level][batch_index, grid_y, grid_x, anchor_idx] = 1.0
                        # 标记正样本处的类别标签，采用one-hot格式
                        cls_ont_hot = torch.zeros(self.num_classes)
                        cls_ont_hot[int(gt_label)] = 1.0
                        gt_classes[level][batch_index, grid_y, grid_x, anchor_idx] = cls_ont_hot
                        # 标记正样本处的bbox标签
                        gt_bboxes[level][batch_index, grid_y, grid_x, anchor_idx] = torch.as_tensor([x1, y1, x2, y2])

        # 首先，将每个尺度的标签数据的shape从 [B, H, W, A， C] 的形式reshape成 [B, M, C] ，其中M = HWA，以便后续的处理
        # 然后，将所有尺度的预测拼接在一起，方便后续的损失计算
        gt_objectness = torch.cat([gt.view(bs, -1, 1) for gt in gt_objectness], dim=1).float()
        gt_classes = torch.cat([gt.view(bs, -1, self.num_classes) for gt in gt_classes], dim=1).float()
        gt_bboxes = torch.cat([gt.view(bs, -1, 4) for gt in gt_bboxes], dim=1).float()

        return gt_objectness, gt_classes, gt_bboxes
