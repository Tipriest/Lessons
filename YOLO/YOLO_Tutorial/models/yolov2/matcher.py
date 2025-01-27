import torch
import numpy as np


class Yolov2Matcher(object):
    def __init__(self, iou_thresh, num_classes, anchor_size):
        self.num_classes = num_classes
        self.iou_thresh = iou_thresh
        # anchor box
        self.num_anchors = len(anchor_size)
        self.anchor_size = anchor_size
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

        # 计算先验框和目标框之间的交集
        inter_w = np.minimum(anchors[:, 2], gt_box_[:, 2]) - \
                  np.maximum(anchors[:, 0], gt_box_[:, 0])
        inter_h = np.minimum(anchors[:, 3], gt_box_[:, 3]) - \
                  np.maximum(anchors[:, 1], gt_box_[:, 1])
        inter_area = inter_w * inter_h
        
        # 计算先验框和目标框之间的并集
        union_area = anchors_area + gt_box_area - inter_area

        # 计算先验框和目标框之间的交并比
        iou = inter_area / union_area
        iou = np.clip(iou, a_min=1e-10, a_max=1.0)
        
        return iou

    @torch.no_grad()
    def __call__(self, fmp_size, stride, targets):
        """
        输入参数的解释:
            img_size: (Int) 输入图像的尺寸
            stride:   (Int) YOLOv1网络的输出步长
            targets:  (List[Dict]) 为List类型，包含一批数据的标签，每一个数据标签为Dict类型，其主要的数据结构为：
                             dict{'boxes':  (torch.Tensor) [N, 4], 一张图像中的N个目标边界框坐标
                                  'labels': (torch.Tensor) [N,], 一张图像中的N个目标类别标签
                                  ...}
        """
        # 准备后续处理会用到的变量
        bs = len(targets)
        fmp_h, fmp_w = fmp_size
        gt_objectness = np.zeros([bs, fmp_h, fmp_w, self.num_anchors, 1]) 
        gt_classes = np.zeros([bs, fmp_h, fmp_w, self.num_anchors, self.num_classes]) 
        gt_bboxes = np.zeros([bs, fmp_h, fmp_w, self.num_anchors, 4])

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
                x1, y1, x2, y2 = gt_box

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

                    # 确定先验框的索引
                    iou_ind = np.argmax(iou)
                    anchor_idx = iou_ind

                    # 计算网格坐标
                    xc_s = xc / stride
                    yc_s = yc / stride
                    grid_x = int(xc_s)
                    grid_y = int(yc_s)

                    # 保存正样本的网格坐标，和对应的先验框的索引
                    label_assignment_results.append([grid_x, grid_y, anchor_idx])
                else:
                    # 情况2&3，至少有一个先验框和目标框的IoU大于给定的阈值
                    for iou_ind, iou_m in enumerate(iou_mask):
                        if iou_m:
                            # 先验框的索引
                            anchor_idx = iou_ind

                            # 计算网格坐标
                            xc_s = xc / stride
                            yc_s = yc / stride
                            grid_x = int(xc_s)
                            grid_y = int(yc_s)

                            # # 保存正样本的网格坐标，和对应的先验框的索引
                            label_assignment_results.append([grid_x, grid_y, anchor_idx])

                # 依据上述的先验框的标记，开始标记正样本的位置
                for result in label_assignment_results:
                    grid_x, grid_y, anchor_idx = result
                    if grid_x < fmp_w and grid_y < fmp_h:
                        # 标记objectness标签，即此处的网格有物体，对应一个正样本
                        gt_objectness[batch_index, grid_y, grid_x, anchor_idx] = 1.0
                        # 标记正样本处的类别标签，采用one-hot格式
                        cls_ont_hot = np.zeros(self.num_classes)
                        cls_ont_hot[int(gt_label)] = 1.0
                        gt_classes[batch_index, grid_y, grid_x, anchor_idx] = cls_ont_hot
                        # 标记正样本处的bbox标签
                        gt_bboxes[batch_index, grid_y, grid_x, anchor_idx] = np.array([x1, y1, x2, y2])

        # 将标签数据的shape从 [B, H, W, A， C] 的形式reshape成 [B, M, C] ，其中M = HWA，以便后续的处理
        gt_objectness = gt_objectness.reshape(bs, -1, 1)
        gt_classes = gt_classes.reshape(bs, -1, self.num_classes)
        gt_bboxes = gt_bboxes.reshape(bs, -1, 4)

        # 将numpy.array类型转换为torch.Tensor类型
        gt_objectness = torch.from_numpy(gt_objectness).float()
        gt_classes = torch.from_numpy(gt_classes).float()
        gt_bboxes = torch.from_numpy(gt_bboxes).float()

        return gt_objectness, gt_classes, gt_bboxes
