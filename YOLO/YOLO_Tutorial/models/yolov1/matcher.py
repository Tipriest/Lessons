import torch
import numpy as np


# YoloMatcher类用于完成训练阶段的<标签分配>
class YoloMatcher(object):
    def __init__(self, num_classes):
        self.num_classes = num_classes

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
        gt_objectness = np.zeros([bs, fmp_h, fmp_w, 1]) 
        gt_classes = np.zeros([bs, fmp_h, fmp_w, self.num_classes]) 
        gt_bboxes = np.zeros([bs, fmp_h, fmp_w, 4])

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

                # 检查该目标边界框是否有效
                if bw < 1. or bh < 1.:
                    continue    

                # 计算中心点所在的网格坐标
                xs_c = xc / stride
                ys_c = yc / stride
                grid_x = int(xs_c)
                grid_y = int(ys_c)

                #  检查网格坐标是否有效
                if grid_x < fmp_w and grid_y < fmp_h:
                    # 标记objectness标签，即此处的网格有物体，对应一个正样本
                    gt_objectness[batch_index, grid_y, grid_x] = 1.0

                    # 标记正样本处的类别标签，采用one-hot格式
                    cls_ont_hot = np.zeros(self.num_classes)
                    cls_ont_hot[int(gt_label)] = 1.0
                    gt_classes[batch_index, grid_y, grid_x] = cls_ont_hot

                    # 标记正样本处的bbox标签
                    gt_bboxes[batch_index, grid_y, grid_x] = np.array([x1, y1, x2, y2])

        # 将标签数据的shape从 [B, H, W, C] 的形式reshape成 [B, M, C] ，其中M = HW，以便后续的处理
        gt_objectness = gt_objectness.reshape(bs, -1, 1)
        gt_classes = gt_classes.reshape(bs, -1, self.num_classes)
        gt_bboxes = gt_bboxes.reshape(bs, -1, 4)

        # 将numpy.array类型转换为torch.Tensor类型
        gt_objectness = torch.from_numpy(gt_objectness).float()
        gt_classes = torch.from_numpy(gt_classes).float()
        gt_bboxes = torch.from_numpy(gt_bboxes).float()

        return gt_objectness, gt_classes, gt_bboxes
