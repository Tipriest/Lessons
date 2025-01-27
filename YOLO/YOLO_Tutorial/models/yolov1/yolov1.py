import torch
import torch.nn as nn
import numpy as np

from utils.misc import multiclass_nms

from .yolov1_backbone import build_backbone
from .yolov1_neck import build_neck
from .yolov1_head import build_head


# YOLOv1
class YOLOv1(nn.Module):
    def __init__(self,
                 cfg,
                 device,
                 img_size=None,
                 num_classes=20,
                 conf_thresh=0.01,
                 nms_thresh=0.5,
                 trainable=False,
                 deploy=False):
        super(YOLOv1, self).__init__()
        # ------------------- 基础参数 -------------------
        self.cfg = cfg                     # 模型配置文件
        self.img_size = img_size           # 输入图像大小
        self.device = device               # cuda或者是cpu
        self.num_classes = num_classes     # 类别的数量
        self.trainable = trainable         # 训练的标记
        self.conf_thresh = conf_thresh     # 得分阈值
        self.nms_thresh = nms_thresh       # NMS阈值
        self.stride = 32                   # 网络的最大步长
        self.deploy = deploy
        
        # ------------------- 网络结构 -------------------
        ## 主干网络
        self.backbone, feat_dim = build_backbone(
            cfg['backbone'], trainable&cfg['pretrained'])

        ## 颈部网络
        self.neck = build_neck(cfg, feat_dim, out_dim=512)
        head_dim = self.neck.out_dim

        ## 检测头
        self.head = build_head(cfg, head_dim, head_dim, num_classes)

        ## 预测层
        self.obj_pred = nn.Conv2d(head_dim, 1, kernel_size=1)
        self.cls_pred = nn.Conv2d(head_dim, num_classes, kernel_size=1)
        self.reg_pred = nn.Conv2d(head_dim, 4, kernel_size=1)
    
    def create_grid(self, fmp_size):
        """ 
            用于生成G矩阵，其中每个元素都是特征图上的像素坐标。
        """
        # 特征图的宽和高
        ws, hs = fmp_size

        # 生成网格的x坐标和y坐标
        grid_y, grid_x = torch.meshgrid([torch.arange(hs), torch.arange(ws)])

        # 将xy两部分的坐标拼起来：[H, W, 2]
        grid_xy = torch.stack([grid_x, grid_y], dim=-1).float()

        # [H, W, 2] -> [HW, 2] -> [HW, 2]
        grid_xy = grid_xy.view(-1, 2).to(self.device)
        
        return grid_xy

    def decode_boxes(self, pred_reg, fmp_size):
        """
            将YOLO预测的 (tx, ty)、(tw, th) 转换为bbox的左上角坐标 (x1, y1) 和右下角坐标 (x2, y2)。
            输入:
                pred_reg: (torch.Tensor) -> [B, HxW, 4] or [HxW, 4]，网络预测的txtytwth
                fmp_size: (List[int, int])，包含输出特征图的宽度和高度两个参数
            输出:
                pred_box: (torch.Tensor) -> [B, HxW, 4] or [HxW, 4]，解算出的边界框坐标
        """
        # 生成网格坐标矩阵
        grid_cell = self.create_grid(fmp_size)

        # 计算预测边界框的中心点坐标和宽高
        pred_ctr = (torch.sigmoid(pred_reg[..., :2]) + grid_cell) * self.stride
        pred_wh = torch.exp(pred_reg[..., 2:]) * self.stride

        # 将所有bbox的中心带你坐标和宽高换算成x1y1x2y2形式
        pred_x1y1 = pred_ctr - pred_wh * 0.5
        pred_x2y2 = pred_ctr + pred_wh * 0.5
        pred_box = torch.cat([pred_x1y1, pred_x2y2], dim=-1)

        return pred_box

    def postprocess(self, bboxes, scores):
        """
            后处理环节，包括<阈值筛选>和<非极大值抑制(NMS)>两个环节
            输入:
                bboxes: (numpy.array) -> [HxW, 4]
                scores: (numpy.array) -> [HxW, num_classes]
            输出:
                bboxes: (numpy.array) -> [N, 4]
                score:  (numpy.array) -> [N,]
                labels: (numpy.array) -> [N,]
        """
        labels = np.argmax(scores, axis=1)
        scores = scores[(np.arange(scores.shape[0]), labels)]
        
        # threshold
        keep = np.where(scores >= self.conf_thresh)
        bboxes = bboxes[keep]
        scores = scores[keep]
        labels = labels[keep]

        # nms
        scores, labels, bboxes = multiclass_nms(
            scores, labels, bboxes, self.nms_thresh, self.num_classes, False)

        return bboxes, scores, labels

    @torch.no_grad()
    def inference(self, x):
        # 主干网络
        feat = self.backbone(x)

        # 颈部网络
        feat = self.neck(feat)

        # 检测头
        cls_feat, reg_feat = self.head(feat)

        # 预测层
        obj_pred = self.obj_pred(cls_feat)
        cls_pred = self.cls_pred(cls_feat)
        reg_pred = self.reg_pred(reg_feat)
        fmp_size = obj_pred.shape[-2:]

        # 对 pred 的 size 做一些 view 调整，便于后续的处理
        # [B, C, H, W] -> [B, H, W, C] -> [B, H*W, C]
        obj_pred = obj_pred.permute(0, 2, 3, 1).contiguous().flatten(1, 2)
        cls_pred = cls_pred.permute(0, 2, 3, 1).contiguous().flatten(1, 2)
        reg_pred = reg_pred.permute(0, 2, 3, 1).contiguous().flatten(1, 2)

        # 测试时，笔者默认 batch 是1，
        # 因此，我们不需要用batch这个维度，用[0]将其取走。
        obj_pred = obj_pred[0]       # [H*W, 1]
        cls_pred = cls_pred[0]       # [H*W, NC]
        reg_pred = reg_pred[0]       # [H*W, 4]

        # 每个边界框的得分
        scores = torch.sqrt(obj_pred.sigmoid() * cls_pred.sigmoid())
        
        # 解算边界框, 并归一化边界框: [H*W, 4]
        bboxes = self.decode_boxes(reg_pred, fmp_size)
        
        if self.deploy:
            # 这段代码和ONNX部署有关，读者不必关注这段if的代码
            # [n_anchors_all, 4 + C]
            outputs = torch.cat([bboxes, scores], dim=-1)

            return outputs
        else:
            # 将 bbox 和 score 预测都放在 cpu 处理上，以便进行后处理
            scores = scores.cpu().numpy()
            bboxes = bboxes.cpu().numpy()
            
            # 后处理
            bboxes, scores, labels = self.postprocess(bboxes, scores)

        return bboxes, scores, labels

    def forward(self, x):
        if not self.trainable:
            return self.inference(x)
        else:
            # 主干网络
            feat = self.backbone(x)

            # 颈部网络
            feat = self.neck(feat)

            # 检测头
            cls_feat, reg_feat = self.head(feat)

            # 预测层
            obj_pred = self.obj_pred(cls_feat)
            cls_pred = self.cls_pred(cls_feat)
            reg_pred = self.reg_pred(reg_feat)
            fmp_size = obj_pred.shape[-2:]

            # 对 pred 的 size 做一些 view 调整，便于后续的处理
            # [B, C, H, W] -> [B, H, W, C] -> [B, H*W, C]
            obj_pred = obj_pred.permute(0, 2, 3, 1).contiguous().flatten(1, 2)
            cls_pred = cls_pred.permute(0, 2, 3, 1).contiguous().flatten(1, 2)
            reg_pred = reg_pred.permute(0, 2, 3, 1).contiguous().flatten(1, 2)

            # 解算边界框坐标
            box_pred = self.decode_boxes(reg_pred, fmp_size)

            # 网络输出
            outputs = {"pred_obj": obj_pred,        # (torch.Tensor) [B, M, 1]
                       "pred_cls": cls_pred,        # (torch.Tensor) [B, M, C]
                       "pred_box": box_pred,        # (torch.Tensor) [B, M, 4]
                       "stride": self.stride,       # (Int)
                       "fmp_size": fmp_size         # (List[int, int])
                       }           
            return outputs
        