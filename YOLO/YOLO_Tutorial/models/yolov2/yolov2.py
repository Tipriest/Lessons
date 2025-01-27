import torch
import torch.nn as nn
import numpy as np

from utils.misc import multiclass_nms

from .yolov2_backbone import build_backbone
from .yolov2_neck import build_neck
from .yolov2_head import build_head


# YOLOv2
class YOLOv2(nn.Module):
    def __init__(self,
                 cfg,
                 device,
                 num_classes=20,
                 conf_thresh=0.01,
                 nms_thresh=0.5,
                 topk=100,
                 trainable=False,
                 deploy=False):
        super(YOLOv2, self).__init__()
        # ------------------- 基础参数 -------------------
        self.cfg = cfg                     # 模型配置文件
        self.device = device               # cuda或者是cpu
        self.num_classes = num_classes     # 类别的数量
        self.trainable = trainable         # 训练的标记
        self.conf_thresh = conf_thresh     # 得分阈值
        self.nms_thresh = nms_thresh       # NMS阈值
        self.topk = topk                   # topk
        self.stride = 32                   # 网络的最大步长
        self.deploy = deploy
        # ------------------- Anchor box -------------------
        self.anchor_size = torch.as_tensor(cfg['anchor_size']).float().view(-1, 2) # [A, 2]
        self.num_anchors = self.anchor_size.shape[0]
        
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
        self.obj_pred = nn.Conv2d(head_dim, 1*self.num_anchors, kernel_size=1)
        self.cls_pred = nn.Conv2d(head_dim, num_classes*self.num_anchors, kernel_size=1)
        self.reg_pred = nn.Conv2d(head_dim, 4*self.num_anchors, kernel_size=1)
    
    def generate_anchors(self, fmp_size):
        """ 
            用于生成G矩阵，其中每个元素都是特征图上的像素坐标和先验框的尺寸。
        """
        fmp_h, fmp_w = fmp_size

        # generate grid cells
        anchor_y, anchor_x = torch.meshgrid([torch.arange(fmp_h), torch.arange(fmp_w)])
        anchor_xy = torch.stack([anchor_x, anchor_y], dim=-1).float().view(-1, 2)
        # [HW, 2] -> [HW, A, 2] -> [M, 2]
        anchor_xy = anchor_xy.unsqueeze(1).repeat(1, self.num_anchors, 1)
        anchor_xy = anchor_xy.view(-1, 2).to(self.device)

        # [A, 2] -> [1, A, 2] -> [HW, A, 2] -> [M, 2]
        anchor_wh = self.anchor_size.unsqueeze(0).repeat(fmp_h*fmp_w, 1, 1)
        anchor_wh = anchor_wh.view(-1, 2).to(self.device)

        anchors = torch.cat([anchor_xy, anchor_wh], dim=-1)

        return anchors
        
    def decode_boxes(self, anchors, pred_reg):
        """
            将YOLO预测的 (tx, ty)、(tw, th) 转换为bbox的左上角坐标 (x1, y1) 和右下角坐标 (x2, y2)。
            输入:
                pred_reg: (torch.Tensor) -> [B, HxWxA, 4] or [HxWxA, 4]，网络预测的txtytwth
                fmp_size: (List[int, int])，包含输出特征图的宽度和高度两个参数
            输出:
                pred_box: (torch.Tensor) -> [B, HxWxA, 4] or [HxWxA, 4]，解算出的边界框坐标
        """
        # 计算预测边界框的中心点坐标和宽高
        pred_ctr = (torch.sigmoid(pred_reg[..., :2]) + anchors[..., :2]) * self.stride
        pred_wh = torch.exp(pred_reg[..., 2:]) * anchors[..., 2:]

        # 将所有bbox的中心带你坐标和宽高换算成x1y1x2y2形式
        pred_x1y1 = pred_ctr - pred_wh * 0.5
        pred_x2y2 = pred_ctr + pred_wh * 0.5
        pred_box = torch.cat([pred_x1y1, pred_x2y2], dim=-1)

        return pred_box

    def postprocess(self, obj_pred, cls_pred, reg_pred, anchors):
        """
            后处理环节，包括<阈值筛选>和<非极大值抑制(NMS)>两个环节
            输入:
                obj_pred: (torch.Tensor) -> [HxWxA, 4]
                cls_pred: (torch.Tensor) -> [HxWxA, 4]
                reg_pred: (torch.Tensor) -> [HxWxA, num_classes]
            输出:
                bboxes: (numpy.array) -> [N, 4]
                score:  (numpy.array) -> [N,]
                labels: (numpy.array) -> [N,]
        """
        # Reshape: [H x W x A, C] -> [H x W x A x C,]
        scores = torch.sqrt(obj_pred.sigmoid() * cls_pred.sigmoid()).flatten()

        # 依据得分scores，保留前topk的预测
        num_topk = min(self.topk, reg_pred.size(0))
        predicted_prob, topk_idxs = scores.sort(descending=True)
        topk_scores = predicted_prob[:num_topk]
        topk_idxs = topk_idxs[:num_topk]

        # 滤除掉得分低于指定阈值的预测
        keep_idxs = topk_scores > self.conf_thresh
        scores = topk_scores[keep_idxs]
        topk_idxs = topk_idxs[keep_idxs]

        # 获得最终结果的预测标签
        anchor_idxs = torch.div(topk_idxs, self.num_classes, rounding_mode='floor')
        labels = topk_idxs % self.num_classes

        # 获得最终结果的预测bbox
        reg_pred = reg_pred[anchor_idxs]
        anchors = anchors[anchor_idxs]
        bboxes = self.decode_boxes(anchors, reg_pred)

        # 将预测结果都放在cpu上，并转为numpy.array格式
        scores = scores.cpu().numpy()
        labels = labels.cpu().numpy()
        bboxes = bboxes.cpu().numpy()

        # 做NMS处理，滤除冗余检测结果
        scores, labels, bboxes = multiclass_nms(
            scores, labels, bboxes, self.nms_thresh, self.num_classes, False)

        return bboxes, scores, labels

    @torch.no_grad()
    def inference(self, x):
        bs = x.shape[0]
        # 主干网络
        feat = self.backbone(x)

        # 颈部网络
        feat = self.neck(feat)

        # 检测头
        cls_feat, reg_feat = self.head(feat)

        # 预测层
        obj_pred = self.obj_pred(reg_feat)
        cls_pred = self.cls_pred(cls_feat)
        reg_pred = self.reg_pred(reg_feat)
        fmp_size = obj_pred.shape[-2:]

        # anchors: [M, 2]
        anchors = self.generate_anchors(fmp_size)

        # 对 pred 的 size 做一些 view 调整，便于后续的处理
        # [B, A*C, H, W] -> [B, H, W, A*C] -> [B, H*W*A, C]
        obj_pred = obj_pred.permute(0, 2, 3, 1).contiguous().view(bs, -1, 1)
        cls_pred = cls_pred.permute(0, 2, 3, 1).contiguous().view(bs, -1, self.num_classes)
        reg_pred = reg_pred.permute(0, 2, 3, 1).contiguous().view(bs, -1, 4)

        # 测试时，笔者默认batch是1，
        # 因此，我们不需要用batch这个维度，用[0]将其取走。
        obj_pred = obj_pred[0]       # [H*W*A, 1]
        cls_pred = cls_pred[0]       # [H*W*A, NC]
        reg_pred = reg_pred[0]       # [H*W*A, 4]

        if self.deploy:
            # 这段代码和ONNX部署有关，读者不必关注这段if的代码
            scores = torch.sqrt(obj_pred.sigmoid() * cls_pred.sigmoid())
            bboxes = self.decode_boxes(anchors, reg_pred)
            # [n_anchors_all, 4 + C]
            outputs = torch.cat([bboxes, scores], dim=-1)

            return outputs
        else:
            # 后处理
            bboxes, scores, labels = self.postprocess(
                obj_pred, cls_pred, reg_pred, anchors)

            return bboxes, scores, labels

    def forward(self, x):
        if not self.trainable:
            return self.inference(x)
        else:
            bs = x.shape[0]
            # 主干网络
            feat = self.backbone(x)

            # 颈部网络
            feat = self.neck(feat)

            # 检测头
            cls_feat, reg_feat = self.head(feat)

            # 预测层
            obj_pred = self.obj_pred(reg_feat)
            cls_pred = self.cls_pred(cls_feat)
            reg_pred = self.reg_pred(reg_feat)
            fmp_size = obj_pred.shape[-2:]

            # anchors: [M, 2]
            anchors = self.generate_anchors(fmp_size)

            # 对 pred 的 size 做一些 view 调整，便于后续的处理
            # [B, A*C, H, W] -> [B, H, W, A*C] -> [B, H*W*A, C]
            obj_pred = obj_pred.permute(0, 2, 3, 1).contiguous().view(bs, -1, 1)
            cls_pred = cls_pred.permute(0, 2, 3, 1).contiguous().view(bs, -1, self.num_classes)
            reg_pred = reg_pred.permute(0, 2, 3, 1).contiguous().view(bs, -1, 4)

            # 解算边界框坐标
            box_pred = self.decode_boxes(anchors, reg_pred)

            # 网络输出
            outputs = {"pred_obj": obj_pred,        # (torch.Tensor) [B, M, 1]
                       "pred_cls": cls_pred,        # (torch.Tensor) [B, M, C]
                       "pred_box": box_pred,        # (torch.Tensor) [B, M, 4]
                       "stride": self.stride,       # (Int)
                       "fmp_size": fmp_size         # (List[int, int])
                       }           
            return outputs
        