#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import torch
import torch.nn as nn

from .loss import build_criterion
from .yolox import YOLOX


# 构建 YOLOX 网络
def build_yolox(args, cfg, device, num_classes=80, trainable=False, deploy=False):
    print('==============================')
    print('Build {} ...'.format(args.model.upper()))
    
    print('==============================')
    print('Model Configuration: \n', cfg)
    
    # -------------- 构建YOLOX --------------
    model = YOLOX(
        cfg=cfg,
        device=device, 
        num_classes=num_classes,
        conf_thresh=args.conf_thresh,
        nms_thresh=args.nms_thresh,
        topk=args.topk,
        trainable = trainable,
        deploy = deploy
        )

    # -------------- 初始化YOLOX的部分网络参数 --------------
    ## 初始化YOLOX的所有的BN层的eps和momentum参数
    for m in model.modules():
        if isinstance(m, nn.BatchNorm2d):
            m.eps = 1e-3
            m.momentum = 0.03    
    ## 初始化YOLOX的预测层的weight和bias
    init_prob = 0.01
    bias_value = -torch.log(torch.tensor((1. - init_prob) / init_prob))
    ## obj pred
    for obj_pred in model.obj_preds:
        b = obj_pred.bias.view(1, -1)
        b.data.fill_(bias_value.item())
        obj_pred.bias = torch.nn.Parameter(b.view(-1), requires_grad=True)
    ## cls pred
    for cls_pred in model.cls_preds:
        b = cls_pred.bias.view(1, -1)
        b.data.fill_(bias_value.item())
        cls_pred.bias = torch.nn.Parameter(b.view(-1), requires_grad=True)
    ## reg pred
    for reg_pred in model.reg_preds:
        b = reg_pred.bias.view(-1, )
        b.data.fill_(1.0)
        reg_pred.bias = torch.nn.Parameter(b.view(-1), requires_grad=True)
        w = reg_pred.weight
        w.data.fill_(0.)
        reg_pred.weight = torch.nn.Parameter(w, requires_grad=True)


    # -------------- 构建用于计算标签分配和计算损失的Criterion类 --------------
    criterion = None
    if trainable:
        # build criterion for training
        criterion = build_criterion(args, cfg, device, num_classes)
        
    return model, criterion
