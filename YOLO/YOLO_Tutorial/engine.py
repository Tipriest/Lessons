"""
    该python文件包含了若干个训练常用的Trainer类，包括以下两种：
    1. YOLOv8Trainer：该Trainer类主要用于训练YOLOv1~v5等YOLO模型，相关参数如optimizer参数、学习策略等均采用默认设置；
    2. YoloxTrainer：该Trainer类主要用于训练YOLOX和笔者实现的较为简单的YOLOv7模型，相关参数如optimizer参数、学习策略等均采用默认设置；
    读者可以根据自己的需求来调整所使用的Trainer类的参数
"""

import time
import os
import numpy as np
import random

# ----------------- Torch Components -----------------
import torch
import torch.distributed as dist

# ----------------- Extra Components -----------------
from utils import distributed_utils
from utils.misc import ModelEMA, CollateFunc, build_dataloader
from utils.vis_tools import vis_data

# ----------------- Evaluator Components -----------------
from evaluator.build import build_evluator

# ----------------- Optimizer & LrScheduler Components -----------------
from utils.solver.optimizer import build_yolo_optimizer
from utils.solver.lr_scheduler import build_lr_scheduler

# ----------------- Dataset Components -----------------
from dataset.build import build_dataset, build_transform


# YOLOv8 Trainer: 主要用于训练YOLOv1~v5等YOLO模型
class Yolov8Trainer(object):
    def __init__(self, args, data_cfg, model_cfg, trans_cfg, device, model, criterion, world_size):
        # ------------------- 基础参数 -------------------
        self.args          = args
        self.epoch         = 0
        self.best_map      = -1.
        self.last_opt_step = 0
        self.no_aug_epoch  = args.no_aug_epoch
        self.clip_grad     = 10
        self.device        = device
        self.criterion     = criterion
        self.world_size    = world_size
        self.heavy_eval    = False
        self.second_stage  = False

        # 创建路径，用于保存模型的训练文件
        self.path_to_save = os.path.join(args.save_folder, args.dataset, args.model)
        os.makedirs(self.path_to_save, exist_ok=True)

        # ---------------------------- YOLOv5和YOLOv8项目的超参数设置 ----------------------------
        ## 优化器的超参数
        self.optimizer_dict = {'optimizer': 'sgd',    # 使用SGD优化器; 若使用AdamW，则修改为'adamw'，但建议使用SGD优化器
                               'momentum': 0.937,     # SGD所需的momentum，AdamW优化器不需要此参数
                               'weight_decay': 5e-4,  # SGD所需的weight decay; 若使用AdamW，则修改为5e-2
                               'lr0': 0.01,           # 初始学习率; 若使用AdamW，则修改为0.001
                               }
        ## 学习策略的超参数
        self.lr_schedule_dict = {'scheduler': 'linear', # 使用YOLOv5&v8官方的线性衰减策略; 读者若想使用Cosine策略，则自改为'cosine'
                                 'lrf': 0.01,           # 最终学习率与初始学习率的比值，即最终学习率=lr0 * lrf; 若使用Cosine策略，则修改为0.05
                                 }
        ## 训练的Warmup阶段的超参数
        self.warmup_dict = {'warmup_momentum': 0.8,  # 使用YOLOv5&v8官方的超参数设定，建议保持默认设置
                            'warmup_bias_lr': 0.1,   # 使用YOLOv5&v8官方的超参数设定，建议保持默认设置
                            }
        ## EMA技巧的超参数，建议保持默认设置
        self.ema_dict = {'ema_decay': 0.9999,  # 使用YOLOv5&v8官方的超参数设定，建议保持默认设置
                         'ema_tau': 2000,      # 使用YOLOv5&v8官方的超参数设定，建议保持默认设置
                         }

        # ---------------------------- 构建Dataset、Model和Transforms所需的config变量 ----------------------------
        ## 数据集的config
        self.data_cfg = data_cfg
        ## 模型的config
        self.model_cfg = model_cfg
        ## 数据预处理的config
        self.trans_cfg = trans_cfg

        # ---------------------------- 构建数据预处理 Transform类 ----------------------------
        ## 构建训练(Train)所需的数据预处理
        self.train_transform, self.trans_cfg = build_transform(
            args=args, trans_config=self.trans_cfg, max_stride=model_cfg['max_stride'], is_train=True)
        ## 构建测试(Evaluate)所需的数据预处理
        self.val_transform, _ = build_transform(
            args=args, trans_config=self.trans_cfg, max_stride=model_cfg['max_stride'], is_train=False)

        # ---------------------------- 构建Dataset & Dataloader ----------------------------
        ## 构建Dataset，用于读取数据集的图像和标签
        self.dataset, self.dataset_info = build_dataset(self.args, self.data_cfg, self.trans_cfg, self.train_transform, is_train=True)
        ## 构建Dataloader，用于后续的训练
        self.train_loader = build_dataloader(self.args, self.dataset, self.args.batch_size // self.world_size, CollateFunc())

        # ---------------------------- 构建测试模型性能的Evaluator类 ----------------------------
        self.evaluator = build_evluator(self.args, self.data_cfg, self.val_transform, self.device)

        # ---------------------------- 构建梯度缩放器 ----------------------------
        self.scaler = torch.cuda.amp.GradScaler(enabled=self.args.fp16)

        # ---------------------------- 构建优化器 ----------------------------
        ## 梯度累加的次数
        accumulate = max(1, round(64 / self.args.batch_size))
        print('Grad Accumulate: {}'.format(accumulate))
        ## 依据有效的batch size，自适应调整weight decay
        self.optimizer_dict['weight_decay'] *= self.args.batch_size * accumulate / 64
        ## 构建YOLO项目的优化器
        self.optimizer, self.start_epoch = build_yolo_optimizer(self.optimizer_dict, model, self.args.resume)

        # ---------------------------- 构建学习率策略 ----------------------------
        ## 构建YOLO项目的学习率策略
        self.lr_scheduler, self.lf = build_lr_scheduler(self.lr_schedule_dict, self.optimizer, self.args.max_epoch)
        self.lr_scheduler.last_epoch = self.start_epoch - 1  # do not move
        if self.args.resume:
            self.lr_scheduler.step()

        # ---------------------------- 构建 Model-EMA ----------------------------
        if self.args.ema and distributed_utils.get_rank() in [-1, 0]:
            print('Build ModelEMA ...')
            self.model_ema = ModelEMA(self.ema_dict, model, self.start_epoch * len(self.train_loader))
        else:
            self.model_ema = None

    # 训练模型的主函数
    def train(self, model):
        for epoch in range(self.start_epoch, self.args.max_epoch):
            if self.args.distributed:
                self.train_loader.batch_sampler.sampler.set_epoch(epoch)

            # 检查当前是否进入训练的第二阶段，在第二阶段中，会关闭Mosaic & Mixup增强
            if epoch >= (self.args.max_epoch - self.no_aug_epoch - 1) and not self.second_stage:
                self.check_second_stage()
                # 保存Mosaic增强阶段的最后一次的模型参数
                weight_name = '{}_last_mosaic_epoch.pth'.format(self.args.model)
                checkpoint_path = os.path.join(self.path_to_save, weight_name)
                if not os.path.exists(checkpoint_path):
                    print('Saving state of the last Mosaic epoch-{}.'.format(self.epoch + 1))
                    torch.save({'model': model.state_dict(),
                                'mAP': round(self.evaluator.map*100, 1),
                                'optimizer': self.optimizer.state_dict(),
                                'epoch': self.epoch,
                                'args': self.args}, 
                                checkpoint_path)                      

            # 训练模型一个epoch
            self.epoch = epoch
            self.train_one_epoch(model)

            # 在训练完毕后，会考虑是否要测试当前模型的性能
            # 如果heavy_eval为True，则每个epoch后都进行测试
            if self.heavy_eval:
                model_eval = model.module if self.args.distributed else model
                self.eval(model_eval)
            # 如果heavy_eval为False，则只在特定epoch后去测试性能
            else:
                model_eval = model.module if self.args.distributed else model
                if (epoch % self.args.eval_epoch) == 0 or (epoch == self.args.max_epoch - 1):
                    self.eval(model_eval)

    # 测试模型的主函数
    def eval(self, model):
        # 如果启动了EMA，则使用保存在EMA中的模型参数来进行测试
        # 否则，使用当前的模型参数进行测试
        model_eval = model if self.model_ema is None else self.model_ema.ema

        # 对于分布式训练，只在Rank0线程上进行测试
        if distributed_utils.is_main_process():
            # 如果Evaluator类为None，则只保存模型，不测试（无法测试）
            if self.evaluator is None:
                print('No evaluator ... save model and go on training.')
                print('Saving state, epoch: {}'.format(self.epoch + 1))
                weight_name = '{}_no_eval.pth'.format(self.args.model)
                checkpoint_path = os.path.join(self.path_to_save, weight_name)
                torch.save({'model': model_eval.state_dict(),
                            'mAP': -1.,
                            'optimizer': self.optimizer.state_dict(),
                            'epoch': self.epoch,
                            'args': self.args}, 
                            checkpoint_path)               
            # 如果Evaluator类不是None，则进行测试
            else:
                print('Evaluating model ...')
                # 将模型切换至torch要求的eval模式
                model_eval.eval()
                # 设置模型中的trainable为False，以便模型做前向推理（包括各种后处理）
                model_eval.trainable = False

                # 测试模型的性能
                with torch.no_grad():
                    self.evaluator.evaluate(model_eval)

                # 只有当前的性能指标大于上一次的指标，才会保存模型权重
                cur_map = self.evaluator.map
                if cur_map > self.best_map:
                    # update best-map
                    self.best_map = cur_map
                    # save model
                    print('Saving state, epoch:', self.epoch + 1)
                    weight_name = '{}_best.pth'.format(self.args.model)
                    checkpoint_path = os.path.join(self.path_to_save, weight_name)
                    torch.save({'model': model_eval.state_dict(),
                                'mAP': round(self.best_map*100, 1),
                                'optimizer': self.optimizer.state_dict(),
                                'epoch': self.epoch,
                                'args': self.args}, 
                                checkpoint_path)                      

                # 将模型切换至torch要求的train模式，以便继续训练
                model_eval.train()
                model_eval.trainable = True

        if self.args.distributed:
            # wait for all processes to synchronize
            dist.barrier()

    # 训练模型一个epoch的主函数
    def train_one_epoch(self, model):
        # 一些基础参数
        epoch_size = len(self.train_loader)
        img_size = self.args.img_size
        t0 = time.time()
        nw = epoch_size * self.args.wp_epoch
        accumulate = accumulate = max(1, round(64 / self.args.batch_size))

        # 训练模型一个epoch
        for iter_i, (images, targets) in enumerate(self.train_loader):
            ni = iter_i + self.epoch * epoch_size
            # Warmup阶段
            if ni <= nw:
                xi = [0, nw]  # x interp
                accumulate = max(1, np.interp(ni, xi, [1, 64 / self.args.batch_size]).round())
                for j, x in enumerate(self.optimizer.param_groups):
                    # 对于bias参数，其学习率从设定好的warmup_bias_lr降低至初始学习率lr0，
                    # 其他参数的学习率则从0增加至初始学习率lr0，
                    # 该策略参考YOLOv5 & v8项目
                    x['lr'] = np.interp(
                        ni, xi, [self.warmup_dict['warmup_bias_lr'] if j == 0 else 0.0, x['initial_lr'] * self.lf(self.epoch)])
                    # 在Warmup阶段，优化器的momentum参数也会从设定好的warmup_momentum增加至指定的momentum
                    if 'momentum' in x:
                        x['momentum'] = np.interp(ni, xi, [self.warmup_dict['warmup_momentum'], self.optimizer_dict['momentum']])
                                
            # 将数据放置在指定的device上，并做归一化
            images = images.to(self.device, non_blocking=True).float() / 255.

            # 多尺度训练技巧
            if self.args.multi_scale:
                images, targets, img_size = self.rescale_image_targets(
                    images, targets, self.model_cfg['stride'], self.args.min_box_size, self.model_cfg['multi_scale'])
            else:
                targets = self.refine_targets(targets, self.args.min_box_size)
                
            # 可视化训练阶段的数据和标签，以便查看数据是否有bug
            if self.args.vis_tgt:
                vis_data(images*255, targets)

            # 前向推理 & 计算损失
            with torch.cuda.amp.autocast(enabled=self.args.fp16):
                # 前向推理
                outputs = model(images)
                # 计算损失
                loss_dict = self.criterion(outputs=outputs, targets=targets, epoch=self.epoch)
                losses = loss_dict['losses']
                # 参考YOLOv5 & v8项目，损失前面要乘以batch size
                losses *= images.shape[0]  # loss * bs

                # 求多卡之间的平均loss，对于单卡，该函数没有作用
                loss_dict_reduced = distributed_utils.reduce_dict(loss_dict)

                # 参考YOLOv5 & v8项目，损失前面还要乘以分布式训练所用到的显卡数量，
                # 因为在Torch的默认设置下，梯度会在多卡之间做平均，
                # YOLOv5 & v8项目的一些技巧比较“非主流”，建议读者不要深究
                # 对于单卡，该函数没有作用
                losses *= distributed_utils.get_world_size()

            # 计算梯度
            self.scaler.scale(losses).backward()

            # 优化模型的参数
            if ni - self.last_opt_step >= accumulate:
                # 如有必要，做梯度剪裁，避免梯度爆炸
                if self.clip_grad > 0:
                    self.scaler.unscale_(self.optimizer)
                    torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=self.clip_grad)
                # 梯度反向传播，更新模型的参数
                self.scaler.step(self.optimizer)
                self.scaler.update()
                self.optimizer.zero_grad()
                # 如果使用了EMA，则在反向传播之后，更新EMA中的模型参数
                if self.model_ema is not None:
                    self.model_ema.update(model)
                # 记录本次更新权重的迭代位置
                self.last_opt_step = ni

            # 打印训练阶段的一些参数，以便读者在终端查看训练的各种输出
            if distributed_utils.is_main_process() and iter_i % 10 == 0:
                t1 = time.time()
                cur_lr = [param_group['lr']  for param_group in self.optimizer.param_groups]
                # 打印一些基本信息，如训练的epoch、iteration和学习率
                log =  '[Epoch: {}/{}]'.format(self.epoch+1, self.args.max_epoch)
                log += '[Iter: {}/{}]'.format(iter_i, epoch_size)
                log += '[lr: {:.6f}]'.format(cur_lr[2])
                # 打印模型的loss
                for k in loss_dict_reduced.keys():
                    log += '[{}: {:.2f}]'.format(k, loss_dict_reduced[k])
                # 打印一些其他信息，比如当前迭代的耗时和图像尺寸
                log += '[time: {:.2f}]'.format(t1 - t0)
                log += '[size: {}]'.format(img_size)

                print(log, flush=True)
                
                t0 = time.time()
        
        # 学习率更新
        self.lr_scheduler.step()
        
    # 训练的第二阶段
    def check_second_stage(self):
        # 在第二阶段，关闭Mosaic和Mixup两个强大的数据增强
        # 使得模型最后在正常的数据上完成最终的收敛
        print('============== Second stage of Training ==============')
        self.second_stage = True

        # 关闭 mosaic augmentation
        if self.train_loader.dataset.mosaic_prob > 0.:
            print(' - Close < Mosaic Augmentation > ...')
            self.train_loader.dataset.mosaic_prob = 0.
            self.heavy_eval = True

        # 关闭 mixup augmentation
        if self.train_loader.dataset.mixup_prob > 0.:
            print(' - Close < Mixup Augmentation > ...')
            self.train_loader.dataset.mixup_prob = 0.
            self.heavy_eval = True

        # 如果使用到了旋转相关的数据增强，也要将其关闭，因为旋转增强会引入一些不准确的bbox坐标
        if 'degrees' in self.trans_cfg.keys() and self.trans_cfg['degrees'] > 0.0:
            print(' - Close < degress of rotation > ...')
            self.trans_cfg['degrees'] = 0.0
        if 'shear' in self.trans_cfg.keys() and self.trans_cfg['shear'] > 0.0:
            print(' - Close < shear of rotation >...')
            self.trans_cfg['shear'] = 0.0
        if 'perspective' in self.trans_cfg.keys() and self.trans_cfg['perspective'] > 0.0:
            print(' - Close < perspective of rotation > ...')
            self.trans_cfg['perspective'] = 0.0

        # 如果使用到了平移和缩放，也将其关闭
        if 'translate' in self.trans_cfg.keys() and self.trans_cfg['translate'] > 0.0:
            print(' - Close < translate of affine > ...')
            self.trans_cfg['translate'] = 0.0
        if 'scale' in self.trans_cfg.keys():
            print(' - Close < scale of affine >...')
            self.trans_cfg['scale'] = [1.0, 1.0]

        # 修改第二阶段的Transform类
        print(' - Rebuild transforms ...')
        self.train_transform, self.trans_cfg = build_transform(
            args=self.args, trans_config=self.trans_cfg, max_stride=self.model_cfg['max_stride'], is_train=True)
        self.train_loader.dataset.transform = self.train_transform
        
    # 清洗训练阶段的数据，滤除无效的bbox标签
    def refine_targets(self, targets, min_box_size):
        # rescale targets
        for tgt in targets:
            boxes = tgt["boxes"].clone()
            labels = tgt["labels"].clone()
            # refine tgt
            tgt_boxes_wh = boxes[..., 2:] - boxes[..., :2]
            min_tgt_size = torch.min(tgt_boxes_wh, dim=-1)[0]
            keep = (min_tgt_size >= min_box_size)

            tgt["boxes"] = boxes[keep]
            tgt["labels"] = labels[keep]
        
        return targets

    # 调整训练的图像尺寸和相应的bbox坐标，服务于多尺度训练技巧
    def rescale_image_targets(self, images, targets, stride, min_box_size, multi_scale_range=[0.5, 1.5]):
        """
            Deployed for Multi scale trick.
        """
        if isinstance(stride, int):
            max_stride = stride
        elif isinstance(stride, list):
            max_stride = max(stride)

        # 随机选择一个新的图像尺寸
        old_img_size = images.shape[-1]
        new_img_size = random.randrange(old_img_size * multi_scale_range[0], old_img_size * multi_scale_range[1] + max_stride)
        new_img_size = new_img_size // max_stride * max_stride  # size
        
        # 如果新的图像尺寸不等于当前的图像尺寸，则将其调整为新的图像尺寸
        # 注意，这里我们一次性将一批数据中的所有图像都调整为新的尺寸
        if new_img_size / old_img_size != 1:
            images = torch.nn.functional.interpolate(input = images, 
                                                     size  = new_img_size, 
                                                     mode  = 'bilinear', 
                                                     align_corners = False)
        # 依据新的图像尺寸，调整bbox坐标
        for tgt in targets:
            boxes = tgt["boxes"].clone()
            labels = tgt["labels"].clone()
            boxes = torch.clamp(boxes, 0, old_img_size)
            # rescale box
            boxes[:, [0, 2]] = boxes[:, [0, 2]] / old_img_size * new_img_size
            boxes[:, [1, 3]] = boxes[:, [1, 3]] / old_img_size * new_img_size
            # refine tgt
            tgt_boxes_wh = boxes[..., 2:] - boxes[..., :2]
            min_tgt_size = torch.min(tgt_boxes_wh, dim=-1)[0]
            keep = (min_tgt_size >= min_box_size)

            tgt["boxes"] = boxes[keep]
            tgt["labels"] = labels[keep]

        return images, targets, new_img_size


# YOLOX Trainer: 主要用于训练YOLOX和本项目实现的较为简单的YOLOv7
class YoloxTrainer(object):
    def __init__(self, args, data_cfg, model_cfg, trans_cfg, device, model, criterion, world_size):
        # ------------------- 基础参数 -------------------
        self.args               = args
        self.epoch              = 0
        self.best_map           = -1.
        self.device             = device
        self.criterion          = criterion
        self.world_size         = world_size
        self.grad_accumulate    = args.grad_accumulate
        self.no_aug_epoch       = args.no_aug_epoch
        self.heavy_eval         = False
        self.second_stage       = False
        self.third_stage        = False
        self.second_stage_epoch = args.no_aug_epoch
        self.third_stage_epoch  = args.no_aug_epoch // 2

        # 创建路径，用于保存模型的训练文件
        self.path_to_save = os.path.join(args.save_folder, args.dataset, args.model)
        os.makedirs(self.path_to_save, exist_ok=True)

        # ---------------------------- YOLOX项目的超参数设置 ----------------------------
        self.optimizer_dict = {'optimizer': 'sgd',    # 使用SGD优化器; 若使用AdamW，则修改为'adamw'，但建议使用SGD优化器
                               'momentum': 0.9,       # SGD所需的momentum，AdamW优化器不需要此参数
                               'weight_decay': 5e-4,  # SGD所需的weight decay; 若使用AdamW，则修改为5e-2
                               'lr0': 0.01,           # 初始学习率; 若使用AdamW，则修改为0.001
                               }
        ## 学习策略的超参数
        self.lr_schedule_dict = {'scheduler': 'cosine', # 使用YOLOX官方的Cosine衰减策略
                                 'lrf': 0.05,           # 最终学习率与初始学习率的比值，即最终学习率=lr0 * lrf
                                 }
        ## 训练的Warmup阶段的超参数
        self.warmup_dict = {'warmup_momentum': 0.8,  # 使用YOLOv5&v8官方的超参数设定，建议保持默认设置
                            'warmup_bias_lr': 0.1,   # 使用YOLOv5&v8官方的超参数设定，建议保持默认设置
                            }        
        ## EMA技巧的超参数，建议保持默认设置
        self.ema_dict = {'ema_decay': 0.9999, 'ema_tau': 2000}

        # ---------------------------- 构建Dataset、Model和Transforms所需的config变量 ----------------------------
        ## 数据集的config
        self.data_cfg = data_cfg
        ## 模型的config
        self.model_cfg = model_cfg
        ## 数据预处理的config
        self.trans_cfg = trans_cfg

        # ---------------------------- 构建数据预处理 Transform类 ----------------------------
        ## 构建训练(Train)所需的数据预处理
        self.train_transform, self.trans_cfg = build_transform(
            args=self.args, trans_config=self.trans_cfg, max_stride=self.model_cfg['max_stride'], is_train=True)
        ## 构建测试(Evaluate)所需的数据预处理
        self.val_transform, _ = build_transform(
            args=self.args, trans_config=self.trans_cfg, max_stride=self.model_cfg['max_stride'], is_train=False)

        # ---------------------------- 构建Dataset & Dataloader ----------------------------
        ## 构建Dataset，用于读取数据集的图像和标签
        self.dataset, self.dataset_info = build_dataset(self.args, self.data_cfg, self.trans_cfg, self.train_transform, is_train=True)
        ## 构建Dataloader，用于后续的训练
        self.train_loader = build_dataloader(self.args, self.dataset, self.args.batch_size // self.world_size, CollateFunc())

        # ---------------------------- 构建测试模型性能的Evaluator类 ----------------------------
        self.evaluator = build_evluator(self.args, self.data_cfg, self.val_transform, self.device)

        # ---------------------------- 构建梯度缩放器 ----------------------------
        self.scaler = torch.cuda.amp.GradScaler(enabled=self.args.fp16)

        # ---------------------------- 构建优化器 ----------------------------
        ## 构建YOLO项目的优化器
        self.optimizer_dict['lr0'] *= self.args.batch_size * self.grad_accumulate / 64
        self.optimizer, self.start_epoch = build_yolo_optimizer(self.optimizer_dict, model, self.args.resume)

        # ---------------------------- 构建学习率策略 ----------------------------
        ## 构建YOLO项目的学习率策略
        self.lr_scheduler, self.lf = build_lr_scheduler(self.lr_schedule_dict, self.optimizer, self.args.max_epoch - self.no_aug_epoch)
        self.lr_scheduler.last_epoch = self.start_epoch - 1  # do not move
        if self.args.resume:
            self.lr_scheduler.step()

        # ---------------------------- 构建 Model-EMA ----------------------------
        if self.args.ema and distributed_utils.get_rank() in [-1, 0]:
            print('Build ModelEMA ...')
            self.model_ema = ModelEMA(self.ema_dict, model, self.start_epoch * len(self.train_loader))
        else:
            self.model_ema = None

    # 训练模型的主函数
    def train(self, model):
        for epoch in range(self.start_epoch, self.args.max_epoch):
            if self.args.distributed:
                self.train_loader.batch_sampler.sampler.set_epoch(epoch)

            # 检查当前是否进入训练的第二阶段，在第二阶段中，会关闭Mosaic & Mixup增强
            if epoch >= (self.args.max_epoch - self.second_stage_epoch - 1) and not self.second_stage:
                self.check_second_stage()
                # 保存第一阶段的最后一次的模型参数
                weight_name = '{}_last_mosaic_epoch.pth'.format(self.args.model)
                checkpoint_path = os.path.join(self.path_to_save, weight_name)
                print('Saving state of the last Mosaic epoch-{}.'.format(self.epoch + 1))
                torch.save({'model': model.state_dict(),
                            'mAP': round(self.evaluator.map*100, 1),
                            'optimizer': self.optimizer.state_dict(),
                            'epoch': self.epoch,
                            'args': self.args}, 
                            checkpoint_path)

            # 检查当前是否进入训练的第三阶段，在第三阶段中，会所有的数据增强
            if epoch >= (self.args.max_epoch - self.third_stage_epoch - 1) and not self.third_stage:
                self.check_third_stage()
                # 保存第二阶段的最后一次的模型参数
                weight_name = '{}_last_weak_augment_epoch.pth'.format(self.args.model)
                checkpoint_path = os.path.join(self.path_to_save, weight_name)
                print('Saving state of the last weak augment epoch-{}.'.format(self.epoch + 1))
                torch.save({'model': model.state_dict(),
                            'mAP': round(self.evaluator.map*100, 1),
                            'optimizer': self.optimizer.state_dict(),
                            'epoch': self.epoch,
                            'args': self.args}, 
                            checkpoint_path)
                
            # 训练模型一个epoch
            self.epoch = epoch
            self.train_one_epoch(model)

            # 在训练完毕后，会考虑是否要测试当前模型的性能
            # 如果heavy_eval为True，则每个epoch后都进行测试
            if self.heavy_eval:
                model_eval = model.module if self.args.distributed else model
                self.eval(model_eval)
            # 如果heavy_eval为False，则只在特定epoch后去测试性能
            else:
                model_eval = model.module if self.args.distributed else model
                if (epoch % self.args.eval_epoch) == 0 or (epoch == self.args.max_epoch - 1):
                    self.eval(model_eval)

    # 测试模型的主函数
    def eval(self, model):
        # 如果启动了EMA，则使用保存在EMA中的模型参数来进行测试
        # 否则，使用当前的模型参数进行测试
        model_eval = model if self.model_ema is None else self.model_ema.ema

        # 对于分布式训练，只在Rank0线程上进行测试
        if distributed_utils.is_main_process():
            # 如果Evaluator类为None，则只保存模型，不测试（无法测试）
            if self.evaluator is None:
                print('No evaluator ... save model and go on training.')
                print('Saving state, epoch: {}'.format(self.epoch + 1))
                weight_name = '{}_no_eval.pth'.format(self.args.model)
                checkpoint_path = os.path.join(self.path_to_save, weight_name)
                torch.save({'model': model_eval.state_dict(),
                            'mAP': -1.,
                            'optimizer': self.optimizer.state_dict(),
                            'epoch': self.epoch,
                            'args': self.args}, 
                            checkpoint_path)               
            # 如果Evaluator类不是None，则进行测试
            else:
                print('Evaluating model ...')
                # 将模型切换至torch要求的eval模式
                model_eval.eval()
                # 设置模型中的trainable为False，以便模型做前向推理（包括各种后处理）
                model_eval.trainable = False

                # 测试模型的性能
                with torch.no_grad():
                    self.evaluator.evaluate(model_eval)

                # 只有当前的性能指标大于上一次的指标，才会保存模型权重
                cur_map = self.evaluator.map
                if cur_map > self.best_map:
                    # update best-map
                    self.best_map = cur_map
                    # save model
                    print('Saving state, epoch:', self.epoch + 1)
                    weight_name = '{}_best.pth'.format(self.args.model)
                    checkpoint_path = os.path.join(self.path_to_save, weight_name)
                    torch.save({'model': model_eval.state_dict(),
                                'mAP': round(self.best_map*100, 1),
                                'optimizer': self.optimizer.state_dict(),
                                'epoch': self.epoch,
                                'args': self.args}, 
                                checkpoint_path)                      

                # 将模型切换至torch要求的train模式，以便继续训练
                model_eval.train()
                model_eval.trainable = True

        if self.args.distributed:
            # wait for all processes to synchronize
            dist.barrier()

    # 训练模型一个epoch的主函数
    def train_one_epoch(self, model):
        # 一些基础参数
        epoch_size = len(self.train_loader)
        img_size = self.args.img_size
        t0 = time.time()
        nw = epoch_size * self.args.wp_epoch

        # 训练模型一个epoch
        for iter_i, (images, targets) in enumerate(self.train_loader):
            ni = iter_i + self.epoch * epoch_size
            # Warmup阶段
            if ni <= nw:
                xi = [0, nw]  # x interp
                for j, x in enumerate(self.optimizer.param_groups):
                    # 对于bias参数，其学习率从设定好的warmup_bias_lr降低至初始学习率lr0，
                    # 其他参数的学习率则从0增加至初始学习率lr0，
                    # 该策略参考YOLOv5 & v8项目
                    x['lr'] = np.interp(
                        ni, xi, [self.warmup_dict['warmup_bias_lr'] if j == 0 else 0.0, x['initial_lr'] * self.lf(self.epoch)])
                    # 在Warmup阶段，优化器的momentum参数也会从设定好的warmup_momentum增加至指定的momentum
                    if 'momentum' in x:
                        x['momentum'] = np.interp(ni, xi, [self.warmup_dict['warmup_momentum'], self.optimizer_dict['momentum']])
                                
            # 将数据放置在指定的device上，并做归一化
            images = images.to(self.device, non_blocking=True).float() / 255.

            # 多尺度训练技巧
            if self.args.multi_scale and ni % 10 == 0:
                images, targets, img_size = self.rescale_image_targets(
                    images, targets, self.model_cfg['stride'], self.args.min_box_size, self.model_cfg['multi_scale'])
            else:
                targets = self.refine_targets(targets, self.args.min_box_size)
                
            # 可视化训练阶段的数据和标签，以便查看数据是否有bug
            if self.args.vis_tgt:
                vis_data(images*255, targets)

            # 前向推理 & 计算损失
            with torch.cuda.amp.autocast(enabled=self.args.fp16):
                # 前向推理
                outputs = model(images)
                # 计算损失
                loss_dict = self.criterion(outputs=outputs, targets=targets, epoch=self.epoch)
                losses = loss_dict['losses']
                # 如果设置了梯度累加次数，则对loss进行缩放
                if self.grad_accumulate > 1: 
                    losses /= self.grad_accumulate

                # 求多卡之间的平均loss，对于单卡，该函数没有作用
                loss_dict_reduced = distributed_utils.reduce_dict(loss_dict)

            # 计算梯度
            self.scaler.scale(losses).backward()

            # 优化模型的参数
            if ni % self.grad_accumulate == 0:
                self.scaler.step(self.optimizer)
                self.scaler.update()
                self.optimizer.zero_grad()
                # 如果使用了EMA，则在反向传播之后，更新EMA中的模型参数
                if self.model_ema is not None:
                    self.model_ema.update(model)

            # 打印训练阶段的一些参数，以便读者在终端查看训练的各种输出
            if distributed_utils.is_main_process() and iter_i % 10 == 0:
                t1 = time.time()
                cur_lr = [param_group['lr']  for param_group in self.optimizer.param_groups]
                # 打印一些基本信息，如训练的epoch、iteration和学习率
                log =  '[Epoch: {}/{}]'.format(self.epoch+1, self.args.max_epoch)
                log += '[Iter: {}/{}]'.format(iter_i, epoch_size)
                log += '[lr: {:.6f}]'.format(cur_lr[2])
                # 打印模型的loss
                for k in loss_dict_reduced.keys():
                    loss_val = loss_dict_reduced[k]
                    if k == 'losses':
                        loss_val *= self.grad_accumulate
                    log += '[{}: {:.2f}]'.format(k, loss_val)
                # 打印一些其他信息，比如当前迭代的耗时和图像尺寸
                log += '[time: {:.2f}]'.format(t1 - t0)
                log += '[size: {}]'.format(img_size)

                print(log, flush=True)
                
                t0 = time.time()
        
        # 学习率更新
        if not self.second_stage:
            self.lr_scheduler.step()
        
    # 训练的第二阶段
    def check_second_stage(self):
        # 在第二阶段，关闭Mosaic和Mixup两个强大的数据增强
        # 在YOLOX官方中，平移、缩放和剪裁等弱增强在第二阶段也会被关闭
        # 笔者实现的第二阶段会继续使用平移、缩放和剪裁等弱增强
        # 因此，笔者实现的第二阶段要略不同于官方的YOLOX
        print('============== Second stage of Training ==============')
        self.second_stage = True

        # 关闭 mosaic augmentation
        if self.train_loader.dataset.mosaic_prob > 0.:
            print(' - Close < Mosaic Augmentation > ...')
            self.train_loader.dataset.mosaic_prob = 0.
            self.heavy_eval = True

        # 关闭 mixup augmentation
        if self.train_loader.dataset.mixup_prob > 0.:
            print(' - Close < Mixup Augmentation > ...')
            self.train_loader.dataset.mixup_prob = 0.
            self.heavy_eval = True

        # 如果使用到了旋转相关的数据增强，也要将其关闭，因为旋转增强会引入一些不准确的bbox坐标
        if 'degrees' in self.trans_cfg.keys() and self.trans_cfg['degrees'] > 0.0:
            print(' - Close < degress of rotation > ...')
            self.trans_cfg['degrees'] = 0.0
        if 'shear' in self.trans_cfg.keys() and self.trans_cfg['shear'] > 0.0:
            print(' - Close < shear of rotation >...')
            self.trans_cfg['shear'] = 0.0
        if 'perspective' in self.trans_cfg.keys() and self.trans_cfg['perspective'] > 0.0:
            print(' - Close < perspective of rotation > ...')
            self.trans_cfg['perspective'] = 0.0

        # 修改第二阶段的Transform类
        print(' - Rebuild transforms ...')
        self.train_transform, self.trans_cfg = build_transform(
            args=self.args, trans_config=self.trans_cfg, max_stride=self.model_cfg['max_stride'], is_train=True)
        self.train_loader.dataset.transform = self.train_transform
        
    # 训练的第三阶段
    def check_third_stage(self):
        # 在第二三阶段，平移、缩放和剪裁等弱增强也会被关闭
        # 使得模型最后在正常的数据上完成最终的收敛
        print('============== Third stage of Training ==============')
        self.third_stage = True

        # close random affine
        if 'translate' in self.trans_cfg.keys() and self.trans_cfg['translate'] > 0.0:
            print(' - Close < translate of affine > ...')
            self.trans_cfg['translate'] = 0.0
        if 'scale' in self.trans_cfg.keys():
            print(' - Close < scale of affine >...')
            self.trans_cfg['scale'] = [1.0, 1.0]

        # build a new transform for second stage
        print(' - Rebuild transforms ...')
        self.train_transform, self.trans_cfg = build_transform(
            args=self.args, trans_config=self.trans_cfg, max_stride=self.model_cfg['max_stride'], is_train=True)
        self.train_loader.dataset.transform = self.train_transform
        
    # 清洗训练阶段的数据，滤除无效的bbox标签
    def refine_targets(self, targets, min_box_size):
        # rescale targets
        for tgt in targets:
            boxes = tgt["boxes"].clone()
            labels = tgt["labels"].clone()
            # refine tgt
            tgt_boxes_wh = boxes[..., 2:] - boxes[..., :2]
            min_tgt_size = torch.min(tgt_boxes_wh, dim=-1)[0]
            keep = (min_tgt_size >= min_box_size)

            tgt["boxes"] = boxes[keep]
            tgt["labels"] = labels[keep]
        
        return targets

    # 调整训练的图像尺寸和相应的bbox坐标，服务于多尺度训练技巧
    def rescale_image_targets(self, images, targets, stride, min_box_size, multi_scale_range=[0.5, 1.5]):
        """
            Deployed for Multi scale trick.
        """
        if isinstance(stride, int):
            max_stride = stride
        elif isinstance(stride, list):
            max_stride = max(stride)

        # 随机选择一个新的图像尺寸
        old_img_size = images.shape[-1]
        new_img_size = random.randrange(old_img_size * multi_scale_range[0], old_img_size * multi_scale_range[1] + max_stride)
        new_img_size = new_img_size // max_stride * max_stride  # size

        # 如果新的图像尺寸不等于当前的图像尺寸，则将其调整为新的图像尺寸
        # 注意，这里我们一次性将一批数据中的所有图像都调整为新的尺寸
        if new_img_size / old_img_size != 1:
            images = torch.nn.functional.interpolate(input = images, 
                                                     size  = new_img_size, 
                                                     mode  = 'bilinear', 
                                                     align_corners = False)
        # 依据新的图像尺寸，调整bbox坐标
        for tgt in targets:
            boxes = tgt["boxes"].clone()
            labels = tgt["labels"].clone()
            boxes = torch.clamp(boxes, 0, old_img_size)
            # rescale box
            boxes[:, [0, 2]] = boxes[:, [0, 2]] / old_img_size * new_img_size
            boxes[:, [1, 3]] = boxes[:, [1, 3]] / old_img_size * new_img_size
            # refine tgt
            tgt_boxes_wh = boxes[..., 2:] - boxes[..., :2]
            min_tgt_size = torch.min(tgt_boxes_wh, dim=-1)[0]
            keep = (min_tgt_size >= min_box_size)

            tgt["boxes"] = boxes[keep]
            tgt["labels"] = labels[keep]

        return images, targets, new_img_size


# Build Trainer
def build_trainer(args, data_cfg, model_cfg, trans_cfg, device, model, criterion, world_size):
    if model_cfg['trainer_type'] == 'yolov8':
        return Yolov8Trainer(args, data_cfg, model_cfg, trans_cfg, device, model, criterion, world_size)
    elif model_cfg['trainer_type'] == 'yolox':
        return YoloxTrainer(args, data_cfg, model_cfg, trans_cfg, device, model, criterion, world_size)
    else:
        raise NotImplementedError
    