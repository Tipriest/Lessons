from __future__ import division

import argparse
from copy import deepcopy

# ----------------- Torch Components -----------------
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP

# ----------------- Extra Components -----------------
from utils import distributed_utils
from utils.misc import compute_flops

# ----------------- Config Components -----------------
from config import build_dataset_config, build_model_config, build_trans_config

# ----------------- Model Components -----------------
from models import build_model

# ----------------- Train Components -----------------
from engine import build_trainer


def parse_args():
    parser = argparse.ArgumentParser(description='YOLO-Tutorial')
    # Basic
    parser.add_argument('--cuda', action='store_true', default=False,
                        help='use cuda.')
    parser.add_argument('-size', '--img_size', default=640, type=int, 
                        help='input image size')
    parser.add_argument('--num_workers', default=4, type=int, 
                        help='Number of workers used in dataloading')
    parser.add_argument('--tfboard', action='store_true', default=False,
                        help='use tensorboard')
    parser.add_argument('--save_folder', default='weights/', type=str, 
                        help='path to save weight')
    parser.add_argument('--eval_first', action='store_true', default=False,
                        help='evaluate model before training.')
    parser.add_argument('--fp16', dest="fp16", action="store_true", default=False,
                        help="Adopting mix precision training.")
    parser.add_argument('--vis_tgt', action="store_true", default=False,
                        help="visualize training data.")
    parser.add_argument('--vis_aux_loss', action="store_true", default=False,
                        help="visualize aux loss.")
    
    # Batchsize
    parser.add_argument('-bs', '--batch_size', default=32, type=int, 
                        help='batch size on all the GPUs.')

    # Epoch
    parser.add_argument('--max_epoch', default=150, type=int, 
                        help='max epoch.')
    parser.add_argument('--wp_epoch', default=1, type=int, 
                        help='warmup epoch.')
    parser.add_argument('--eval_epoch', default=10, type=int, 
                        help='after eval epoch, the model is evaluated on val dataset.')
    parser.add_argument('--no_aug_epoch', default=20, type=int, 
                        help='cancel strong augmentation.')

    # Model
    parser.add_argument('-m', '--model', default='yolov1', type=str,
                        help='build yolo')
    parser.add_argument('-ct', '--conf_thresh', default=0.005, type=float,
                        help='confidence threshold')
    parser.add_argument('-nt', '--nms_thresh', default=0.6, type=float,
                        help='NMS threshold')
    parser.add_argument('--topk', default=1000, type=int,
                        help='topk candidates for evaluation')
    parser.add_argument('-p', '--pretrained', default=None, type=str,
                        help='load pretrained weight')
    parser.add_argument('-r', '--resume', default=None, type=str,
                        help='keep training')

    # Dataset
    parser.add_argument('--root', default='/home/tipriest/data/',
                        help='data root')
    parser.add_argument('-d', '--dataset', default='coco',
                        help='coco, voc, widerface, crowdhuman')
    parser.add_argument('--load_cache', action='store_true', default=False,
                        help='load data into memory.')
    
    # Train trick
    parser.add_argument('-ms', '--multi_scale', action='store_true', default=False,
                        help='Multi scale')
    parser.add_argument('--ema', action='store_true', default=False,
                        help='Model EMA')
    parser.add_argument('--min_box_size', default=8.0, type=float,
                        help='min size of target bounding box.')
    parser.add_argument('--mosaic', default=None, type=float,
                        help='mosaic augmentation.')
    parser.add_argument('--mixup', default=None, type=float,
                        help='mixup augmentation.')
    parser.add_argument('--grad_accumulate', default=1, type=int,
                        help='gradient accumulation')

    # DDP train
    parser.add_argument('-dist', '--distributed', action='store_true', default=False,
                        help='distributed training')
    parser.add_argument('--dist_url', default='env://', 
                        help='url used to set up distributed training')
    parser.add_argument('--world_size', default=1, type=int,
                        help='number of distributed processes')
    parser.add_argument('--sybn', action='store_true', default=False, 
                        help='use sybn.')

    return parser.parse_args()


def train():
    args = parse_args()
    print("Setting Arguments.. : ", args)
    print("----------------------------------------------------------")

    # 如果args.distributed为True，则初始化PyTorch框架提供的分布式训练（DDP）
    if args.distributed:
        distributed_utils.init_distributed_mode(args)
        print("git:\n  {}\n".format(distributed_utils.get_sha()))
    # 对于单卡，world_size = 1； 对于多卡，world_size = 卡的数量
    world_size = distributed_utils.get_world_size()
    print('World size: {}'.format(world_size))

    # 如果args.cuda为True，则使用GPU来训练，否则使用CPU来训练（强烈不推荐）
    if args.cuda:
        print('use GPU to train')
        device = torch.device("cuda")
    else:
        print('use CPU to train')
        device = torch.device("cpu")

    # 构建训练所用到的 Dataset & Model & Transform相关的config变量
    data_cfg = build_dataset_config(args)
    model_cfg = build_model_config(args)
    trans_cfg = build_trans_config(model_cfg['trans_type'])

    # 构建YOLO模型
    model, criterion = build_model(args, model_cfg, device, data_cfg['num_classes'], True)

    # 如果指定了args.resume，则表明我们要从之前停止的迭代节点继续训练模型
    if distributed_utils.is_main_process and args.resume is not None:
        print('keep training: ', args.resume)
        checkpoint = torch.load(args.resume, map_location='cpu')
        # checkpoint state dict
        checkpoint_state_dict = checkpoint.pop("model")
        model.load_state_dict(checkpoint_state_dict)

    # 将模型切换至train模式
    model = model.to(device).train()

    # 标记单卡模式的model，方便我们做一些其他的处理，省去了DDP模式下的model.module的判断
    model_without_ddp = model

    # 如果args.distributed为True，且args.sybn也为True，表明我们使用SyncBatchNorm层，同步多卡之间的BN统计量
    # 只有在DDP模式下才会考虑SyncBatchNorm层
    if args.sybn and args.distributed:
        print('use SyncBatchNorm ...')
        model = torch.nn.SyncBatchNorm.convert_sync_batchnorm(model)
    if args.distributed:
        model = DDP(model, device_ids=[args.gpu])
        model_without_ddp = model.module

    # 计算模型的参数量和FLOPs
    if distributed_utils.is_main_process:
        model_copy = deepcopy(model_without_ddp)
        model_copy.trainable = False
        model_copy.eval()
        compute_flops(model=model_copy,
                      img_size=args.img_size,
                      device=device)
        del model_copy
    if args.distributed:
        dist.barrier()

    # 构建训练所需的Trainer类
    trainer = build_trainer(args, data_cfg, model_cfg, trans_cfg, device, model_without_ddp, criterion, world_size)

    # --------------------------------- Train: Start ---------------------------------
    ## 如果args.eval_first为True，则在训练开始前，先测试模型的性能
    if args.eval_first and distributed_utils.is_main_process():
        # to check whether the evaluator can work
        model_eval = model_without_ddp
        trainer.eval(model_eval)

    ## 开始训练我们的模型
    trainer.train(model)
    # --------------------------------- Train: End ---------------------------------

    # 训练完毕后，清空占用的GPU显存
    del trainer
    if args.cuda:
        torch.cuda.empty_cache()


if __name__ == '__main__':
    train()
