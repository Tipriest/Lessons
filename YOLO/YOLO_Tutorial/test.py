import argparse
import cv2
import os
import time
import numpy as np
from copy import deepcopy
import torch

# load transform
from dataset.build import build_dataset, build_transform

# load some utils
from utils.misc import load_weight, compute_flops
from utils.box_ops import rescale_bboxes

from config import build_dataset_config, build_model_config, build_trans_config
from models import build_model


def parse_args():
    parser = argparse.ArgumentParser(description='YOLO-Tutorial')

    # basic
    parser.add_argument('-size', '--img_size', default=640, type=int,
                        help='the max size of input image')
    parser.add_argument('--show', action='store_true', default=False,
                        help='show the visulization results.')
    parser.add_argument('--save', action='store_true', default=False,
                        help='save the visulization results.')
    parser.add_argument('--cuda', action='store_true', default=False, 
                        help='use cuda.')
    parser.add_argument('--save_folder', default='det_results/', type=str,
                        help='Dir to save results')
    parser.add_argument('-vt', '--visual_threshold', default=0.4, type=float,
                        help='Final confidence threshold')
    parser.add_argument('-ws', '--window_scale', default=1.0, type=float,
                        help='resize window of cv2 for visualization.')
    parser.add_argument('--resave', action='store_true', default=False, 
                        help='resave checkpoints without optimizer state dict.')

    # model
    parser.add_argument('-m', '--model', default='yolov1', type=str,
                        help='build yolo')
    parser.add_argument('--weight', default=None,
                        type=str, help='Trained state_dict file path to open')
    parser.add_argument('-ct', '--conf_thresh', default=0.1, type=float,
                        help='confidence threshold')
    parser.add_argument('-nt', '--nms_thresh', default=0.5, type=float,
                        help='NMS threshold')
    parser.add_argument('--topk', default=100, type=int,
                        help='topk candidates for testing')
    parser.add_argument("--no_decode", action="store_true", default=False,
                        help="not decode in inference or yes")
    parser.add_argument('--fuse_conv_bn', action='store_true', default=False,
                        help='fuse Conv & BN')

    # dataset
    parser.add_argument('--root', default='/mnt/share/ssd2/dataset',
                        help='data root')
    parser.add_argument('-d', '--dataset', default='coco',
                        help='coco, voc.')
    parser.add_argument('--min_box_size', default=8.0, type=float,
                        help='min size of target bounding box.')
    parser.add_argument('--mosaic', default=None, type=float,
                        help='mosaic augmentation.')
    parser.add_argument('--mixup', default=None, type=float,
                        help='mixup augmentation.')
    parser.add_argument('--load_cache', action='store_true', default=False,
                        help='load data into memory.')

    return parser.parse_args()


# 绘制单个的bbox
def plot_bbox_labels(img, bbox, label=None, cls_color=None, text_scale=0.4):
    x1, y1, x2, y2 = bbox
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    t_size = cv2.getTextSize(label, 0, fontScale=1, thickness=2)[0]
    # 绘制bbox
    cv2.rectangle(img, (x1, y1), (x2, y2), cls_color, 2)
    
    # 在bbox上添加类别标签
    if label is not None:
        # plot title bbox
        cv2.rectangle(img, (x1, y1-t_size[1]), (int(x1 + t_size[0] * text_scale), y1), cls_color, -1)
        # put the test on the title bbox
        cv2.putText(img, label, (int(x1), int(y1 - 5)), 0, text_scale, (0, 0, 0), 1, lineType=cv2.LINE_AA)

    return img

# 可视化单张图片中的所有bbox
def visualize(img, 
              bboxes, 
              scores, 
              labels, 
              vis_thresh, 
              class_colors, 
              class_names, 
              class_indexs=None, 
              dataset_name='voc'):
    ts = 0.4
    for i, bbox in enumerate(bboxes):
        if scores[i] > vis_thresh:
            cls_id = int(labels[i])
            if dataset_name == 'coco':
                cls_color = class_colors[cls_id]
                cls_id = class_indexs[cls_id]
            else:
                cls_color = class_colors[cls_id]
                
            mess = '%s: %.2f' % (class_names[cls_id], scores[i])
            img = plot_bbox_labels(img, bbox, mess, cls_color, text_scale=ts)

    return img
        
# 测试函数
@torch.no_grad()
def test(args,
         model, 
         device, 
         dataset,
         transform=None,
         class_colors=None, 
         class_names=None, 
         class_indexs=None):
    num_images = len(dataset)
    save_path = os.path.join('det_results/', args.dataset, args.model)
    os.makedirs(save_path, exist_ok=True)

    for index in range(num_images):
        print('Testing image {:d}/{:d}....'.format(index+1, num_images))
        image, _ = dataset.pull_image(index)

        orig_h, orig_w, _ = image.shape

        # 数据预处理
        x, _, deltas = transform(image)
        x = x.unsqueeze(0).to(device) / 255.

        # 记录前向推理的耗时，以便计算FPS，默认时间单位为“秒(s)”
        t0 = time.time()
        # 模型前向推理，包括后处理等步骤
        bboxes, scores, labels = model(x)
        # 计算前向推理的耗时
        print("detection time used ", time.time() - t0, "s")
        
        # 依据原始图像的尺寸，调整预测bbox的坐标
        origin_img_size = [orig_h, orig_w]
        cur_img_size = [*x.shape[-2:]]
        bboxes = rescale_bboxes(bboxes, origin_img_size, cur_img_size, deltas)

        # 绘制检测结果
        img_processed = visualize(
                            img=image,
                            bboxes=bboxes,
                            scores=scores,
                            labels=labels,
                            vis_thresh=args.visual_threshold,
                            class_colors=class_colors,
                            class_names=class_names,
                            class_indexs=class_indexs,
                            dataset_name=args.dataset)
        
        # 如果args.show为True，则可视化上面绘制的检测结果
        if args.show:
            h, w = img_processed.shape[:2]
            sw, sh = int(w*args.window_scale), int(h*args.window_scale)
            cv2.namedWindow('detection', 0)
            cv2.resizeWindow('detection', sw, sh)
            cv2.imshow('detection', img_processed)
            cv2.waitKey(0)

        # 如果args.save为True，则保存上面绘制的检测结果
        if args.save:
            # save result
            cv2.imwrite(os.path.join(save_path, str(index).zfill(6) +'.jpg'), img_processed)


if __name__ == '__main__':
    args = parse_args()
    # 如果args.cuda为True，则使用GPU来推理，否则使用CPU来训练（可接受）
    if args.cuda:
        print('use cuda')
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")

    # 构建测试所用到的 Dataset & Model & Transform相关的config变量
    data_cfg = build_dataset_config(args)
    model_cfg = build_model_config(args)
    trans_cfg = build_trans_config(model_cfg['trans_type'])

    # 构建测试所用到的数据预处理Transform类
    val_transform, trans_cfg = build_transform(args, trans_cfg, model_cfg['max_stride'], is_train=False)

    # 构建测试所用到的Dataset类
    dataset, dataset_info = build_dataset(args, data_cfg, trans_cfg, val_transform, is_train=False)
    num_classes = dataset_info['num_classes']

    # 用于标记不同类别的bbox的颜色，更加美观
    np.random.seed(0)
    class_colors = [(np.random.randint(255),
                     np.random.randint(255),
                     np.random.randint(255)) for _ in range(num_classes)]

    # 构建YOLO模型
    model = build_model(args, model_cfg, device, num_classes, False)

    # 加载已训练好的模型权重
    model = load_weight(model, args.weight, args.fuse_conv_bn)
    model.to(device).eval()

    # 计算模型的参数量和FLOPs
    model_copy = deepcopy(model)
    model_copy.trainable = False
    model_copy.eval()
    compute_flops(
        model=model_copy,
        img_size=args.img_size, 
        device=device)
    del model_copy

    # 如果args.resave为True，则重新保存模型的权重，
    # 因为在训练阶段，模型权重文件中还包含了优化器、学习率策略等参数，这会使得权重文件过大
    # 因为，为了缩小文件的大小，可以重新保存一次，只保存模型的参数
    if args.resave:
        print('Resave: {}'.format(args.model.upper()))
        checkpoint = torch.load(args.weight, map_location='cpu')
        checkpoint_path = 'weights/{}/{}/{}_pure.pth'.format(args.dataset, args.model, args.model)
        torch.save({'model': model.state_dict(),
                    'mAP': checkpoint.pop("mAP"),
                    'epoch': checkpoint.pop("epoch")}, 
                    checkpoint_path)
        
    print("================= DETECT =================")
    # 开始在指定的数据集上去测试我们的代码
    # 对于使用VOC数据集训练出来的模型，就使用VOC测试集来做测试
    # 对于使用COCO数据集训练出来的模型，就使用COCO验证机来做测试
    test(args         = args,
         model        = model, 
         device       = device, 
         dataset      = dataset,
         transform    = val_transform,
         class_colors = class_colors,
         class_names  = dataset_info['class_names'],
         class_indexs = dataset_info['class_indexs'],
         )
