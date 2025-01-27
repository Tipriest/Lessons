import torch
import torch.nn as nn

try:
    from .yolov7_basic import Conv, ELANBlock, DownSample
except:
    from yolov7_basic import Conv, ELANBlock, DownSample
    

# ImageNet pretrained权重的链接
model_urls = {
    "elannet_tiny": "https://github.com/yjh0410/image_classification_pytorch/releases/download/weight/yolov7_elannet_tiny.pth",
    "elannet_large": "https://github.com/yjh0410/image_classification_pytorch/releases/download/weight/yolov7_elannet_large.pth",
    "elannet_huge": "https://github.com/yjh0410/image_classification_pytorch/releases/download/weight/yolov7_elannet_huge.pth",
}


# --------------------- ELANNet -----------------------
## ELANNet-Tiny
class ELANNet_Tiny(nn.Module):
    """
    ELAN-Net of YOLOv7-Tiny.
    """
    def __init__(self, act_type='silu', norm_type='BN', depthwise=False):
        super(ELANNet_Tiny, self).__init__()
        # -------------- Basic parameters --------------
        self.feat_dims = [32, 64, 128, 256, 512]
        self.squeeze_ratios = [0.5, 0.5, 0.5, 0.5]   # Stage-1 -> Stage-4
        self.branch_depths = [1, 1, 1, 1]            # Stage-1 -> Stage-4
        
        # -------------- Network parameters --------------
        ## P1/2
        self.layer_1 = Conv(3, self.feat_dims[0], k=3, p=1, s=2, act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        ## P2/4: Stage-1
        self.layer_2 = nn.Sequential(   
            Conv(self.feat_dims[0], self.feat_dims[1], k=3, p=1, s=2, act_type=act_type, norm_type=norm_type, depthwise=depthwise),             
            ELANBlock(self.feat_dims[1], self.feat_dims[1], self.squeeze_ratios[0], self.branch_depths[0], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P3/8: Stage-2
        self.layer_3 = nn.Sequential(
            nn.MaxPool2d((2, 2), 2),             
            ELANBlock(self.feat_dims[1], self.feat_dims[2], self.squeeze_ratios[1], self.branch_depths[1], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P4/16: Stage-3
        self.layer_4 = nn.Sequential(
            nn.MaxPool2d((2, 2), 2),             
            ELANBlock(self.feat_dims[2], self.feat_dims[3], self.squeeze_ratios[2], self.branch_depths[2], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P5/32: Stage-4
        self.layer_5 = nn.Sequential(
            nn.MaxPool2d((2, 2), 2),             
            ELANBlock(self.feat_dims[3], self.feat_dims[4], self.squeeze_ratios[3], self.branch_depths[3], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )


    def forward(self, x):
        c1 = self.layer_1(x)
        c2 = self.layer_2(c1)
        c3 = self.layer_3(c2)
        c4 = self.layer_4(c3)
        c5 = self.layer_5(c4)

        outputs = [c3, c4, c5]

        return outputs

## ELANNet-Large
class ELANNet_Lagre(nn.Module):
    def __init__(self, act_type='silu', norm_type='BN', depthwise=False):
        super(ELANNet_Lagre, self).__init__()
        # -------------------- Basic parameters --------------------
        self.feat_dims = [32, 64, 128, 256, 512, 1024, 1024]
        self.squeeze_ratios = [0.5, 0.5, 0.5, 0.25]  # Stage-1 -> Stage-4
        self.branch_depths = [2, 2, 2, 2]            # Stage-1 -> Stage-4

        # -------------------- Network parameters --------------------
        ## P1/2
        self.layer_1 = nn.Sequential(
            Conv(3, self.feat_dims[0], k=3, p=1, act_type=act_type, norm_type=norm_type, depthwise=depthwise),      
            Conv(self.feat_dims[0], self.feat_dims[1], k=3, p=1, s=2, act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            Conv(self.feat_dims[1], self.feat_dims[1], k=3, p=1, act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P2/4: Stage-1
        self.layer_2 = nn.Sequential(   
            Conv(self.feat_dims[1], self.feat_dims[2], k=3, p=1, s=2, act_type=act_type, norm_type=norm_type, depthwise=depthwise),             
            ELANBlock(self.feat_dims[2], self.feat_dims[3], self.squeeze_ratios[0], self.branch_depths[0], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P3/8: Stage-2
        self.layer_3 = nn.Sequential(
            DownSample(self.feat_dims[3], self.feat_dims[3], act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            ELANBlock(self.feat_dims[3], self.feat_dims[4], self.squeeze_ratios[1], self.branch_depths[1], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P4/16: Stage-3
        self.layer_4 = nn.Sequential(
            DownSample(self.feat_dims[4], self.feat_dims[4], act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            ELANBlock(self.feat_dims[4], self.feat_dims[5], self.squeeze_ratios[2], self.branch_depths[2], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P5/32: Stage-4
        self.layer_5 = nn.Sequential(
            DownSample(self.feat_dims[5], self.feat_dims[5], act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            ELANBlock(self.feat_dims[5], self.feat_dims[6], self.squeeze_ratios[3], self.branch_depths[3], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )


    def forward(self, x):
        c1 = self.layer_1(x)
        c2 = self.layer_2(c1)
        c3 = self.layer_3(c2)
        c4 = self.layer_4(c3)
        c5 = self.layer_5(c4)

        outputs = [c3, c4, c5]

        return outputs

## ELANNet-Huge
class ELANNet_Huge(nn.Module):
    def __init__(self, act_type='silu', norm_type='BN', depthwise=False):
        super(ELANNet_Huge, self).__init__()
        # -------------------- Basic parameters --------------------
        self.feat_dims = [40, 80, 160, 320, 640, 1280, 1280]
        self.squeeze_ratios = [0.5, 0.5, 0.5, 0.25]  # Stage-1 -> Stage-4
        self.branch_depths = [3, 3, 3, 3]            # Stage-1 -> Stage-4

        # -------------------- Network parameters --------------------
        ## P1/2
        self.layer_1 = nn.Sequential(
            Conv(3, self.feat_dims[0], k=3, p=1, act_type=act_type, norm_type=norm_type, depthwise=depthwise),      
            Conv(self.feat_dims[0], self.feat_dims[1], k=3, p=1, s=2, act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            Conv(self.feat_dims[1], self.feat_dims[1], k=3, p=1, act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P2/4: Stage-1
        self.layer_2 = nn.Sequential(   
            Conv(self.feat_dims[1], self.feat_dims[2], k=3, p=1, s=2, act_type=act_type, norm_type=norm_type, depthwise=depthwise),             
            ELANBlock(self.feat_dims[2], self.feat_dims[3], self.squeeze_ratios[0], self.branch_depths[0], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P3/8: Stage-2
        self.layer_3 = nn.Sequential(
            DownSample(self.feat_dims[3], self.feat_dims[3], act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            ELANBlock(self.feat_dims[3], self.feat_dims[4], self.squeeze_ratios[1], self.branch_depths[1], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P4/16: Stage-3
        self.layer_4 = nn.Sequential(
            DownSample(self.feat_dims[4], self.feat_dims[4], act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            ELANBlock(self.feat_dims[4], self.feat_dims[5], self.squeeze_ratios[2], self.branch_depths[2], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )
        ## P5/32: Stage-4
        self.layer_5 = nn.Sequential(
            DownSample(self.feat_dims[5], self.feat_dims[5], act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            ELANBlock(self.feat_dims[5], self.feat_dims[6], self.squeeze_ratios[3], self.branch_depths[3], act_type=act_type, norm_type=norm_type, depthwise=depthwise)
        )


    def forward(self, x):
        c1 = self.layer_1(x)
        c2 = self.layer_2(c1)
        c3 = self.layer_3(c2)
        c4 = self.layer_4(c3)
        c5 = self.layer_5(c4)

        outputs = [c3, c4, c5]

        return outputs


# --------------------- 构建ELANNet网络的函数 -----------------------
def build_backbone(cfg, pretrained=False): 
    # build backbone
    if cfg['backbone'] == 'elannet_huge':
        backbone = ELANNet_Huge(cfg['bk_act'], cfg['bk_norm'], cfg['bk_dpw'])
    elif cfg['backbone'] == 'elannet_large':
        backbone = ELANNet_Lagre(cfg['bk_act'], cfg['bk_norm'], cfg['bk_dpw'])
    elif cfg['backbone'] == 'elannet_tiny':
        backbone = ELANNet_Tiny(cfg['bk_act'], cfg['bk_norm'], cfg['bk_dpw'])
    # pyramid feat dims
    feat_dims = backbone.feat_dims[-3:]

    # load imagenet pretrained weight
    if pretrained:
        url = model_urls[cfg['backbone']]
        if url is not None:
            print('Loading pretrained weight for {}.'.format(cfg['backbone'].upper()))
            checkpoint = torch.hub.load_state_dict_from_url(
                url=url, map_location="cpu", check_hash=True)
            # checkpoint state dict
            checkpoint_state_dict = checkpoint.pop("model")
            # model state dict
            model_state_dict = backbone.state_dict()
            # check
            for k in list(checkpoint_state_dict.keys()):
                if k in model_state_dict:
                    shape_model = tuple(model_state_dict[k].shape)
                    shape_checkpoint = tuple(checkpoint_state_dict[k].shape)
                    if shape_model != shape_checkpoint:
                        checkpoint_state_dict.pop(k)
                else:
                    checkpoint_state_dict.pop(k)
                    print(k)

            backbone.load_state_dict(checkpoint_state_dict)
        else:
            print('No backbone pretrained: ELANNet')        

    return backbone, feat_dims


if __name__ == '__main__':
    # 这是一段测试代码，方便读者测试能否正常的下载ResNet权重和调用ResNet网络

    # 模型的config变量
    cfg = {
        'pretrained': True,
        'backbone': 'elannet_large',
        'bk_act': 'silu',
        'bk_norm': 'BN',
        'bk_dpw': False,
    }

    # 构建模型
    model, feat_dim = build_backbone(cfg, pretrained=False)

    # 打印模型的结构
    print(model)

    # 输入图像的参数
    batch_size    = 2
    image_channel = 3
    image_height  = 512
    image_width   = 512

    # 随机生成一张图像
    image = torch.randn(batch_size, image_channel, image_height, image_width)

    # 模型推理
    output = model(image)

    # 查看模型的输出的shape
    for out in output:
        print(out.shape)
