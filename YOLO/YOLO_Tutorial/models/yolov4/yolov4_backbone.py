import torch
import torch.nn as nn

try:
    from .yolov4_basic import Conv, CSPBlock
except:
    from yolov4_basic import Conv, CSPBlock
    

# ImageNet pretrained权重的链接
model_urls = {
    "cspdarknet_tiny": "https://github.com/yjh0410/image_classification_pytorch/releases/download/weight/cspdarknet_tiny.pth",
    "cspdarknet53": "https://github.com/yjh0410/image_classification_pytorch/releases/download/weight/cspdarknet53_silu.pth",
}


# --------------------- CSPDarkNet-53 -----------------------
## CSPDarkNet-53
class CSPDarkNet53(nn.Module):
    def __init__(self, act_type='silu', norm_type='BN'):
        super(CSPDarkNet53, self).__init__()
        self.feat_dims = [256, 512, 1024]

        # P1
        self.layer_1 = nn.Sequential(
            Conv(3, 32, k=3, p=1, act_type=act_type, norm_type=norm_type),
            Conv(32, 64, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(64, 64, expand_ratio=0.5, nblocks=1, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # P2
        self.layer_2 = nn.Sequential(
            Conv(64, 128, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(128, 128, expand_ratio=0.5, nblocks=2, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # P3
        self.layer_3 = nn.Sequential(
            Conv(128, 256, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(256, 256, expand_ratio=0.5, nblocks=8, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # P4
        self.layer_4 = nn.Sequential(
            Conv(256, 512, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(512, 512, expand_ratio=0.5, nblocks=8, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # P5
        self.layer_5 = nn.Sequential(
            Conv(512, 1024, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(1024, 1024, expand_ratio=0.5, nblocks=4, shortcut=True, act_type=act_type, norm_type=norm_type)
        )


    def forward(self, x):
        c1 = self.layer_1(x)
        c2 = self.layer_2(c1)
        c3 = self.layer_3(c2)
        c4 = self.layer_4(c3)
        c5 = self.layer_5(c4)

        outputs = [c3, c4, c5]

        return outputs

## CSPDarkNet-Tiny
class CSPDarkNetTiny(nn.Module):
    def __init__(self, act_type='silu', norm_type='BN'):
        super(CSPDarkNetTiny, self).__init__()
        self.feat_dims = [64, 128, 256]

        # stride = 2
        self.layer_1 = nn.Sequential(
            Conv(3, 16, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(16, 16, expand_ratio=0.5, nblocks=1, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # stride = 4
        self.layer_2 = nn.Sequential(
            Conv(16, 32, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(32, 32, expand_ratio=0.5, nblocks=1, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # stride = 8
        self.layer_3 = nn.Sequential(
            Conv(32, 64, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(64, 64, expand_ratio=0.5, nblocks=3, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # stride = 16
        self.layer_4 = nn.Sequential(
            Conv(64, 128, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(128, 128, expand_ratio=0.5, nblocks=3, shortcut=True, act_type=act_type, norm_type=norm_type)
        )
        # stride = 32
        self.layer_5 = nn.Sequential(
            Conv(128, 256, k=3, p=1, s=2, act_type=act_type, norm_type=norm_type),
            CSPBlock(256, 256, expand_ratio=0.5, nblocks=2, shortcut=True, act_type=act_type, norm_type=norm_type)
        )


    def forward(self, x):
        c1 = self.layer_1(x)
        c2 = self.layer_2(c1)
        c3 = self.layer_3(c2)
        c4 = self.layer_4(c3)
        c5 = self.layer_5(c4)

        outputs = [c3, c4, c5]

        return outputs


# --------------------- 构建CSPDarkNet网络的函数 -----------------------
## 搭建CSPDarkNet网络
def build_backbone(model_name='cspdarknet53', pretrained=False): 
    if model_name == 'cspdarknet53':
        backbone = CSPDarkNet53(act_type='silu', norm_type='BN')
        feat_dims = backbone.feat_dims
    elif model_name == 'cspdarknet_tiny':
        backbone = CSPDarkNetTiny(act_type='silu', norm_type='BN')
        feat_dims = backbone.feat_dims

    if pretrained:
        url = model_urls[model_name]
        if url is not None:
            print('Loading pretrained weight ...')
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
            print('No backbone pretrained: CSPDarkNet53')        

    return backbone, feat_dims


if __name__=='__main__':
    # 这是一段测试代码，方便读者测试能否正常的下载ResNet权重和调用ResNet网络    
    model, feat_dim = build_backbone(model_name='cspdarknet53', pretrained=False)

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
