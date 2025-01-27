import torch
import torch.nn as nn


# ----------------------- 常用的基础模块 -----------------------
class SiLU(nn.Module):
    """export-friendly version of nn.SiLU()"""

    @staticmethod
    def forward(x):
        return x * torch.sigmoid(x)

def get_conv2d(c1, c2, k, p, s, d, g, bias=False):
    conv = nn.Conv2d(c1, c2, k, stride=s, padding=p, dilation=d, groups=g, bias=bias)

    return conv

def get_activation(act_type=None):
    if act_type == 'relu':
        return nn.ReLU(inplace=True)
    elif act_type == 'lrelu':
        return nn.LeakyReLU(0.1, inplace=True)
    elif act_type == 'mish':
        return nn.Mish(inplace=True)
    elif act_type == 'silu':
        return nn.SiLU(inplace=True)
    elif act_type is not None:
        return nn.Identity()
    else:
        raise NotImplementedError('Activation {} not implemented.'.format(act_type))

def get_norm(norm_type, dim):
    if norm_type == 'BN':
        return nn.BatchNorm2d(dim)
    elif norm_type == 'GN':
        return nn.GroupNorm(num_groups=32, num_channels=dim)
    elif norm_type is not None:
        return nn.Identity()
    else:
        raise NotImplementedError('Normalization {} not implemented.'.format(norm_type))

class Conv(nn.Module):
    def __init__(self, 
                 c1,                   # 输入通道数
                 c2,                   # 输出通道数 
                 k=1,                  # 卷积核尺寸 
                 p=0,                  # 补零的尺寸
                 s=1,                  # 卷积的步长
                 d=1,                  # 卷积膨胀系数
                 act_type='lrelu',     # 激活函数的类别
                 norm_type='BN',       # 归一化层的类别
                 depthwise=False       # 是否使用depthwise卷积
                 ):
        super(Conv, self).__init__()
        convs = []
        add_bias = False if norm_type else True

        # 构建depthwise + pointwise卷积
        if depthwise:
            convs.append(get_conv2d(c1, c1, k=k, p=p, s=s, d=d, g=c1, bias=add_bias))
            # 首先，搭建depthwise卷积
            if norm_type:
                convs.append(get_norm(norm_type, c1))
            if act_type:
                convs.append(get_activation(act_type))
            # 然后，搭建pointwise卷积
            convs.append(get_conv2d(c1, c2, k=1, p=0, s=1, d=d, g=1, bias=add_bias))
            if norm_type:
                convs.append(get_norm(norm_type, c2))
            if act_type:
                convs.append(get_activation(act_type))

        # 构建普通的标准卷积
        else:
            convs.append(get_conv2d(c1, c2, k=k, p=p, s=s, d=d, g=1, bias=add_bias))
            if norm_type:
                convs.append(get_norm(norm_type, c2))
            if act_type:
                convs.append(get_activation(act_type))
            
        self.convs = nn.Sequential(*convs)


    def forward(self, x):
        return self.convs(x)


# ----------------------- YOLOv3的必要组件 -----------------------
## YOLOv3风格的Bottleneck
class Bottleneck(nn.Module):
    def __init__(self,
                 in_dim       :int,
                 out_dim      :int,
                 expand_ratio :float = 0.5,
                 shortcut     :bool  = False,
                 depthwise    :bool  = False,
                 act_type     :str   = 'silu',
                 norm_type    :str   = 'BN'):
        super(Bottleneck, self).__init__()
        inter_dim = int(out_dim * expand_ratio)
        self.cv1 = Conv(in_dim, inter_dim, k=1, norm_type=norm_type, act_type=act_type)
        self.cv2 = Conv(inter_dim, out_dim, k=3, p=1, norm_type=norm_type, act_type=act_type, depthwise=depthwise)
        self.shortcut = shortcut and in_dim == out_dim

    def forward(self, x):
        h = self.cv2(self.cv1(x))

        return x + h if self.shortcut else h

## YOLOv3风格的ResBlock
class ResBlock(nn.Module):
    def __init__(self,
                 in_dim    :int,
                 out_dim   :int,
                 nblocks   :int = 1,
                 act_type  :str = 'silu',
                 norm_type :str = 'BN'):
        super(ResBlock, self).__init__()
        assert in_dim == out_dim
        self.m = nn.Sequential(*[
            Bottleneck(in_dim, out_dim, expand_ratio=0.5, shortcut=True,
                       norm_type=norm_type, act_type=act_type)
                       for _ in range(nblocks)
                       ])

    def forward(self, x):
        return self.m(x)

## YOLOv3风格的ConvBlock
class ConvBlocks(nn.Module):
    def __init__(self,
                 in_dim    :int,
                 out_dim   :int,
                 act_type  :str  = 'silu',
                 norm_type :str  = 'BN',
                 depthwise :bool = False):
        super().__init__()
        inter_dim = out_dim // 2
        self.convs = nn.Sequential(
            Conv(in_dim, out_dim, k=1, act_type=act_type, norm_type=norm_type),
            Conv(out_dim, inter_dim, k=3, p=1, act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            Conv(inter_dim, out_dim, k=1, act_type=act_type, norm_type=norm_type),
            Conv(out_dim, inter_dim, k=3, p=1, act_type=act_type, norm_type=norm_type, depthwise=depthwise),
            Conv(inter_dim, out_dim, k=1, act_type=act_type, norm_type=norm_type)
        )

    def forward(self, x):
        return self.convs(x)
    