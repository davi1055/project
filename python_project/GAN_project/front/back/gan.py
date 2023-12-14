import random

import torch
import torch.nn as nn
import torch.nn.parallel
import torch.utils.data

# 设定随机种子
manualSeed = 999
# manualSeed = random.randint(1, 10000)
random.seed(manualSeed)
torch.manual_seed(manualSeed)
nz = 100


# 生成器
class Generator(nn.Module):
    def __init__(self, train_args):
        super(Generator, self).__init__()
        self.ngpu = train_args["ngpu"]
        self.generate = nn.Sequential(
            nn.ConvTranspose2d(
                nz, train_args["generator_feature_maps_num"] * 8, 4, 1, 0, bias=False
            ),
            nn.BatchNorm2d(train_args["generator_feature_maps_num"] * 8),
            nn.ReLU(True),
            nn.ConvTranspose2d(
                train_args["generator_feature_maps_num"] * 8,
                train_args["generator_feature_maps_num"] * 4,
                4,
                2,
                1,
                bias=False,
            ),
            nn.BatchNorm2d(train_args["generator_feature_maps_num"] * 4),
            nn.ReLU(True),
            nn.ConvTranspose2d(
                train_args["generator_feature_maps_num"] * 4,
                train_args["generator_feature_maps_num"] * 2,
                4,
                2,
                1,
                bias=False,
            ),
            nn.BatchNorm2d(train_args["generator_feature_maps_num"] * 2),
            nn.ReLU(True),
            nn.ConvTranspose2d(
                train_args["generator_feature_maps_num"] * 2,
                train_args["generator_feature_maps_num"],
                4,
                2,
                1,
                bias=False,
            ),
            nn.BatchNorm2d(train_args["generator_feature_maps_num"]),
            nn.ReLU(True),
            nn.ConvTranspose2d(
                train_args["generator_feature_maps_num"],
                train_args["image_channels"],
                4,
                2,
                1,
                bias=False,
            ),
            nn.Tanh(),
        )

    def forward(self, input):
        return self.generate(input)


# 鉴别器
class Discriminator(nn.Module):
    def __init__(self, train_args):
        super(Discriminator, self).__init__()
        self.ngpu = train_args["ngpu"]
        self.discriminate = nn.Sequential(
            nn.Conv2d(
                train_args["image_channels"],
                train_args["discriminator_feature_maps_num"],
                4,
                2,
                1,
                bias=False,
            ),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(
                train_args["discriminator_feature_maps_num"],
                train_args["discriminator_feature_maps_num"] * 2,
                4,
                2,
                1,
                bias=False,
            ),
            nn.BatchNorm2d(train_args["discriminator_feature_maps_num"] * 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(
                train_args["discriminator_feature_maps_num"] * 2,
                train_args["discriminator_feature_maps_num"] * 4,
                4,
                2,
                1,
                bias=False,
            ),
            nn.BatchNorm2d(train_args["discriminator_feature_maps_num"] * 4),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(
                train_args["discriminator_feature_maps_num"] * 4,
                train_args["discriminator_feature_maps_num"] * 8,
                4,
                2,
                1,
                bias=False,
            ),
            nn.BatchNorm2d(train_args["discriminator_feature_maps_num"] * 8),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(
                train_args["discriminator_feature_maps_num"] * 8,
                1,
                4,
                1,
                0,
                bias=False,
            ),
            nn.Sigmoid(),
        )

    def forward(self, input):
        return self.discriminate(input)
