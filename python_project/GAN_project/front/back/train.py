import os

import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
import torchvision.utils as vutils
from gan import Discriminator, Generator
from torchvision import datasets


# netD netG 权重初始化函数
def weights_init(m):
    classname = m.__class__.__name__
    if classname.find("Conv") != -1:
        nn.init.normal_(m.weight.data, 0.0, 0.02)
    elif classname.find("BatchNorm") != -1:
        nn.init.normal_(m.weight.data, 1.0, 0.02)
        nn.init.constant_(m.bias.data, 0)


class Trainer:
    def __init__(self):
        self.nz = 100
        self.real_label = 1.0
        self.fake_label = 0.0
        self.lr = 0.0002
        self.beta1 = 0.5

    # 训练函数
    def train(self, netG: Generator, netD: Discriminator, train_args):
        transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize((0.5,), (0.5,)),
                transforms.Resize((64, 64)),
            ]
        )

        #  设置数据集以及训练硬件
        dataset = datasets.MNIST(train_args["data_path"], True, transform=transform)
        dataloader = torch.utils.data.DataLoader(dataset, batch_size=64, shuffle=True)
        device = torch.device(
            "cuda:0"
            if (torch.cuda.is_available() and train_args["ngpu"] > 0)
            else "cpu"
        )
        device = device

        G_losses = []
        D_losses = []
        iters = 0
        criterion = nn.BCELoss()

        fixed_noise = torch.randn(64, self.nz, 1, 1, device=device)

        optimizerD = optim.Adam(
            netD.parameters(), lr=self.lr, betas=(self.beta1, 0.999)
        )
        optimizerG = optim.Adam(
            netG.parameters(), lr=self.lr, betas=(self.beta1, 0.999)
        )
        print("Starting Training Loop...")
        for epoch in range(train_args["epochs"]):
            for i, data in enumerate(dataloader, 0):
                netD.zero_grad()  # 梯度清零
                real_cpu = data[0].to(device)
                b_size = real_cpu.size(0)
                label = torch.full(
                    (b_size,), self.real_label, dtype=torch.float, device=device
                )
                output = netD(real_cpu).view(-1)  # 向前传播
                errD_real = criterion(output, label)  # 计算损失
                errD_real.backward()  # 梯度更新
                D_x = output.mean().item()  # 真实图像平均值

                noise = torch.randn(b_size, self.nz, 1, 1, device=device)
                fake = netG(noise)
                label.fill_(self.fake_label)

                output = netD(fake.detach()).view(-1)  # 通过判别器向前传播
                errD_fake = criterion(output, label)  # 判别器计算出的假图像的损失
                errD_fake.backward()  # 反向传播
                D_G_z1 = output.mean().item()  # 生成器生成的假图像的平均值（假图像的表现）

                errD = errD_real + errD_fake
                optimizerD.step()

                netG.zero_grad()
                label.fill_(self.real_label)
                output = netD(fake).view(-1)
                errG = criterion(output, label)  # 计算生成器损失
                errG.backward()  # 反向传播
                D_G_z2 = output.mean().item()
                optimizerG.step()  # 优化生成器参数

                if i % 5 == 0:
                    print(
                        "[%d/%d][%d/%d]\tLoss_D: %.4f\tLoss_G: %.4f\tD(x): %.4f\tD(G(z)): %.4f / %.4f"
                        % (
                            epoch,
                            train_args["epochs"],
                            i,
                            len(dataloader),
                            errD.item(),
                            errG.item(),
                            D_x,
                            D_G_z1,
                            D_G_z2,
                        )
                    )

                G_losses.append(errG.item())
                D_losses.append(errD.item())

                #  保存生成的图片
                if (epoch == train_args["epochs"] - 1) and (i == len(dataloader) - 1):
                    fake = netG(fixed_noise).detach().cpu()
                    output_dir = "./picture/fake/"
                    os.makedirs(output_dir, exist_ok=True)
                    for idx, image_tensor in enumerate(fake):
                        filename = f"fake_image_{iters}_{idx}.png"
                        save_path = os.path.join(output_dir, filename)
                        vutils.save_image(image_tensor, save_path)

                    output_dir_real = "./picture/true/"
                    os.makedirs(output_dir_real, exist_ok=True)
                    for idx, real_image_tensor in enumerate(real_cpu):
                        filename = f"real_image_{epoch}_{i}_{idx}.png"
                        save_path_real = os.path.join(output_dir_real, filename)
                        vutils.save_image(real_image_tensor, save_path_real)
        plt.figure(figsize=(10, 5))
        plt.title("Generator and Discriminator Loss During Training")
        plt.plot(G_losses, label="G")
        plt.plot(D_losses, label="D")
        plt.xlabel("iterations")
        plt.ylabel("Loss")
        plt.legend()
        plt.show()

        #  保存训练模型
        generator_save_path = f"{train_args['result_dictionary']}{'Gout'}.pth"
        os.makedirs(os.path.dirname(generator_save_path), exist_ok=True)
        torch.save(netG.state_dict(), generator_save_path)

        discriminator_save_path = f"{train_args['result_dictionary']}{'Dout'}.pth"
        os.makedirs(os.path.dirname(discriminator_save_path), exist_ok=True)
        torch.save(netD.state_dict(), discriminator_save_path)
        print("Generated images and models saved successfully.")
