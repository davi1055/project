import argparse
import os

import cv2
import torch
import torchvision.utils as vutils

from front.back import train
from front.back.gan import Discriminator, Generator


def main(config=None):
    parser = argparse.ArgumentParser(description="Script description")
    parser.add_argument("-M", type=str, help="mode", default="train")
    parser.add_argument("--ngpu", type=int, help="num of GPU", default=0)
    parser.add_argument("--epochs", type=int, help="num of epochs", default=1)
    parser.add_argument("--seed", type=int, help="seed num", default=1)
    parser.add_argument(
        "--load_path",
        type=str,
        help="path of generator load",
        default="./models/generator_output(echops==1).pth",
    )

    parser.add_argument(
        "--result_dictionary",
        type=str,
        help="path of train result",
        default="./models/",
    )
    parser.add_argument(
        "--data_path", type=str, help="path of data", default="./dataset/"
    )
    parser.add_argument(
        "--image_channels", type=int, help="channels of image", default=1
    )
    parser.add_argument(
        "--generator_feature_maps_num",
        type=int,
        help="size of generator feature,default=64",
        default=64,
    )
    parser.add_argument(
        "--discriminator_feature_maps_num",
        type=int,
        help="size of discriminator feature",
        default=64,
    )

    args = parser.parse_args()
    mode = args.M
    ngpu = args.ngpu
    epochs = args.epochs
    data_path = args.data_path  # 数据集路径
    load_path = args.load_path  # 生成器文件加载路径
    result_dictionary = args.result_dictionary  # pth文件保存路径
    image_channels = args.image_channels
    generator_feature_maps_num = args.generator_feature_maps_num
    discriminator_feature_maps_num = args.discriminator_feature_maps_num
    seed = args.seed  # 随机种子

    if config is not None:
        # 使用config
        mode = config["mode"]
        ngpu = config["GPU_nums"]
        epochs = config["epochs"]
        result_dictionary = config["model_save_path"]
        data_path = config["dataset_path"]
        seed = config["seed"]

    trainer_args = {
        "ngpu": ngpu,
        "epochs": epochs,
        "data_path": data_path,
        "load_path": load_path,
        "result_dictionary": result_dictionary,
        "image_channels": image_channels,
        "generator_feature_maps_num": generator_feature_maps_num,
        "discriminator_feature_maps_num": discriminator_feature_maps_num,
    }

    if mode == "train":
        device = torch.device(
            "cuda:0" if (torch.cuda.is_available() and ngpu > 0) else "cpu"
        )
        # Create the generator
        netG = Generator(trainer_args).to(device)
        netG.apply(train.weights_init)
        print(netG)

        # Create the Discriminator
        netD = Discriminator(trainer_args).to(device)
        netD.apply(train.weights_init)
        print(netD)

        trainer = train.Trainer()
        trainer.train(netG, netD, trainer_args)
    if mode == "generate":
        generator = Generator(trainer_args)

        checkpoint = torch.load(
            # trainer_args["load_path"], map_location=torch.device("cpu")
            trainer_args["load_path"],
            map_location="cuda:0"
            if (torch.cuda.is_available() and ngpu > 0)
            else "cpu",
        )
        generator.load_state_dict(checkpoint)
        generator.eval()

        torch.manual_seed(seed)  # 可手动指定随机种子
        noise = torch.randn(1, 100, 1, 1)
        with torch.no_grad():
            noise[0, 0, 0, 0] = 1.0
            generated_image = generator(noise)

        output_dir_real = "./picture/test/"
        os.makedirs(output_dir_real, exist_ok=True)
        for idx, real_image_tensor in enumerate(generated_image):
            filename = f"generate_image_{idx}.png"
            save_path_real = os.path.join(output_dir_real, filename)
            vutils.save_image(real_image_tensor, save_path_real)
            # numpy_img = real_image_tensor[0].cpu().numpy()
            # cv2.imshow("", numpy_img)
            # cv2.waitKey(0)


if __name__ == "__main__":
    main(config=None)
