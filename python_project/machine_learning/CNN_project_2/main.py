# std
import os
import sys
from functools import partial

# custom
from logger.logger import log, setLogLevel
from train.get_data import MnistData
from train import trainner
from model_test.test import testModel
from detect import detector
from detect.user_event import *

if __name__ == "__main__":
    # get main dir
    current_path = os.path.dirname(__file__)
    # set log level
    setLogLevel("UNKNOW")
    # user args
    user_args = ["train", "test", "visual", "detect", "all"]

    assert len(sys.argv) > 1, "\033[31mInput parameters are required"
    assert (
        sys.argv[1] in user_args
    ), f"\033[31mPlease enter one of the three parameters of {user_args}"

    # dataset
    #  It's not the detect that loads a dataset
    if sys.argv[1] != "detect":
        # load dataset
        dataset = MnistData()
        zero_fill_dataset = MnistData()
        # visualization
        is_show = True if sys.argv[1] in ["visual", "all"] else False
        log("----Zerofill start----", "INFO")
        if sys.argv[1] in ["train", "all", "visual"]:
            log("Train dataset be zerofilling...", "INFO")
            zero_fill_dataset.toZeroFillMnist((28, 28), is_show=is_show, train=True)
        if sys.argv[1] in ["test", "all", "visual"]:
            log("Test dataset be zerofilling...", "INFO")
            zero_fill_dataset.toZeroFillMnist((28, 28), is_show=is_show, train=False)

    # train
    if sys.argv[1] in ["train", "all"]:
        log("----Default Train start----", "INFO")
        trainer = trainner.Trainer()
        trainer.train(dataset.getLoader()[0])
        trainer.saveModel(f"{current_path}/default_model.pth")
        log("----Zerofill Train start----", "INFO")
        trainer = trainner.Trainer()
        trainer.train(zero_fill_dataset.getLoader()[0])
        trainer.saveModel(f"{current_path}/zero_fill_model.pth")

    # set model path
    if len(sys.argv) == 2:
        log(">>>Use defalut model path", "INFO")
        model_path = f"{current_path}/model.pth"
    if len(sys.argv) == 3:
        log(">>>Use custom model path", "INFO")
        model_path = f"{sys.argv[2]}"
    assert os.path.isfile(
        model_path
    ), "\033[31mPlease enter the path to the file that exists"

    # test
    if sys.argv[1] in ["test", "all"]:
        log("----Test start----", "INFO")
        testModel(model_path, dataset.getLoader()[1])
        testModel(model_path, zero_fill_dataset.getLoader()[1])

    # detect
    if sys.argv[1] in ["detect", "all"]:
        log("----Detect start----", "INFO")
        detector = detector.Detector()
        # set call_f
        call_f = partial(detector.Detect, model_path=model_path)
        drawInit()
        loopEvent(call_f)
