# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import glob
import multiprocessing as mp
import numpy as np
import os
import tempfile
import time
import warnings
import cv2
import tqdm
import sys
import mss

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir + '/Detic')
sys.path.append(current_dir + '/Detic/third_party/CenterNet2/')

from centernet.config import add_centernet_config
from detic.config import add_detic_config
from detic.predictor import VisualizationDemo

# constants
WINDOW_NAME = "Detic"

class SegmentInterface():
    def __init__(self):
        mp.set_start_method("spawn", force=True)
        self.args = self.get_parser().parse_args('')
        print(self.args)

        setup_logger(name="fvcore")
        self.logger = setup_logger()
        self.logger.info("Arguments: " + str(self.args))

        self.cfg = self.setup_cfg(self.args)
        
        self.Detic_model = VisualizationDemo(self.cfg, self.args)
        print("Segmentation model ready..")

    def imgRGB2Segmentation(self, img):

        start_time = time.time()
        predictions, visualized_output = self.Detic_model.run_on_image(img)
        self.logger.info(
            "{} in {:.2f}s".format(
                "detected {} instances".format(len(predictions["instances"]))
                if "instances" in predictions
                else "finished",
                time.time() - start_time,
            )
        )
        visualized_output.save(self.args.output)
        # cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        # cv2.imshow(WINDOW_NAME, visualized_output.get_image()[:, :, ::-1])       
        return

    def setup_cfg(self, args):
        cfg = get_cfg()
        if args.cpu:
            cfg.MODEL.DEVICE="cpu"
        add_centernet_config(cfg)
        add_detic_config(cfg)
        print(args)
        cfg.merge_from_file(args.config_file)
        cfg.merge_from_list(args.opts)
        # Set score_threshold for builtin models
        cfg.MODEL.RETINANET.SCORE_THRESH_TEST = args.confidence_threshold
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = args.confidence_threshold
        cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = args.confidence_threshold
        cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'rand' # load later
        if not args.pred_all_class:
            cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True
        cfg.freeze()
        return cfg

    def get_parser(self):
        parser = argparse.ArgumentParser(description="Detectron2 demo for builtin configs")
        parser.add_argument(
            "--config-file",
            default= current_dir + "/Detic/configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml",
            metavar="FILE",
            help="path to config file",
        )
        parser.add_argument("--webcam", help="Take inputs from webcam.")
        parser.add_argument("--cpu", action='store_true', help="Use CPU only.")
        parser.add_argument("--video-input", help="Path to video file.")
        parser.add_argument(
            "--input",
            default= current_dir + "/Detic/data_img/segment_rgb.png",
            nargs="+",
            help="A list of space separated input images; "
            "or a single glob pattern such as 'directory/*.jpg'",
        )
        parser.add_argument(
            "--output",
            default= current_dir + "/Detic/result/out8.png",
            help="A file or directory to save output visualizations. "
            "If not given, will show output in an OpenCV window.",
        )

        parser.add_argument(
            "--vocabulary",
            default="lvis",
            choices=['lvis', 'openimages', 'objects365', 'coco', 'custom'],
            help="",
        )
        parser.add_argument(
            "--custom_vocabulary",
            default="",
            help="",
        )
        
        parser.add_argument("--pred_all_class", action='store_true')
        parser.add_argument(
            "--confidence-threshold",
            type=float,
            default=0.4,
            help="Minimum score for instance predictions to be shown",
        )
        
        parser.add_argument(
            "--opts",
            help="Modify config options using the command-line 'KEY VALUE' pairs",
            default=["MODEL.WEIGHTS", current_dir + "/Detic/models/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth"],
            nargs=argparse.REMAINDER,
        )
        
        return parser
