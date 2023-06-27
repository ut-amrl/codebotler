#!/usr/bin/env python3

import sys
import os
import warnings
warnings.filterwarnings("ignore")
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'  # or any {'0', '1', '2'}
from groundingdino.util.inference import load_model, load_image, predict, annotate
from groundingdino.datasets import transforms as T
import numpy as np
from PIL import Image
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../third_party/GroundingDINO"))


class GroundingDINO:
    def __init__(self, box_threshold, text_threshold, device, config_path, weights_path):
        self.device = device
        self.model = load_model(config_path, weights_path, device=self.device)
        self.model = self.model.to(device)
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold

    def predict_from_image_path(self, image_path, text_prompt):
        image, image_transformed = load_image(image_path)
        image_transformed = image_transformed.to(self.device)
        boxes, logits, phrases = predict(
            model=self.model,
            image=image_transformed,
            caption=text_prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self.device
        )
        annotated_frame = annotate(image_source=image, boxes=boxes, logits=logits, phrases=phrases)  # this is BGR
        return boxes, logits, phrases, annotated_frame[..., ::-1]

    def predict_from_image(self, img, text_prompt):
        """
        Image is assumed to be RGB.
        """
        transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        img = Image.fromarray(img)
        image = np.asarray(img)
        image_transformed, _ = transform(img, None)
        image_transformed = image_transformed.to(self.device)
        boxes, logits, phrases = predict(
            model=self.model,
            image=image_transformed,
            caption=text_prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self.device
        )
        annotated_frame = annotate(image_source=image, boxes=boxes, logits=logits, phrases=phrases)  # this is BGR
        return boxes, logits, phrases, annotated_frame[..., ::-1]
