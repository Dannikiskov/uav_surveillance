import cvlib


# Class for detecting bounding boxes of people in a given image using YOLO
class PeopleDetector:
    def __init__(self, confidence=0.2, model='yolov4-tiny', enable_gpu=False):
        self.confidence = confidence
        self.model = model
        self.enable_gpu = enable_gpu

    # Returns the bounding boxes of the people detected in the image
    def detect(self, image):
        bbox, labels, confidences = cvlib.detect_common_objects(image, confidence=self.confidence,  model=self.model, enable_gpu=self.enable_gpu)
        return [bbox[i] for i in range(len(bbox)) if labels[i] == 'person']

