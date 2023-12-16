import platform
import numpy as np
import cscore as cs

import re
import math
import random
import cv2

from sort import *
from rknnlite.api import RKNNLite

import ntcore

# decice tree for rk356x/rk3588
DEVICE_COMPATIBLE_NODE = '/proc/device-tree/compatible'
mot_tracker = Sort(max_age=20, 
                       min_hits=1,
                       iou_threshold=0.05)

def get_host():
    # get platform and device type
    system = platform.system()
    machine = platform.machine()
    os_machine = system + '-' + machine
    if os_machine == 'Linux-aarch64':
        try:
            with open(DEVICE_COMPATIBLE_NODE) as f:
                device_compatible_str = f.read()
                if 'rk3588' in device_compatible_str:
                    host = 'RK3588'
                else:
                    host = 'RK356x'
        except IOError:
            print('Read device node {} failed.'.format(DEVICE_COMPATIBLE_NODE))
            exit(-1)
    else:
        host = os_machine
    return host


INPUT_SIZE = 640
IMG_PATH = './cube.jpeg'

NUM_RESULTS = 1917
NUM_CLASSES = 2

Y_SCALE = 10.0
X_SCALE = 10.0
H_SCALE = 5.0
W_SCALE = 5.0

AREA_THRESHOLD = 1640
CENTER_TARGET = 400

RK356X_RKNN_MODEL = 'resnet18_for_rk356x.rknn'
#RK3588_RKNN_MODEL = 'resnet18_for_rk3588.rknn'
RK3588_RKNN_MODEL = 'yolov5s.rknn'

BOX_THESH = 0.5
NMS_THRESH = 0.6
IMG_SIZE = 640

CLASSES = ("cone", "cube")

class NetworkTables():
    def __init__(self):
        instance = ntcore.NetworkTableInstance.getDefault()
        instance.setServer('10.54.6.2', 5810) #server_name (str), port (int)
        instance.startClient4("5406")
        instance.setUpdateRate(0.01)

        table = instance.getTable("orangepi")

        self.jsonPublish = table.getStringTopic("json").publish()
        self.coneCenter = table.getStringTopic("coneCenter").publish()
        self.cubeCenter = table.getStringTopic("cubeCenter").publish()
        self.foundCube = table.getStringTopic("foundCube").publish()
        self.foundCone = table.getStringTopic("foundCone").publish()

    def send(self, json):
        self.jsonObject = str(json)
        self.jsonPublish.set(self.jsonObject)
        self.coneCenter.set(str(json["coneCenter"]))
        self.cubeCenter.set(str(json["cubeCenter"]))
        self.foundCube.set(str(json["foundCube"]))
        self.foundCone.set(str(json["foundCone"]))


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def xywh2xyxy(x):
    # Convert [x, y, w, h] to [x1, y1, x2, y2]
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def process(input, mask, anchors):

    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    box_confidence = sigmoid(input[..., 4])
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    box_class_probs = sigmoid(input[..., 5:])

    box_xy = sigmoid(input[..., :2])*2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE/grid_h)

    box_wh = pow(sigmoid(input[..., 2:4])*2, 2)
    box_wh = box_wh * anchors

    box = np.concatenate((box_xy, box_wh), axis=-1)

    return box, box_confidence, box_class_probs


def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with box threshold. It's a bit different with origin yolov5 post process!

    # Arguments
        boxes: ndarray, boxes of objects.
        box_confidences: ndarray, confidences of objects.
        box_class_probs: ndarray, class_probs of objects.

    # Returns
        boxes: ndarray, filtered boxes.
        classes: ndarray, classes for boxes.
        scores: ndarray, scores for boxes.
    """
    box_classes = np.argmax(box_class_probs, axis=-1)
    box_class_scores = np.max(box_class_probs, axis=-1)
    pos = np.where(box_confidences[..., 0] >= BOX_THESH)

    boxes = boxes[pos]
    classes = box_classes[pos]
    scores = box_class_scores[pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.

    # Arguments
        boxes: ndarray, boxes of objects.
        scores: ndarray, scores of objects.

    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def yolov5_post_process(input_data):
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
               [59, 119], [116, 90], [156, 198], [373, 326]]

    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        keep = nms_boxes(b, s)

        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores

def track(image, boxes, scores, classes):
    dets = np.empty((0,7))
    if boxes is not None:
        #boxes[:, 2:4] += boxes[:, 0:2]
        npScores = np.array([scores])
        npClasses = np.array([classes])
        dets = np.concatenate((boxes, npScores.T), axis=1)
        dets = np.concatenate((dets, npClasses.T), axis=1)
        #print('---')
        #print(dets)
        #print('---')   
    track_bbs_ids = mot_tracker.update(dets)
    #print (track_bbs_ids)

    jsonData = {
        "foundCone": 0,
        "foundCube": 0,
        "cone": {},
        "cube": {},
        "coneCenter": 0,
        "cubeCenter": 0
    }

    if boxes is not None:
        bestCone = CENTER_TARGET
        bestCube = CENTER_TARGET
        foundCone = False
        foundCube = False
        for detection in track_bbs_ids:
            #print(detection)
            top, left, right, bottom, score, cl, oid = detection
            cl = cl.astype(int)
            #print('class: {}, oid: {}'.format(CLASSES[cl], oid))
            #print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(top, left, right, bottom))
            top = int(top)
            left = int(left)
            right = int(right)
            bottom = int(bottom) 

            centerX = abs((top + right)/2)
            centerY = abs((bottom + left)/2)
            centerDistance = abs(centerX - CENTER_TARGET)

            area = abs((right - top) * (left - bottom))
            using = False

            if CLASSES[cl] == "cube":
                if area >= AREA_THRESHOLD and centerDistance < bestCube:
                    bestCube = centerDistance
                    using = True
                    foundCube = True
                    jsonData["cubeCenter"] = centerX
            else:
                if area >= AREA_THRESHOLD and centerDistance < bestCone:
                    bestCone = centerDistance
                    using = True
                    foundCone = True
                    jsonData["coneCenter"] = centerX

            if using:
                jsonObj = {
                    "type": CLASSES[cl], 
                    "loc": detection[:3].tolist(), 
                    "conf": score,
                    "oid": oid,
                    "centerX": centerX,
                    "centerY": centerY,
                    "area": area,
                    "top": top,
                    "left": left,
                    "right": right,
                    "bottom": bottom
                    }
                    
                #print('area:', area)
                jsonData[CLASSES[cl]] = jsonObj
        #print(jsonData)
        if foundCone:
            cv2.rectangle(image, (jsonData["cone"]["top"], jsonData["cone"]["left"]), (jsonData["cone"]["right"], jsonData["cone"]["bottom"]), (255, 0, 0), 2)
            cv2.putText(image, '{0} (ID: {1})'.format(jsonData["cone"]["type"], jsonData["cone"]["oid"]),
                        (jsonData["cone"]["top"], jsonData["cone"]["left"] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
            jsonData["foundCone"] = 1
            
        if foundCube:
            cv2.rectangle(image, (jsonData["cube"]["top"], jsonData["cube"]["left"]), (jsonData["cube"]["right"], jsonData["cube"]["bottom"]), (255, 0, 0), 2)
            cv2.putText(image, '{0} (ID: {1})'.format(jsonData["cube"]["type"], jsonData["cube"]["oid"]),
                        (jsonData["cube"]["top"], jsonData["cube"]["left"] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
            jsonData["foundCube"] = 1
    nt.send(jsonData)








def draw(image, boxes, scores, classes):
    """Draw the boxes on the image.

    # Argument:
        image: original image.
        boxes: ndarray, boxes of objects.
        classes: ndarray, classes of objects.
        scores: ndarray, scores of objects.
        all_classes: all classes name.
    """
    jsonData = []
    print(track_bbs_ids)
    for detection, cl in zip(track_bbs_ids, classes):
        top, left, right, bottom, oid = detection
        print('class: {}, oid: {}'.format(CLASSES[cl], oid))
        print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(top, left, right, bottom))
        top = int(top)
        left = int(left)
        right = int(right)
        bottom = int(bottom) 
        jsonObj = {
            "type": CLASSES[cl], 
            "loc": detection[:3].tolist(), 
            #"conf": score,
            "oid": oid
            }
        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} (ID: {1})'.format(CLASSES[cl], oid),
                    (top, left - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)
        #detE = box
        #detE = np.append(detE, [score])
        #print(detE)
        #det = np.concatenate((det, [detE]), axis=0)
        jsonData.append(jsonObj)

    #nt.send(str(jsonData))
    #print(det)
    print(jsonData)


def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)


if __name__ == '__main__':
    NetworkTables()
    nt = NetworkTables()

    host_name = get_host()
    if host_name == 'RK356x':
        rknn_model = RK356X_RKNN_MODEL
    elif host_name == 'RK3588':
        rknn_model = RK3588_RKNN_MODEL
    else:
        print("This demo cannot run on the current platform: {}".format(host_name))
        exit(-1)

    rknn_lite = RKNNLite()

    # load RKNN model
    print('--> Load RKNN model')
    ret = rknn_lite.load_rknn(rknn_model)
    if ret != 0:
        print('Load RKNN model failed')
        exit(ret)
    print('done')


    # init runtime environment
    print('--> Init runtime environment')
    # run on RK356x/RK3588 with Debian OS, do not need specify target.
    if host_name == 'RK3588':
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
    else:
        ret = rknn_lite.init_runtime()
    if ret != 0:
        print('Init runtime environment failed')
        exit(ret)
    print('done')


    camera = cs.UsbCamera("usbcam", 0)
    camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 90)
    
    #fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    #out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (640, 640))

    cvsink = cs.CvSink("cvsink")
    cvsink.setSource(camera)
    cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 160, 160, 30)

    mjpegServer = cs.MjpegServer("httpserver", 8081) 
    mjpegServer.setSource(cvSource)

    frame = np.zeros(shape=(640, 480, 3), dtype=np.uint8)
    img = np.zeros(shape=(640, 640, 3), dtype=np.uint8)
    img_1 = np.zeros(shape=(640, 640, 3), dtype=np.uint8)

    while(True):
        time, frame = cvsink.grabFrame(frame)
        if time == 0:
            print("error:", cvsink.getError())
            continue
        if time:
            # img, ratio, (dw, dh) = letterbox(img, new_shape=(IMG_SIZE, IMG_SIZE))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            img = cv2.copyMakeBorder(frame, 80, 80, 0, 0, cv2.BORDER_CONSTANT, None, value = 0)
  
            #img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))   
            
            
            # Inference
            #print('--> Running model')
            outputs = rknn_lite.inference(inputs=[img])
            
            #print('done')
            
            # post process
            input0_data = outputs[0]
            input1_data = outputs[1]
            input2_data = outputs[2]
            
            input0_data = input0_data.reshape([3, -1]+list(input0_data.shape[-2:]))
            input1_data = input1_data.reshape([3, -1]+list(input1_data.shape[-2:]))
            input2_data = input2_data.reshape([3, -1]+list(input2_data.shape[-2:]))
            
            input_data = list()
            input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
            input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
            input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))
            
            boxes, classes, scores = yolov5_post_process(input_data)
            img_1 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            track(img_1, boxes, scores, classes)
            #if boxes is not None:
                #for box, score, cl in zip(boxes, scores, classes):
                #    top, left, right, bottom = box
                #    print('class: {}, score: {}'.format(CLASSES[cl], score))
                #    print('box coordinate left,top,right,down: [{}, {}, {}, {}]'.format(top, left, right, bottom))
                #draw(img_1, boxes, scores, classes)
            #out.write(img_1) 
            cvSource.putFrame(img_1)

    out.release() 	
    rknn_lite.release() 