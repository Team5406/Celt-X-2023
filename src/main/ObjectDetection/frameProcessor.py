import numpy as np

import re
import math
import random
import cv2

from sort import *


CENTER_TARGET = 400

OBJ_THRESH = 0.5
NMS_THRESH = 0.6
IMG_SIZE = (640, 640)

CLASSES = ("cone", "cube")

mot_tracker = Sort(max_age=20, 
                       min_hits=1,
                       iou_threshold=0.05)
lastCubeOid = -1
lastConeOid = -1


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
    

def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold.
    """
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score* box_confidences >= OBJ_THRESH)
    scores = (class_max_score* box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

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

def dfl(position):
    # Distribution Focal Loss (DFL)
    import torch
    x = torch.tensor(position)
    n,c,h,w = x.shape
    p_num = 4
    mc = c//p_num
    y = x.reshape(n,p_num,mc,h,w)
    y = y.softmax(2)
    acc_metrix = torch.tensor(range(mc)).float().reshape(1,1,mc,1,1)
    y = (y*acc_metrix).sum(2)
    return y.numpy()

def box_process(position, anchors):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1]//grid_h, IMG_SIZE[0]//grid_w]).reshape(1,2,1,1)

    col = col.repeat(len(anchors), axis=0)
    row = row.repeat(len(anchors), axis=0)
    anchors = np.array(anchors)
    anchors = anchors.reshape(*anchors.shape, 1, 1)

    box_xy = position[:,:2,:,:]*2 - 0.5
    box_wh = pow(position[:,2:4,:,:]*2, 2) * anchors

    box_xy += grid
    box_xy *= stride
    box = np.concatenate((box_xy, box_wh), axis=1)

    # Convert [c_x, c_y, w, h] to [x1, y1, x2, y2]
    xyxy = np.copy(box)
    xyxy[:, 0, :, :] = box[:, 0, :, :] - box[:, 2, :, :]/ 2  # top left x
    xyxy[:, 1, :, :] = box[:, 1, :, :] - box[:, 3, :, :]/ 2  # top left y
    xyxy[:, 2, :, :] = box[:, 0, :, :] + box[:, 2, :, :]/ 2  # bottom right x
    xyxy[:, 3, :, :] = box[:, 1, :, :] + box[:, 3, :, :]/ 2  # bottom right y

    return xyxy
    

def post_process(input_data, anchors):
    boxes, scores, classes_conf = [], [], []
    # 1*255*h*w -> 3*85*h*w
    input_data = [_in.reshape([len(anchors[0]),-1]+list(_in.shape[-2:])) for _in in input_data]
    for i in range(len(input_data)):
        boxes.append(box_process(input_data[i][:,:4,:,:], anchors[i]))
        scores.append(input_data[i][:,4:5,:,:])
        classes_conf.append(input_data[i][:,5:,:,:])

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0,2,3,1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    # filter according to threshold
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    # nms
    nboxes, nclasses, nscores = [], [], []

    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores


def track(image, boxes, scores, classes, lock):
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
    with lock:
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
        bestCone = -1
        bestCube = -1
        foundCone = False
        foundCube = False
        coneOidMatch=False
        cubeOidMatch=False
        global lastCubeOid, lastConeOid

        for detection in track_bbs_ids:
            #print(detection)
            top, left, right, bottom, score, cl, oid = detection
            if oid == lastCubeOid:
                cubeOidMatch=True
            if oid == lastConeOid:
                coneOidMatch=True


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
            #print('oid: {}, lastCone: {}, lastCube: {}'.format(oid, lastConeOid, lastCubeOid))
            if CLASSES[cl] == "cube":
                if oid == lastCubeOid:
                    bestCube = area
                    using = True
                    foundCube = True
                    jsonData["cubeCenter"] = centerX
                    
                elif area > bestCube and not cubeOidMatch:
                    bestCube = area
                    using = True
                    foundCube = True
                    lastCubeOid = oid
                    jsonData["cubeCenter"] = centerX
            else:
                if oid == lastConeOid:
                    bestCone = area
                    using = True
                    foundCone = True
                    coneOidMatch = True
                    jsonData["coneCenter"] = centerX
                elif area > bestCone and not coneOidMatch:
                    bestCone = area
                    using = True
                    foundCone = True
                    lastConeOid = oid
                    jsonData["coneCenter"] = centerX
                    
            if oid != lastCubeOid and oid != lastConeOid:
                cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
                cv2.putText(image, '{0}'.format(CLASSES[cl]), (top, left - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

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
            cv2.rectangle(image, (jsonData["cone"]["top"], jsonData["cone"]["left"]), (jsonData["cone"]["right"], jsonData["cone"]["bottom"]), (0, 255, 255), 2)
            cv2.putText(image, '{0} (ID: {1})'.format(jsonData["cone"]["type"], jsonData["cone"]["oid"]),
                        (jsonData["cone"]["top"], jsonData["cone"]["left"] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
            jsonData["foundCone"] = 1
            
        if foundCube:
            cv2.rectangle(image, (jsonData["cube"]["top"], jsonData["cube"]["left"]), (jsonData["cube"]["right"], jsonData["cube"]["bottom"]), (0, 255, 255), 2)
            cv2.putText(image, '{0} (ID: {1})'.format(jsonData["cube"]["type"], jsonData["cube"]["oid"]),
                        (jsonData["cube"]["top"], jsonData["cube"]["left"] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
            jsonData["foundCube"] = 1
    return jsonData

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


def processFrames(rknn_lite, frame, lock, anchors):
    frame = cv2.copyMakeBorder(frame, 80, 80, 0, 0, cv2.BORDER_CONSTANT, None, value = 0)
    
    #img = np.zeros(shape=(640, 640, 3), dtype=np.uint8)

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  
    
    img = np.expand_dims(img, 0)

    outputs = rknn_lite.inference(inputs=[img])
    boxes, classes, scores = post_process(outputs, anchors)
    jsonData = track(frame, boxes, scores, classes, lock)
    return frame, jsonData
