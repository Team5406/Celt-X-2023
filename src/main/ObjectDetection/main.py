import cv2
import time
import cscore as cs
import numpy as np


from rknnpool import rknnPoolExecutor

from frameProcessor import processFrames

import ntcore

modelPath = "yolov7.rknn"
TPEs = 3

class NetworkTables():
    def __init__(self):
        instance = ntcore.NetworkTableInstance.getDefault()
        instance.setServer('10.54.6.2', 5810) #server_name (str), port (int)
        instance.startClient4("5406")
  
        table = instance.getTable("orangepi")

        self.jsonPublish = table.getStringTopic("json").publish()
        self.coneCenter = table.getStringTopic("coneCenter").publish()
        self.cubeCenter = table.getStringTopic("cubeCenter").publish()
        self.foundCube = table.getStringTopic("foundCube").publish()
        self.foundCone = table.getStringTopic("foundCone").publish()

    def send(self, json):
        instance = ntcore.NetworkTableInstance.getDefault()
        self.jsonObject = str(json)
        self.jsonPublish.set(self.jsonObject)
        self.coneCenter.set(str(json["coneCenter"]))
        self.cubeCenter.set(str(json["cubeCenter"]))
        self.foundCube.set(str(json["foundCube"]))
        self.foundCone.set(str(json["foundCone"]))
        instance.flush()


pool = rknnPoolExecutor(
    rknnModel=modelPath,
    TPEs=TPEs,
    func=processFrames)

NetworkTables()
nt = NetworkTables()

camera = cs.UsbCamera("usbcam", 0)
camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 90)
camera.getProperty("white_balance_temperature_auto").set(0)
camera.setWhiteBalanceManual(4300)

#fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (640, 640))

cvsink = cs.CvSink("cvsink")
cvsink.setSource(camera)
cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 160, 160, 30)

mjpegServer = cs.MjpegServer("httpserver", 5800) 
mjpegServer.setSource(cvSource)

frame = np.zeros(shape=(640, 480, 3), dtype=np.uint8)

while (not camera.isConnected()):
    time.sleep(1)

i=0
if (camera.isConnected()):
    while( i <= (TPEs + 1)):
        ret, frame = cvsink.grabFrame(frame)
        if not ret:
            print("error:", cvsink.getError())
            continue
        else:
            i+=1
            pool.put(frame)

frames, loopTime, initTime = 0, time.time(), time.time()
while (True):
    frames += 1
    ret, frame = cvsink.grabFrame(frame)
    if not ret:
        continue
    pool.put(frame)
    result, flag = pool.get()
    frame, jsonData = result
    if flag == False:
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    nt.send(jsonData)
    cvSource.putFrame(frame)
    if frames % 30 == 0:
        print("30 frame average frame rate:\t", 30 / (time.time() - loopTime), "frames")
        loopTime = time.time()

print("Overall Average Frame rate\t", frames / (time.time() - initTime))
#out.release() 	
pool.release()
