#!/usr/bin/env python
# coding: utf-8

# In[1]:


from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time


# In[2]:


# Initialize the parameters
confThreshold = 0.5  #Confidence threshold
nmsThreshold = 0.5   #Non-maximum suppression threshold
inpWidth = 416       #Width of network's input image
inpHeight = 416      #Height of network's input image


# In[3]:


# Load names of classes
classesFile = "<Path to 'obj.names'>";
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')
 
# Give the configuration and weight files for the model and load the network using them

modelConfiguration = "<Path to 'yolo-obj.cfg'>";
modelWeights = "<Path to 'yolo-gesture-model.weights'>";
 
net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)


# In[4]:


# Remove the bounding boxes with low confidence using non-maxima suppression
def postprocess(frame, outs):
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]
 
    classIds = []
    confidences = []
    boxes = []
    # Scan through all the bounding boxes output from the network and keep only the
    # ones with high confidence scores. Assign the box's class label as the class with the highest score.
    classIds = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                center_x = int(detection[0] * frameWidth)
                center_y = int(detection[1] * frameHeight)
                width = int(detection[2] * frameWidth)
                height = int(detection[3] * frameHeight)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])
 
    # Perform non maximum suppression to eliminate redundant overlapping boxes with
    # lower confidences.
    indices = cv2.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
    for i in indices:
        i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        drawPred(classIds[i], confidences[i], left, top, left + width, top + height)


# In[5]:


# Get the names of the output layers
def getOutputsNames(net):
    # Get the names of all the layers in the network
    layersNames = net.getLayerNames()
    # Get the names of the output layers, i.e. the layers with unconnected outputs
    return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]


# In[6]:


# Draw the predicted bounding box
def drawPred(classId, conf, left, top, right, bottom):
    # Draw a bounding box.
    cv2.rectangle(frame, (left, top), (right, bottom), (240,248,255), 5)
     
    label = '%.2f' % conf
         
    # Get the label for the class name and its confidence
    if classes:
        assert(classId < len(classes))
        label = '%s:%s' % (classes[classId], label)
 
    #Display the label at the top of the bounding box
    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, labelSize[1])
    cv2.putText(frame, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))


# In[ ]:


# Webcam input
cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)
outputFile = "yolo_out_py.avi"
vid_writer = cv2.VideoWriter(outputFile, cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

while cv2.waitKey(1) < 0:
     
    # get frame from the video
    hasFrame, frame = cap.read()
     
    # Stop the program if reached end of video
    if not hasFrame:
        print("Done processing !!!")
        print("Output file is stored as ", outputFile)
        cv2.waitKey(3000)
        break
 
    # Create a 4D blob from a frame.
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (inpWidth, inpHeight), swapRB=True, crop=False)
 
    # Sets the input to the network
    net.setInput(blob)
    
    # Runs the forward pass to get output of the output layers
    outs = net.forward(getOutputsNames(net))
 
    # Remove the bounding boxes with low confidence
    postprocess(frame, outs)
 
    # Put efficiency information. The function getPerfProfile returns the 
    # overall time for inference(t) and the timings for each of the layers(in layersTimes)
    t, _ = net.getPerfProfile()
    label = ""
    cv2.putText(frame, label, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
 
    # Write the frame with the detection boxes
    vid_writer.write(frame.astype(np.uint8))
    
    cv2.imshow('Demo', frame)
    cv2.waitKey(3)
    
cap.release()
#out.release()


# In[ ]:




