# -*- coding: utf-8 -*-
"""Driving test final.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1vasfddUkFAAD8I-D3m7HSfGK_bZXlRjx
"""

!python --version

# Commented out IPython magic to ensure Python compatibility.
# Install PyTorch
!pip install torch==1.12.0 torchvision --extra-index-url https://download.pytorch.org/whl/cu113
# Install MMCV
!pip install openmim
!mim install mmcv-full==1.6.0

!rm -rf mmsegmentation
!git clone https://github.com/open-mmlab/mmsegmentation.git 
# %cd mmsegmentation
!pip install -e .

!pip install celluloid
# Check Pytorch installation
import torch, torchvision
print(torch.__version__, torch.cuda.is_available())

# Check MMSegmentation installation
import mmseg
print(mmseg.__version__)

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

import cv2
import itertools
from numba import jit, cuda
import gc
from matplotlib import animation 
from scipy.signal import convolve2d

!mkdir checkpoints
!wget https://download.openmmlab.com/mmsegmentation/v0.5/pspnet/pspnet_r50-d8_512x1024_40k_cityscapes/pspnet_r50-d8_512x1024_40k_cityscapes_20200605_003338-2966598c.pth -P checkpoints

from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot
from mmseg.core.evaluation import get_palette

config_file = 'configs/pspnet/pspnet_r50-d8_512x1024_40k_cityscapes.py'
checkpoint_file = 'checkpoints/pspnet_r50-d8_512x1024_40k_cityscapes_20200605_003338-2966598c.pth'

!ls
#%cd mmsegmentation
# build the model from a config file and a checkpoint file
model = init_segmentor(config_file, checkpoint_file, device='cuda:0')

!ls

def decompose_projection_matrix(p):
    out = cv2.decomposeProjectionMatrix(p)
    k = out[0]
    r = out[1]
    t = out[2]
    t = t/t[3]
    return k, r, t[:3]

def detect_lanes(img, segmentation_result):
    grey_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    filtered = cv2.GaussianBlur(grey_scale, (5,5), cv2.BORDER_DEFAULT)
    edges = cv2.Canny(filtered,50,100)
    road = np.array(segmentation_result == 0, dtype=int)[0]
    lanes = edges * road
    plt.imshow(lanes)
    return lanes

def filter_lane_color(lanes, original_img):
    img_hue = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower_bound = np.array([20,100,100],dtype="uint8")
    upper_bound = np.array([200,255,255],dtype="uint8")
    yellow_found = cv2.inRange(img_hue, lower_bound, upper_bound)
    grey_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #plt.imshow(img_hue)
    white_found = cv2.inRange(grey_scale, 150, 255)
    filtered_lanes = lanes * cv2.bitwise_or(yellow_found, white_found)
    plt.imshow(filtered_lanes)

intrinsic = np.array([[1.15694047e+03, 0.00000000e+00, 6.65948821e+02],
       [0.00000000e+00, 1.15213880e+03, 3.88784788e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

position = np.array([[0],[1.25],[0]])

coefficients = np.array([[-2.37638059e-01, -8.54041681e-02, -7.90999653e-04,
        -1.15882228e-04,  1.05725978e-01]])

rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])



# helpful img = cv2.undistort(img, intrinsic, coefficients)

extrinsic = np.concatenate((rotation, position),axis=1)

# assuming a flat earth
extrinsic = np.delete(extrinsic, 1, axis=1)

projection_mat = np.matmul(intrinsic, extrinsic)

inv_projection_mat = np.linalg.inv(projection_mat)

print(inv_projection_mat)

img = np.zeros((720,1280))
allPixelIndices = np.zeros((img.shape[0]*img.shape[1], 2))
for i, element in enumerate(itertools.product(range(img.shape[1]), range(img.shape[0]))):
    allPixelIndices[i, :] = element
allPixelsPadded = np.hstack((allPixelIndices, np.ones((allPixelIndices.shape[0], 1))))
worldCoords = np.dot(inv_projection_mat, allPixelsPadded.T)
worldCoords = worldCoords/worldCoords[2,:]
print(worldCoords)

def build_view(img, segmentation, fig):
    kernel = np.full((3,3), 1)
    counts = convolve2d(img, kernel, mode='same', boundary='fill', fillvalue=1)

############## for testing code on a demo video ###############
input_file = 'test.mp4'
cap = cv2.VideoCapture(input_file)

# Get the input video properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# Define the output video file and codec
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
output_file = 'output.mp4'
writer = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

i = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == False or i==10000:
        break
    result = inference_segmentor(model, frame)
    segmented_img = model.show_result(frame, result,palette=get_palette('cityscapes'),show=False,opacity=0.5)
    segmented_img = cv2.resize(segmented_img, dsize=(640, 360), interpolation=cv2.INTER_CUBIC)
    writer.write(segmented_img)
    #segmented_img = build_view(frame, result, fig)
    i+=1
    print(i)

                    
writer.release()
cap.release()

writer.release()
cap.release()

view_writer.release()      
#writer.release()
cap.release()

#img_coords = np.array([[0],[550],[1]])

#loc = np.matmul(inv_projection_mat, img_coords)
#predicted_location = loc/loc[2][0]

#print(predicted_location)

##%pylab inline

#Project by multiplying by inverse matrix
#worldCoords = np.dot(inv_projection_mat, allPixelsPadded.T)

#Convert from homogeneous to cartesian coordinates
#worldCoords = worldCoords/worldCoords[2,:]

#print(worldCoords)

#Let's visualize our projected points - might be a little slow!
#fig = figure(0, (15,15))
#scatter(worldCoords[0, :], worldCoords[1,:], c = raveledRGB/255., s = 20, alpha = 0.5)

#fig = figure(0, (15,15))
#scatter(worldCoords[0, :], worldCoords[1,:], c = raveledRGB/255., s = 20, alpha = 0.5)
#xlim([-50, 50]);
#ylim([-50, 50]);
