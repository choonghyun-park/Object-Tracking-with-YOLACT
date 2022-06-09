# Object-Tracking-with-YOLACT
This is an object tracking alogorithm using yolact segmentation detector. The algorithm consists of 2 main packages : `mot`, `yolact_ROS` and those are  communicate with ROS. To obtain position value of objects, we used depth camera model `D435i` of realsense company. Also, this algorithm only detect for person. This is overall tracking workflow.
1. Yolact find the object and its instance segmentation
2. Calculate center point of segmentations
3. Obtain position values(x,y,z) of center points using depth camera
4. Publish msg including `id`, `position` from `yolact_ROS` to the `object_tracking` of mot package
5. In `mot`, mapping the positions in x,z plane in 2 method : `bird-eyed-view` and `forward-view`
6. Using multi-object-tracking algorithm in `mot` for every center points, tracking the objects and calculate the current velocity(vx,vy,vz)
7. Finally publish the `object_id`, `position`, `velocity` as a result of object tracking 

<p align="center">
  <img src = https://user-images.githubusercontent.com/78340346/172324383-d483a7d3-f23f-4920-a91f-25458fde69d9.png width=640 height 480>
</p>

## Table of Content
Below is the whole procedures that we are going to do in order to do object tracking with Yolact, from start to end. The detailed explanation will be summarized in each section respectively.\

A. Yolact Setups\
B. Build ROS communication system\
C. Depth Camera SDK\
D. Description of yolact_ROS\
E. Description of mot(multi-object-tracking)\
F. Run shortcuts
## A. Yolact Setups
We use `Yolact` as the instance segmentation detector. While the famous real-time detector `Yolo` is widely used, `Yolo` obtain the bounding box of object and tracking this bounding box may occur much errors. As we want to detect and tracking the object more occurately, we will use `Yolact` as detector. Here are some **prerequisites of Yolact** to be installed. Also, we will use open source weight file for example `Resnet` in this object tracking section, so, if you want to train your own dataset or some other works related to Yolact, please see ![here](https://github.com/nabihandres/YOLACT/blob/main/Tutorials.md)   
### Environment
* Ubuntu 18.04 
* NVIDIA GPU driver & CUDA 10 or later \
**Note : While installing NVIDIA, the Ubuntu default gpu driver 'nouveau' may conflic with nvidia. If that is your case, add nouveau.nomodeset=0 in grub menu.**
* Anaconda Environment 
* PyTorch

### 1. Create a python3 based environment

```Shell
conda create -n yolact python=3.6
```

### 2. Install Pytorch 1.0.1 or higher and TorchVision
Check your pytorch version and type your installation command('Run this command') : 
 https://pytorch.org/get-started/locally/ \
![Run this command](https://user-images.githubusercontent.com/78340346/157592768-b90429c1-2c6c-4e25-a5f5-4c1e41693d78.png) \
Then type in the terminal :

```Shell
pip3 install torch torchvision
```

### 3. Install other packages and modules
Note : Cython needs to be installed before pycocotools!
```Shell
pip install cython
pip install opencv-python pillow pycocotools matplotlib
```
Setups for `Yolact` is doen. Now you have to make catkin workspace and build the resources.

## B. Build ROS communication system
We will use two main packages(**yolact_ROS**, **mot**) with ROS catkin workspace. So, now we start to make catkin workspace and locate package sources. Package sources are already ready in this repository but, this sources **only include additional functions** of `yolact_ROS` and `mot` and build components. This mean you have to install `yolact` and `mot` from **github** open source espectively. Don't be confused. You can install all package resources by folling below workflow. Let's start from making catkin workspace and build.
### 1. Create workspace
```Terminal
mkdir -p tracking_ws/src
```
### 2. Build workspace
This repository contains fundamental backbones of build system(`CMakeLists.txt`, `package.xml`, `msg format` ...). So, clone `Object-Tracking-with-YOLACT` package. 
```Terminal
cd ~/tracking_ws/src
git clone https://github.com/choonghyun-park/Object-Tracking-with-YOLACT.git .
```
cakin_make to build your directory
```Terminal
cd ..
catkin_make
```
### 3. Clone Yolact and mot from github 
Now, you have to install `Yolact` and `mot` package resources from github. Please keep in mind each source files of `Yolact` and `mot` should be stored in `~/tracking_ws/src/yolact_ROS/src` and `~/tracking_ws/src/mot/src` directly. In other word, no master folder(e.g. yolact) should exist in package source directly. Therefore, clone these 2 package at your favorite workspace and just paste the sources to the package src directory.
<p align="center">
  <img src = https://user-images.githubusercontent.com/78340346/172599433-ff343069-0e08-49ef-b31a-1be68b6e4a89.png width=640 height 480>
</p>

```Terminal
cd ~/YOUR_DIRECTORY
git clone https://github.com/dbolya/yolact.git
# Paste resources to ~/tracking_ws/src/yolact_ROS/src
```
You need the weights file for yolact. Please download it into `yolact_ROS/src/weights` also. Download your weights from [here](https://github.com/dbolya/yolact). \
Next, please clone mot package in same way and paste resources to `~/tracking_ws/src/mot/src`. Here is mot clone command
```Terminal
cd ~/YOUR_DIRECTORY
git clone https://github.com/mabhisharma/Multi-Object-Tracking-with-Kalman-Filter.git
# Paste resources to ~/tracking_ws/src/mot/src
```
## C. Depth Camera SDK
In purpose of getting the position data of object, we use the depth camera product of realsense campany and model name is `D435i`. To use the depth camera you have to install `realsense SDK`. The installation page is here. Follow this [tutorial](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md). Also you have to install pyrealsense2 library of python.
```terminal
pip install pyrealsense2
```
This library gives you much comfortable usage of depth camera using predefined pipeline.\ **Note** : The realsense-ros package is a well-known package which publish the entire datas of realsense library, but the image datas of ros topic published from realsense-ros should be dealed with CVBridge. And, CVBridge have many version conflict problems with python. Therefore it is proper to use pyrealsense2 library. Also, if you want to visualize the depth camera datas, use realsense-viewer(Not used in this project).  \
Now you finish the entire setup of this object tracking package. Let me introduce the description of yolact and mot how they works and show you the run shotcuts

## D. Description of yolact_ROS
Using yolact, we can detect the object and obtain the instance segmentations. From this segmentation mask, find the center point using concepts of `Center of Mass`. Next, using depth camera, I found the position datas(x,y,z) from each center points in frames(pixel_x, pixel_y). Every datas are published in `Segment_centers` msg which included in this repository and subscribed by `mot`.  
<p align="center">
  <img src = "https://user-images.githubusercontent.com/78340346/170453499-066a6601-f690-4bc3-9a71-6debc8962c33.png" width=640 height=480> 
</p>

## E. Description of mot(multi-object-tracking)
As I mentioned in the start of tutorial, many bounding box obtained detectors(e.g. Yolo) show errors in object tracking situations. The main problems are occulusion and poor Multi-object-tracking performance. Occulusion means one object may obscured by other object. Multi-object-tracking problem means the difficulty in tracking while many objects stagger each other. To improve this two situations, I implemented the mapping of detected objects to the x-z plane(bird-eyed-view) and tracking the objects by kalman filter. So, the `mot` subscribe the `Segment_centers` msg from yolact_ROS and mapping the position datas. After tracking and calculate the velocity datas of each objects, entire informations of object tracking are published in `obj_tracking` msg. Below pictures are the result of mapping and mot.
<p align="center">
  <img src = "https://user-images.githubusercontent.com/78340346/170453504-63f05509-4b74-4953-b5af-d9297a352fd4.png" width=640 height=480>
</p>

## F. Run shortcuts
**roscore**\
Terminal:
```terminal
roscore
```
**yolact_ROS**\
Terminal :
```Terminal
cd ~/tracking_ws
source devel/setup.bash
conda activate yolact
rosrun yolact_ROS eval_ROS.py --trained_model=src/yolact_ROS/src/weights/yolact_resnet50_54_800000.pth --score_threshold=0.5 --top_k=15 --image=depth
```
**mot**\
Terminal :
```
cd ~/tracking_ws
source devel/setup.bash
rosrun mot object_track_ROS.py 
```


