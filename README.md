# Object-Tracking-with-YOLACT
This is an object tracking alogorithm using yolact segmentation detector. The algorithm consists of 2 main packages : `mot`, `yolact_ROS` and communicate with ROS. To obtain position value of objects, we used depth camera model `D435i` of realsense company. Also, this algorithm only detect for person. This is overall tracking workflow.
1. Yolact find the object and its instance segmentation
2. Calculate center point of segmentations
3. Obtain position values(x,y,z) of center points using depth camera
4. Publish msg including `id`, `position` from `yolact_ROS` to the `object_tracking` of mot package
5. In `mot`, mapping the positions in x,z plane which means the bird-eyed-view
6. Using multi-object-tracking algorithmn for center points, tracking the objects and calculate the current velocity(vx,vy,vz)
7. Finally publish the `object_id`, `position`, `velocity` as a result of object tracking 

<img src = "https://user-images.githubusercontent.com/78340346/170453499-066a6601-f690-4bc3-9a71-6debc8962c33.png" width=640 height=480> 
<img src = "https://user-images.githubusercontent.com/78340346/170453504-63f05509-4b74-4953-b5af-d9297a352fd4.png" width=640 height=480>

## Installation
Create workspace
```Terminal
mkdir -p tracking_ws/src
```
Clone `Object-Tracking-with-YOLACT` package. 
```Terminal
cd ~/tracking_ws/src
git clone https://github.com/choonghyun-park/Object-Tracking-with-YOLACT.git .
```
cakin_make to build your directory
```Terminal
cd ..
catkin_make
```
This repository only include additional source code for tracking alogorithm. Therefore, you should clone `yolact` and `mot` packages.
First, please clone yolact any of your directory and paste all resources to `~/tracking_ws/src/yolact_ROS/src`(Not including root directory). Here is yolact clone command
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
All installations done. Now you ready to use `Object-Tracking-with-YOLACT`. (**Note : You should be ready on prerequisites for using yolact.**)
## Run shourtcuts
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

