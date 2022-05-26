# Object-Tracking-with-YOLACT
This is an object tracking alogorithm using yolact segmentation detector. The algorithm consists of 2 main packages : `mot`, `yolact_ROS` and communicate with ROS. To obtain position value of objects, we used depth camera model `D435i` of realsense company. This is overall tracking workflow.
1. Yolact find the object and its instance segmentation
2. Calculate center point of segmentations
3. Obtain position values(x,y,z) of center points using depth camera
4. Publish msg including `id`, `position` from `yolact_ROS` to the `object_tracking` of mot package
5. In `mot`, mapping the positions in x,z plane which means the bird-eyed-view
6. Using multi-object-tracking algorithmn for center points, tracking the objects and calculate the current velocity(vx,vy,vz)
7. Finally publish the `object_id`, `position`, `velocity` as a result of object tracking 

<img src = "https://user-images.githubusercontent.com/78340346/170453499-066a6601-f690-4bc3-9a71-6debc8962c33.png" width=480 height=360> 
<img src = "https://user-images.githubusercontent.com/78340346/170453504-63f05509-4b74-4953-b5af-d9297a352fd4.png" width=480 height=360>

![realsense](https://user-images.githubusercontent.com/78340346/170453499-066a6601-f690-4bc3-9a71-6debc8962c33.png)
![bird-eyed-view](https://user-images.githubusercontent.com/78340346/170453504-63f05509-4b74-4953-b5af-d9297a352fd4.png)

## Shortcuts
