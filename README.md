# LabelArticulation

An interactive annotator for articulated object part-level 6D pose and kinematic joint

## Requirements
- Numpy
- [Open3D](http://www.open3d.org/)>=0.10.0
- [OpenCV](https://opencv.org/)
- PyQt5
- PIL

## articulation object annotation from raw scanned model

Usage: 
```
$ python guiArticulationTotalAnnotator.py
```

Keyboard HotKeys:

- Left click - Translation to the mouse pointer
- "1" and "2" - Rotation around roll axis.
- "3" and "4" - Rotation around pitch axis.
- "5" and "6" - Rotation around yaw axis.
- "w" and "s" - Translation along x axis.
- "a" and "d" - Translation along y axis.
- "e" and "c" - Translation along z axis.
- "f" - Pose refinement by ICP algorithm.

step:

1. "File" - "Import ..." select the directory of model (to be annotated)

2. Select annotation materials from the top of surface

3. Click axis align, first click a point that you think as origin, 
then use keyboard to fine-tune the material's location and rotation

4. Click part segmentation, press "k" to fix view point and draw polygon to segment the parts, 
save them in the directory "part_meshes"

5. Click joint annotation, first click a point that you think as joint's location, 
then input the joint information, joint type (prismatic or revolute), joint limit lower and upper, 
joint parent part, joint child part ..., click "play animation" you can see a video to help joint verification,
next click "save joint", you can see the articulation tree in the left of surface

6. "File" - "Export URDF..." you can export the annotation to a URDF file.


## 6D pose annotation with mouse and keyboard commands

Usage:
```
$ python gui.py
```

Keyboard HotKeys:

- Left click - Translation to the mouse pointer
- "1" and "2" - Rotation around roll axis.
- "3" and "4" - Rotation around pitch axis.
- "5" and "6" - Rotation around yaw axis.
- "7" and "8" - Translation along prismatic joint.
- "9" and "0" - Rotation along revolute joint.
- "f" - Pose refinement by ICP algorithm.

When you type "q", a final transformation matrix, "trans.json", and a transformed point cloud, "cloud_rot.ply", are saved.


## How to get RGB and depth image?
I strongly recommend to follow [pyrs](https://github.com/Toraudonn/pyrs) repository.
Images of directory "data/" were captured using that repository.

## ToDo

- [x] output a total transformation matrix
- [x] add input arguments (model name, scene name)
- [x] add input argument of initial pose
- [x] add part-level 6D pose annotation
- [x] add prismatic and revolute joint annotation
- [x] add function to choose various models
- [ ] handle depth lack points 
- [ ] visualize depth data of input scene
- [x] visualize coordinate axis
- [x] realsense to capture rgbd images
- [ ] annotate point cloud and mesh at the same time