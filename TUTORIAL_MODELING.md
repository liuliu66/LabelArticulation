Usage: 
```
$ python articulation_modeling.py
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