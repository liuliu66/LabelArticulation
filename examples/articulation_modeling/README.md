# Articulation Object Modeling Example

## Usage: 
```
$ python articulation_modeling.py
```
gui window

![](.readme/gui_window.png)


step:

1. "File" - "Import ..." select the directory of model (to be annotated)

![](.readme/import_raw_pc.png)

2. Select annotation materials from the top of surface

3. Click axis align, first click a point that you think as origin, 
then use keyboard to fine-tune the material's location and rotation

![](.readme/axis_align.png)

after axis align

![](.readme/axis_align_finished.png)

4. Click part segmentation, press "k" to fix view point and draw polygon to segment the parts, 
save them in the directory "part_meshes"

![](.readme/part_segmentation.png)

after part segmentation

![](.readme/part_segmentation_finished.png)

5. Click joint annotation, first click a point that you think as joint's location, 
then input the joint information, joint type (prismatic or revolute), joint limit lower and upper, 
joint parent part, joint child part

![](.readme/joint_annotation.png)

click "play animation" you can see a video to help joint verification,
next click "save joint", you can see the articulation tree in the left of surface

![](.readme/joint_animation.png)

6. "File" - "Export URDF..." you can export the annotation to a URDF file.

![](.readme/urdf_export.png)
