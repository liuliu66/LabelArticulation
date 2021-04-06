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
