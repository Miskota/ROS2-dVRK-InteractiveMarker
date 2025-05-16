# ROS2-dVRK-InteractiveMarker
### ROS2 Robot Programming Semester Project
An interactive marker that can be picked up and moved around with the da Vinci Robot Arm

<br>

# Plans
- Controller support
- Gravity
- Hitbox for the marker

<br>

> [!NOTE]
> Controller support will be tested on an Xbox Series controller

<br>

# Controller layout
- Left Stick: Move Horizontally
- Right Stick: Move Verticaly
- Left Trigger: Grab with jaws

# Parts
## Robot Arm Node
- Receives input (/controller_cmd)
- Processes the information
- Publishes the coordinates (/PSM1/measured_cp, /PSM1/jaw/measured_js)


## Controller Input Node
- Connect controller using pygame
- Sends controller values (/controller_cmd)
- Doesn't publish if the controller is in 'idle' state (No inputs)

## Interactive Marker
- Receives information (/PSM1/measured_cp, /PSM1/jaw/measured_js)
- If the jaws are closed (jaw_position < 0.05) the marker becomes grabbed (Color changes from green to red)
- If the jaws are open the marker becomes free (Color changes from red to green)
- While the marker is grabbed it follows the TCP coordinates
- Sends marker information to RViz (dummy_target_marker)
