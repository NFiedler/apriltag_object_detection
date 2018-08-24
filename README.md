# apriltag_object_detection


### moveit connector

The moveit connector adds detected objects to the moveit planning scene. 
Objects already in the planning scene get updated unless they are attached. 
The objects added to the planning scene have a live time of 10 seconds (set in moveit_connector.h).

Start the node with:
```
rosrun apriltag_object_detection moveit_connector
```
