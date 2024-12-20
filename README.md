# About

This node republishes a pointcloud message, and overwrites the INTENSITY value
  Additionally by filtering some range in Z axis between the max_height_ and min_height_
          (for this, uncomment marked sections "@ filter range in Z axis" )

Topics are hard coded in *.cpp, to modify them, change in source and compile. Or create a launch with topics as parameters


## Compilation

```$ catkin build repub_pc```

## Run

```$ rosrun repub_pc repPC_node```
