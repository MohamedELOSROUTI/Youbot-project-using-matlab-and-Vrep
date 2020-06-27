# Youbot Project Project : introduction to intelligent Robotics


## 1. Introduction
<p align="center">
  <img src="https://i.imgur.com/EKD22yh.png" />
</p>


The tasks performed by the youbot are :

1) Navigating and mapping an unknown map.
2) Store the map in the youbot's memory.
3) Try to identify where are the tables and baskets.
4) Grasping process (take a cylindrical object from the table).
5) Drop it in a basket.

In this project we assume the map and its size are unknown. Our initial position regarding the
whole map is also unknown.
<p align="center">
  <img src="https://i.imgur.com/ErSEakc.png" />
</p>

To make it easier to use we have defined 4 frames 

- Map frame : The map frame is defined as the bottom-left of the occupancy grid
- Origin frame : Located at the center of the occupancy grid, it represents where youbot is starting regarding the map
- GPS frame : Coordinates given by youbot’s GPS
- Youbot frame : Youbot relative frame. 

In the end, we can define some frame transfer functions : 
   
![](https://i.imgur.com/f1c7DiW.png =600x)

To achieve mapping and navigating inside the house we mainly use the MATLAB’s Robotics
System Toolbox.

## 2. Mapping
As said in the introduction the map is considered of unknown size. Plus, our initial position regarding the whole map is also unknown. A solution would be to put the youbot in a random position in our map and then move rows and columns if needed. But this solution would take too much computation time. So we decided to create a map big enough to contain the whole house even if we don’t know where we are at the beginning. So we define a map, 50x50m with
a resolution of 8 cells per meter.

To map we use the **OccupancyGrid class**. It provides a 2D occupancy grid map based on our the width and heigth we want and a resolution (number of cells par meters). Each cell has a value representing the probability of its occupancy. A value close to 0 is a free cell, close to 1 is occupied and around 0.5 is unknown (see Figure 2). OccupancyGrid is also able to inflate occupied cells for obstacles avoidance.

### 2.1 Sensor information
Youbot uses a Hokuyo sensor to give information about it’s vision. They are expressed in
youbot’s frame so to update the OccupancyGrid we need to take into account youbot has an
 angle between it’s front axis and GPS 􀀀⃗y axis. We receive a 3*N (x, y, z) matrix containing
Hokuyo rays end points ans a other 1*N logical matrix if an end point is on a obstacle.
We now need to express these points in the map reference to update our **OccupancyGrid**.

![](https://i.imgur.com/1pEvLKi.png)
### 2.2 Occupancy grid
So we have the information from our Hokuyo sensor, but it’s not enough. We have information on the end of laser rays but we don’t have information about what’s happening between . In the beginning we used inpolygon function to return intermediate points between youbot and known points. But this function needed a meshgrid based on the basic house size
and was very costly. So, in the end, we decided to use OccupancyGrid.insertRay which needs only our youbot position and the position of points we know (see Figure 4). It will automatically compute those intermediate points and update their occupancy probability based on mid and end points probabilities we set when calling it.
<p align="center">
  <img src="https://i.imgur.com/jnnq1Uc.png" />
</p>

Plus we use OccupancyGrid.updateOccupancy for obstacles. Because Hokuyo has limited range, returned information is not necessarily a obstacle. Hokuyo provides a large number of point so we decided to downsample our list. It’s not a
problem because if we have a too large number of ray casting they will update the same cells. Plus we compensate the number of ray casting with the large number of map updates.

### 2.3 End of mapping
We now need a condition to stop mapping. We decided to look at our map occupancy. **OccupancyGrid.occupancyMap(map,
’ternary’)** allows us to get our map matrix and the term ternary makes it easier to compute how many unknown points are inside the map (because returns -1 for unknown, 0 for
free and 1 for obstacles). If this value doesn’t change when we reach a path 4-5 times in a row we assume youbot is not discovering anymore and mapping is over.
<p align="center">
  <img src="https://i.imgur.com/9wxcZcg.png" />
</p>

### 2.4 Navigation
***(The method used for navigation has still to be improved. Any suggestions to improve the navigation algorithm is much welcomed)***

First thing in our navigation is we use an inflated copy of our map to avoid using OccupancyGrid. inflate(map, radius). Then we try to choose a interesting point to navigate to using different algorithms. We then plan a path using PRM (ProbabilityRoadMap class which places random points on our map and connect them in relation the maximum distance we pass
as parameter.

- already visited. If they are close enough, less
    than 5-6 meters, we compute the angle between
    youbot’s position and these points. We then
    do a ’mean’ value using custom weight. Closer
    they are, heavier the point will be in the computation.
    We then use the meanAngle(angles)
    which is basically the following formula : 
    ![](https://i.imgur.com/2k0tyYL.png =500x)
    
- We meshgrid around youbot’s position then compute each      point norm and angle regarding to -y axis. We retrive only points between 3 to 4 meters and if the angle is in a cone of
meanAngle +/- 15°. 
<p align="center">
  <img src="https://i.imgur.com/p0BMVjT.png" />
</p>

- We then retrieve matrix indexes from these coordinates to remove duplicates, get free
cells and choose one randomly. Now transform the target indexes to coordinates to plan
a path. If no free cells was found we increase the plus/minus angle to have a wider cone until we finally find one.

- We update the PRM to take into account what we discovered since last update. We
already have the target to reach so we just call **PRM.findpath(prm, start, end)** which will return a vector composed of our start, our goal and some intermediary points. We also push these points inside a **FIFO** list called **Queue** used for driving.

### 2.5 Driving
To drive youbot we are mainly using PurePursuit which will do most of the computation. Youbot has omnidirectionnal wheels, so we still need to transform data we receive from Pure-Pursuit to make it move sideways, forward and rotating at the same time. As said previously, we use a FIFO list to keep a track of our current intermediate target. Each time we reach one of these targets we popfront() and set the next value as current target. PurePursuit needs our position plus our orientation to return a forward and a rotation speed.
In fact, PurePursuit and our frame don’t have the same angle reference. So we need to add

## 3. Vision
### 3.1 Baskets an tables dectection
After the mapping process, we get an occupancy map made of free cells (0 values) and obstacles
(1 values). The next objective is to localize the baskets and the tables in the map. There are
mainly two ways to identify them : 
1. Online mode : during the mapping and navigation, the youbot tries to catch curved
signals from the Hokuyo sensor.
2. Offline mode : after mapping, the robot tries to find circles or curved segments in the
map (since some baskets are located at corners) by using the function imfindcircles from
matlab.
We chose the second solution since it allows the robot to focus only on the mappingnavigation
process without adding any delay. However, the resolution should be high enough
in order to identify without ambiguity the tables and baskets. It’s the reason why we increased
the map resolution from 8 to 16.
<p align="center">
  <img src="https://i.imgur.com/QlgHoNy.png" />
</p>

As shown in the above figure, the tables and baskets are pretty well localized by their centers and radius. Since there are 7 tables and baskets, we take the 7 more relevant circles found on the map.

## 4. Grasping
The next objective will be to grasp the objects from one table and bring them to any place
of the room. The Inverse Kinematics solver implemented in v-rep will be used. If the center coordinate of one object is known, the rotation angle of each joint of the youbot arm is computed such as the gripper reaches the desired position.
<p align="center">
  <img src="https://i.imgur.com/tDxKG8L.png" />
</p>



The following commands will be used for these purposes : First, the IK solver has to be activated by sending the following signal : 

vrep.simxSetIntegerSignal(id, ’km_mode’, 2, vrep.simx_opmode_oneshot_wait); 

Second, the target position ([x,y,z]) of the gripper has to be specified by : 

vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, [x,y,z], vrep.simx_opmode_oneshot_wait);

Here the reference axis is armRef : located at the center of the first arm piece. In order to reach the correct position, some signal processing has to be implemented in order
to compute the center of a given object. The XYZ sensor will be used in order to identify the objects. For convenient reasons, we decide to focus only on cylindrical objects. To achieve this purpose, a matlab function called pcfitcylinder is used. It takes as input a 3D point Cloud (provided by XYZ sensor) and a reference axis. It returns a cylinder model which fits as well as possible the inlier points. The reference axis is constant since the cylinders are assumed to be initially placed according to the y axis of the XYZ sensor.

<p align="center">
  <img src="https://i.imgur.com/EhJcr43.png" />
</p>



Here's a figure that shows the youbot trying to detect the cylinders : <p align="center">
  <img src="https://i.imgur.com/cMBTcTm.jpg" />
</p>




The XYZ sensor is free to rotate according to the y axis. If a cylinder is not detected, the
XYZ sensor rotates until it finds one. If there is no cylinders, it returns an error message.
When a cylinder (with desired radius) is found, the coordinate of the center [xc; yc; zc] is set
as the new target of the youbot gripper. Furthermore, this point is in the reference of the XYZ
sensor. So the coordinate of the center has to be computed according to the armRef axis. The
image of the center can be easily computed using rotation and translation matrices.

Once a cylinder has been detected, its center is computed in the reference of the arm. Intermediate moves are performed by the youbot before reaching the center. The main trajectory is divided into two sub trajectories in order to reach the objective with max precision. The first
sub-target is located at half the final target. And the second one is located at 103 % of the final target. This value has been determined experimentally such that the cylinder doesn’t drag from the gripper. By the way, the elevation of the gripper is kept constant during the arm movement. Finally, if the cylinder is too far from the arm reference, the youbot comes closer to the cylinder in order to grasp it.
<p align="center">
  <img src="https://i.imgur.com/OniRwnW.png" />
</p>

<p align="center">
  <img src="https://i.imgur.com/m0Tli52.png" />
</p>



## 5. Conclusion
We finally managed to map the house with great performances.  However we still, sometimes, have some issues with the
navigation. The choice of the next point to discover may require additional conditions/modifications to have a very reliable algorithm. We still manage to map the whole house eighty percent of time.

Concerning driving, the youbot is able to drive in all directions using its omnidirectional wheels.

Two methods have been proposed to localize the tables and baskets. For convenience, we decided to detect baskets after the mapping process . After the grasping process, we drop the boxes/cylinders in random baskets.



# Notes

1) See .pdf file to get more information about the project.

2) Here's a video link that shows the project : https://www.youtube.com/watch?v=EAd01dVrhD4&t=168s

3) Link to download vrep edu : https://www.coppeliarobotics.com/

43) Two softwares are involved : matlab for the algorithms and vrep for the graphic interface. They have to work in parallel. 


