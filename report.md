# Report for Assignment 4 - Path Planning with RRT

**Name:** Hala Ishan  
**Student ID:** 213692296
---
**Task 1: Complete the GUI**

### Task 1.1  
Add the ability to set the source and destination points. Add a button to reset the tree. Add a slider to change the step size.

| Requirement                               | Implementation                                                                                 |
|-------------------------------------------|-----------------------------------------------------------------------------------------------|
| Set source & destination points           | Press **S** (green) or **D** (blue), then click free map tiles; invalid clicks show warnings  |
| Reset tree button                         | Clears all RRT nodes & edges but preserves current start/goal markers                         |
| Step‐size slider (1–100)                  | Slider snaps to nearest integer; updates `stepSize` and status box live                       |
| Run/Pause toggle                          | Debounced (0.1 s) start/stop of the timer driving RRT iterations                              |

![RRT GUI Controls](./images/task1_1.png)  
*Figure 1: Interactive GUI with key hints, status box, slider, Run/Pause & Reset buttons.   -    i changed the source and destination point then run it *  


## Task 1.2  
**Add a way to change the obstacle map**  
_Inspired by simple “paint” tools, we built a freehand brush for drawing and erasing obstacles in the RRT map._

| Requirement                       | Implementation                                                                                 |
|-----------------------------------|-----------------------------------------------------------------------------------------------|
| Freehand draw / erase             | **O** toggles draw mode (black brush), **E** toggles erase mode (white brush); move mouse to paint a 5×5 block under the cursor |
| No click-and-drag                 | Brush paints continuously on **mouse move**—just toggle on, sweep over the map, then toggle off |
| Endpoint validation               | Exiting draw mode automatically checks start/dest: if either lies on a newly drawn obstacle, the status box shows “⚠️ Source/Destination on obstacle – press S/D to move” |
| Reset map                         | **Reset Map** button restores the original image and clears all drawn obstacles              |

### Demonstration

[▶ Draw & Erase Obstacles](videos/draw.mkv)  
*Prior to recording, I toggled erase mode and cleared part of an original obstacle, then placed the destination point in that spot—verifying erase functionality. The clip then switches to draw mode to freehand paint new obstacles onto the map.*


 [▶ Reset Map](videos/reset.mkv)  
*The reset video shows restoring the map to its original state. After reset, if the start or destination now lies on an obstacle, the status box warns you to press S/D and move the point.*


## Task 1.2  
**Add other means to change the visualization as you see fit**  
i want to do it after i finished task 2 


## Task 2: Implement RRT

### 2.1 Complete `getNextSample`

The `getNextSample` function was verified to place every new node exactly `stepSize` units from its parent by:

- Sampling a point uniformly in the free space  
- Finding its nearest existing tree node  
- Computing the unit direction from that node toward the sample  
- Stepping exactly `stepSize` along that direction  

#### Collision-free demonstrations

- **stepSize = 60**  
  <img src="images/task2.1_step60.png" alt="RRT tree with stepSize=60" width="900"/>



- **stepSize = 10**  
  <img src="images/task2.1_step10.png" alt="RRT tree with stepSize=60" width="900"/>

> **Note:** Obstacles are not yet considered; collision checking will be added in **Task 2.2**.  
