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


