# Robot Path Planning with RRT and RRT*

An interactive path planner in **Python** that finds a collision-free route for a robot from a start point to a goal on a 2D obstacle map. It implements the **RRT** (Rapidly-exploring Random Tree) algorithm and its optimized variant **RRT\***, with a hands-on GUI for drawing obstacles and watching the search unfold in real time.

Built for the **Introduction to Animation and Robotics** course at the University of Haifa (2025).

<p align="center">
  <img src="images/task3_2RRT.png" alt="Classic RRT reaching the iteration limit" width="45%"/>
  <img src="images/task3.2_NOT_RRT.png" alt="RRT* finding a path" width="45%"/>
  <br/>
  <em>Same map, same 3,000-iteration budget: classic RRT (left) is still searching, while RRT* (right) has found a path.</em>
</p>

## Features

**Algorithms**
- **RRT:** samples random points, steers a fixed step toward each one from the nearest tree node (KD-tree search), and grows the tree until it reaches the goal
- **Collision detection:** Bresenham's line algorithm checks every pixel along each new edge against the obstacle map
- **Path extraction:** walks back through parent pointers and highlights the final route
- **RRT\*:** connects each new node to the lowest-cost neighbor within a radius, then *rewires* nearby nodes when the new node offers them a shorter path to the start

**Interactive GUI**
- Set the start (**S**) and goal (**D**) by clicking on the map
- Freehand **draw** (**O**) and **erase** (**E**) obstacles, like a simple paint tool
- Step-size slider, Run / Pause, Reset Tree and Reset Map buttons
- Switch between RRT and RRT\* with **C**
- Live iteration and node counters, plus an animated replay of the final path

## How It Works

<p align="center">
  <img src="images/task2_5_highlight_path.png" alt="Final path highlighted" width="60%"/>
</p>

## Documentation

- [`report.md`](report.md) – full write-up: the code for each step, collision-detection sanity checks, stopping conditions, and an RRT vs. RRT\* comparison
- [`assignment_instructions.md`](assignment_instructions.md) – the original assignment instructions

## Running

    pip install vedo numpy scipy
    python assignment4.py

A window opens with the obstacle map. Place the start and goal points, press **Run**, and watch the tree grow.

## Tech Stack

Python · NumPy · SciPy (KD-tree) · vedo (interactive visualization)

---

Developed by **Hala Ishan** · Introduction to Animation and Robotics, University of Haifa
