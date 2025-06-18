# Animation and Robotics - Assignment 4: Path Planning with RRT

## Introduction

Path planning is a crucial aspect of robotics, involving the determination of a viable path from a start position to a goal position while avoiding obstacles. In this assignment, you will implement the Rapidly-exploring Random Tree (RRT) algorithm, a sampling-based method for solving path planning problems efficiently.

## Instructions

### Preliminary steps:
1. Ensure you have completed the setup steps from previous assignments, including the installation of Python, VS Code, and required extensions.
2. Make sure you have cloned the repository and have the environment set up correctly.

### Setup steps:
1. Create a folder with no spaces and no non-English characters (Preferably in `C:\Courses\AnimationAndRobotics\Assignments\`) and clone the assignment repository with `git clone`:

    `git clone https://github.com/xCraftCourses/animation-and-robotics-path-planning-[your github id]`
    
2. Open the folder with VS Code.
3. Create a new Python environment (`CTRL-SHIFT-P`, type `python env` and select `Python: Create Environment`). Follow the steps. VS Code should create a new folder called `.venv`.
4. Open a new terminal (`` CTRL-SHIFT-` ``). If VS Code detected the Python environment correctly, the prompt should begin with `(.venv)`. If not, restart VS Code and try again.
5. Install required packages using `pip install vedo numpy scipy` in the terminal.
6. Open `Assignment4.py`. The file is divided into cells, where each cell is defined by `#%%`. Run all of the cells. A window should appear showing obstacles and a 'Run' button. Pressing the button should begin the tree building process, but it currently misses several important components.

## Introduction to RRT

The Rapidly-exploring Random Tree (RRT) algorithm is a sampling-based path planning method that efficiently explores a configuration space. It works by incrementally building a tree structure from a start configuration, extending towards randomly sampled points in the space. The algorithm is particularly effective in high-dimensional spaces and environments with complex obstacles.

The basic RRT algorithm follows these steps:
1. Initialize the tree with the start configuration.
2. Repeat until the goal is reached or maximum iterations are met:
   a. Sample a random point in the configuration space.
   b. Find the nearest node in the tree to the sampled point.
   c. Extend the tree from the nearest node a step towards the sampled point.
   d. If the extension is collision-free, add the new node to the tree.
3. Once the goal is reached, backtrack to find the path from start to goal.

## Tasks

### Task 1: Complete the GUI

1. Add the ability to set the source and destination points. Add a button to reset the tree. Add a slider to change the step size. Place a screenshot of your GUI in the report.
2. Add a way to change the obstacle map. You can take inspiration from existing drawing tools (e.g. MS Paint). The goal is not to implement a full-blown drawing tool, but to seel a very simple way to interact with the map. Be creative! Exaplain your approach and add a screenshot or a short clip.
3. Add other means to change the visualization as you see fit. You may return to this step after you made progress on other parts of the assignment. Demonstrate your visualization in the report.

### Task 2: Implement RRT
1. Complete the implementation of getNextSample. The next sample needs to be at `stepSize` distance from the nearest sample. Add a screenshot showing the correct tree (without considering collisions yet).
2. Implement the `Collision` function. The function accepts the obstacle image as a 2D boolean array `img`, and two points `x1` and `x2`. This function should return true if a straight line path between  `x1` and `x2` with any obstacles. This should be done by finding all of the pixels on that line and checking whether the value in `img` is `True` or `False`. Add the code for the function in the report. In addition, run a sanity check and include it in the report as well.
3. Add a stopping condition in `doRRTIteration`. Reason about it in the report.
4. Setup an experiment and run it until the destination point is found. Put a screenshot in the report.
5. Implement a function to extract the path once the goal is reached. Highlight the path in a different color and show a screenshot in the report.

### Task 3: Implement RRT*
The RRT* is similar to RRT, but makes the two following modifications:
1. Connects the new sample not to the closest node, but to the node with the minimal distance to the source, in a certain, user-defined radius around the new sample.
2. Reconnects nodes in that radius to the new sample, *if* that reduces their distance to the source.

Show results in the report.

## Submission
1. Place the report in a file named `report.md` in the root directory of the repository.
2. Push your changes to the GitHub repository. Every subtask in the assignment must have a unique commit with a commit message identifying it as such. For example, once you finish subtask 2.3, create a commit with the message "Subtask 2.3 finished".

## Grading
Grading will be done based on the completeness and correctness of the implementation, the quality of the report, and adherence to best practices in coding and documentation.