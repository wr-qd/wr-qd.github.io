---
title: Note--National University of Singapore online course
date: 2025-7-12 9:56 +0800
categories: [course, Topics in Robotics]
tags: [course]     # TAG names should always be lowercase
description: The online course about Robotics
math: true
---

# lecture one
the main point is to introduces the basic aspects of the robotics.
## core technology module
**A. Mechanics** 

Familiarized with the fundamentals

- spatial representation, homogeneous transformations(齐次变换), forward and inverse kinematics(正运动学与逆运动学), velocity kinematics(速度运动学), Jacobian(雅可比行列式), static forces(静力)
- dynamics: Newtonian & Lagrangian formulation

**B. Dynamics and control**

Acquainted to the essentials

- feedback control(反馈控制), non-liner control, force control


**C. Planning & Perception**

Apply knowledge to topics in planning & perception

- path/ trajectory/ motion planning
- image formation, processing and analysis, visual tracking, vision-based control, image-guided robotics
  
## the whole structure of the course and the knowledge
![Desktop View](assets/img/robotics1.png){: width="972" height="589" }
we can have a glimpse of the whole sturcture from this picture

and this is the design process
![Desktop View](assets/img/robotics2.png){: width="972" height="589" }

## Mechanics
### overview
**kinematics(运动学)**
: The geometric description of motion, relating joint positions to end-effector pose without considering forces.

**dynamics(动力学)**
: The study of forces/torques and their effects on robotic motion.

**planning(规划)**
: The process of generating feasible paths or action sequences to achieve a task goal.

**perception(感知)**
: Algorithms that interpret sensor data to model the environment and recognize objects.

**control(控制)**
: Real-time algorithms that drive actuators to track planned trajectories while compensating for disturbances.
### Spatial Representation & Transformation
Introduction to Robotics: Fundamentals

#### Coordinate Systems
Everyday-Examples of Coordinate Systems?
- On boardgames, on maps …… even the unit number on your address
- Can be 2D, (partial) 3D, Projective……

**Homogenous Coordinate System**
将3D点/向量升维到4D空间：

- 点的表示：(x, y, z) → (x, y, z, 1)

- 方向向量的表示：(x, y, z) → (x, y, z, 0)

关键：末尾的 1 或 0 成为区分点与向量的几何标签。

**Reference Frames**
- Frame is a coordinate system usually specified in position and orientation relative to other assigned coordinate systems
- Reference frames can be assigned to rigid bodies for the description of object poses and motions

**Spatial Description**
Pose Representation (in ECE 470)
- Position and Orientation w.r.t a frame of reference
- Vector to represent position
- Matrix to represent orientation

for Matrix:
- 矩阵的列向量是单位向量（长度为1）
- 列向量之间相互正交（垂直）
- 满足正交矩阵特性

| 符号    | 含义                                |
|---------|-------------------------------------|
| A_X_B   | {B}的X轴在{A}系中的单位方向向量     |
| A_Y_B   | {B}的Y轴在{A}系中的单位方向向量     |
| A_Z_B   | {B}的Z轴在{A}系中的单位方向向量     |

✅ **应用示例**：  
当{B}是机器人末端坐标系，{A}是世界坐标系时：  
- **A_X_B** 表示末端夹爪指向的方向  
- **A_Z_B** 表示末端垂直于夹爪的方向

完整位姿 = 位置 + 方向 → 齐次变换矩阵
for check
