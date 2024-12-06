# SpineLeggedRobot2D
This repository is an open-source code for the course project of the MIT 2.74 Bio-inspired Robotics class.

#### Title: Galloping with Spine <br/>  
Here, we implemented a 2D legged robot (2 legs) with a spinal joint and investigated the benefit of the spinal joint.
The code stack is written in MATLAB and includes the derivation of the dynamics equation of motion, implementation of the simulator, and convex model predictive controller(MPC).

Contributors: [Ho Jae Lee](https://github.com/hojae-io), [Jaeyoun Choi](https://github.com/JY-HIM4U), [Harry Lee](https://github.com/harrykslee) <br/>
Video Link: [https://youtu.be/XR6XnnfqJGA](https://youtu.be/XR6XnnfqJGA) <br/>

<div align="center">
  <img width = "28.2%" src="https://github.com/user-attachments/assets/b0beea22-e163-4bc4-b33c-262504d7f906">
  <img width = "30%" src="https://github.com/user-attachments/assets/c0768579-46d7-4d2d-a82d-8c89dfd2e2e9">
  <img width = "36.8%" src="https://github.com/user-attachments/assets/3cf9caea-226d-42ef-9f52-ad24763d279f">
</div>



---
## User Manual ##
## How to Run the Simulation
1. Choose the task ('rigid' or 'spine') in `main.m`
2. Run `main.m`
3. _(Optional)_ If you change the dynamics in either `buildLegDynamics_sym.m` or `buildSpineLegDynamics_sym.m`, you have to run this script first individually. It will update codegen codes.
4. Some keyboard controls:
   - `L` : forward velocity command
   - `H` : backward velocity command
   - `R` : reset environment
   - `V` : pause simulation
   - `ESC` : end simulation and save data

## Results
Rigid robot:
<div align="center">
  <img width = "40%" src="https://github.com/user-attachments/assets/bf1a3eb6-73c2-4f84-8130-1126781ea470">
  &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;
  <img width = "35%" src="https://github.com/user-attachments/assets/acc1f296-b41e-4d8c-be9e-1c502ec0b8ea">
</div>

Spine Robot:
<div align="center">
  <img width = "40%" src="https://github.com/user-attachments/assets/c0768579-46d7-4d2d-a82d-8c89dfd2e2e9">
  &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;
  <img width = "35%" src="https://github.com/user-attachments/assets/eef928dc-e7e3-49c8-8033-e242feae4aac">
</div>

You can check more analysis results in `analysis` folder.

## Code Explanation
`main.m` : Main script that initiates Environment and Controller, and runs the experiment. <br/>
`setupProject.m` : Setup rigid robot. <br/>
`setupProject_Spine.m` : Setup spine robot. <br/>
`buildLegDynamics_sym.m` : Generate dynamics equation of motion for rigid robot using Lagrange equation of motion. <br/>
`buildSpineLegDynamics_sym.m` : Generate dynamics equation of motion for spine robot using Lagrange equation of motion. <br/>
`Simulator.m` : Class for dynamics simulator and renderer. <br/>
`Logger.m` : Class data logger and video recording <br/> 
`Environment.m` : Class for overall environment. It inherits Simulator and contains Logger. <br/>
`ConvexMPCController.m` : Class for convex MPC controller. <br/>

