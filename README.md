#1 In this project, a nonlinear terminal sliding mode controller (NTSMC) is employed to track the reference altitude of a quadcopter. To achieve better performance in the presence  of disturbance, the gains of the NTSMC are adapted using a fuzzy inference system (FIS).
To conduct the simulation, follow these steps:
    1- Start by executing the sections in the "Initialization.m" file in the following sequence: "Planner," "Parameters of Drone," "NTSMC parameters," and "NTSMC Adaptive Gain Scheduling with Fuzzy Inference System."
    2- After completing the initialization steps, proceed to run the Simulink file named "quadcopter_Adaptive_fuzzy_NTSMC.slx."
    3- To observe the accuracy of altitude control, attitude control, and variations in adaptive parameters throughout the simulation, run the remaining sections available in the "Initialization.m" file.
A brief explanation of controller design, quadcopter's dynamic and results of simulation with NTSMC is presented in Dis.pdf file.
