# mpc_manipulator

To run, do the following:

1. Download the repository.
2. Change any parameters or the goal configurations in the main.m file.
3. Run the topmost section of main.m to run the controllers. Results will be saved in a 'results' folder.
4. Run the middle section of main.m to plot the results.
5. Run the bottom section of main.m to visualize the manipulator trajectories. This requires installation of Peter Corke's Robotics Toolbox, which is open source.

List of Files

mpc.m: Run model predictive control for the manipulator
ctc.m: Run a general computed torque control for the manipulator
twoLinkRobot.m: Creates the discretized A and B matrices for the two-link planar manipulator, as well as the next state given a current state
main.m: The main file that runs the two control algorithms and does plotting and visualization.
