UAV / ROV Cooperative Control
=============================

What capabilities do I want to show Linda, Derek, and Professor Paley? What performance metrics do I want to show? What's the point I want to make?

1. I want to show that we can search an area cooperatively using the Entropy-based approach that Animesh used. How do we show this? We have a simulation of an environment w/ or w/out ROV(s) but with at least 2 UAVs which search an area in the same manner as is done in the Python simulations. What metrics do I want to show? Maybe total entropy? What is some analysis we could pull in?
- we want to state assumptions. we want to show the impact of changing different parameters. we maybe want to do some performance limitations (what is the minimum entropy achievable? What does that mean physically (like what is the most we can confidently know about the environment given the number of drones and ROVs we have in an area.)


2. I want to show that we can detect a target. How do we show that? We must show searching first, then once we start to read laser intensity high enough, we assume that that must come from a target, and we track switch to the "track" phase.

3. I want to show that we can use drones (ideally more than 1) to determine the location of the peak of the laser signal (or super-gaussian laser signal). How do we do this? After we switch to the track phase, we show other drones in the area swarming to the detection and determining if the centroid of the drones corresponds with the laser axis projected on the lateral plane.
- what if we increase the number of drones? How does that affect convergence speed or if we can converge at all? 
- Robustness to noise or movement of the drone.



