# Boid Messages
This directory holds the declaractions for boid messages used during simulation. 

- BoidFutureTime.msg holds a time variable and status (state). E.g., Is implemented in astc_boids/status_controller_main.cpp to tell boids when to switch from READY to RUN states.
- BoidState.msg holds boid state values (position, velocity, id, time, and frame).
- SimulationTime.msg holds simulation executive state values (frame, max frame, frame per second).
