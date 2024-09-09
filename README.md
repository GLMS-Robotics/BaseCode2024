# Twisted Devils 2024 FTC Base Code

Welcome! This repository contains starting code for the 2024 game INTO THE DEEP. It is based on the Road Runner Quickstart repository, which itself contains the public FTC SDK for the 2023-4 season.

The following tools are included:

- The base FTCRobotController App (v10.0 at time of writing)
- RoadRunner and Roadrunner Quickstart
- Subsystem abstract class
- Op Mode wrappers for RoadRunner Action-based autonomous and iterative teleoperated with action scheduler modes, designed to simplify usage similarly to WPILib's Command-Based programming
- Action subclasses to simplify action usage and bring Actions closer to parity with WPILib Commands
	- Staged Action
	- Conditional Action
	- Motor To Position Action
	- Servo To Position Action
- Wrapper for REV LED chip
- ServoTuner op mode for finding servo target positions