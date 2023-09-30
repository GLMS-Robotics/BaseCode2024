# Twisted Devils 2023 FTC Base Code

Welcome! This repository contains starting code for the 2023 FTC season CENTERSTAGE. It is based on the Road Runner Quickstart repository, which itself contains the public FTC SDK for the 2023-4 season.

The following tools are included:

- The base FTCRobotController App (v9.0 at time of writing)
- RoadRunner and Roadrunner Quickstart
- MeepMeep for testing RoadRunner paths
- Subsystem abstract class
- Op Mode wrappers for RoadRunner Action-based autonomous and iterative teleoperated with action scheduler modes, designed to simplify usage similarly to WPILib's Command-Based programming
- Action subclasses to simplify action usage and bring Actions closer to parity with WPILib Commands
	- Staged Action
	- Conditional Action
