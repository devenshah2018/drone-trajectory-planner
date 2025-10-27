# Drone Trajectory Planner

A comprehensive drone flight planning system that generates optimal flight paths for efficient aerial data capture. This system computes waypoints, camera parameters, and flight times to ensure high-quality imagery with specified overlap requirements while minimizing mission duration.

**🚀 Visit the following URL for an [interactive visualization](https://drone-path-planner.vercel.app/).**

## Overview

This project implements a complete drone trajectory planning system that:
- Generates lawn-mower flight patterns for area coverage
- Calculates optimal image spacing based on overlap/sidelap requirements
- Computes maximum flight speeds to prevent motion blur
- Optimizes flight times using trapezoidal/triangular speed profiles
- Supports both nadir and non-nadir scanning capabilities

## Features
- **Camera Modeling**: Complete pinhole camera model with projection and reprojection
- **Flight Plan Generation**: Automated waypoint generation with lawn-mower patterns
- **Image Spacing**: Precise calculation of image distances for specified overlap/sidelap
- **Speed Optimization**: Motion blur prevention through speed computation
- **Time Optimization**: Optimal flight time calculation with acceleration constraints

## Documentation

Detailed documentation and examples are available in the `main.ipynb` notebook, which includes:
- Step-by-step implementation walkthrough
- Parameter sensitivity analysis
- Visualization examples
- Performance comparisons

## Acknowledgements

This project was developed as part of **The Build Fellowship** program by **OpenAvenue Ventures**, a transformative experience that provided the foundation for building real-world engineering solutions. Special thanks to **Ayush Baid** and the **Drone Flight Planner System** team for providing mentorship and support throughout the development process.

This project was originally forked from the [Drone Flight Planner System](https://hub.buildfellowship.com/projects/drone-flight-planner-system-flight-path-for-efficient-data-capture) build project, which served as the foundation for implementing advanced flight planning algorithms and optimization techniques.

