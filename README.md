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

### Core Functionality
- **Camera Modeling**: Complete pinhole camera model with projection and reprojection
- **Flight Plan Generation**: Automated waypoint generation with lawn-mower patterns
- **Image Spacing**: Precise calculation of image distances for specified overlap/sidelap
- **Speed Optimization**: Motion blur prevention through speed computation
- **Time Optimization**: Optimal flight time calculation with acceleration constraints

### Advanced Features
- **Non-Nadir Scanning**: Support for tilted camera angles with footprint adjustments
- **Ground Sampling Distance (GSD)**: Accurate GSD computation for image quality assessment
- **Mission Time Calculation**: Total flight duration with realistic acceleration profiles
- **Parameter Analysis**: Tools for analyzing how camera/flight parameters affect performance

## Technical Implementation

### Data Models
- **Camera**: Pinhole camera model with intrinsic parameters and sensor specifications
- **DatasetSpec**: Flight mission specifications including overlap, height, and scan area
- **Waypoint**: Individual flight positions with coordinates, speed, and optional look-at points

### Key Algorithms
- **Trapezoidal Speed Profiles**: Optimal acceleration/deceleration between waypoints
- **Motion Blur Prevention**: Speed limits based on exposure time and ground sampling distance
- **Coverage Optimization**: Efficient area coverage with minimal flight time

## Project Structure

```
├── src/
│   ├── data_model.py          # Data models for camera, dataset, and waypoints
│   ├── camera_utils.py        # Camera projection and GSD calculations
│   ├── plan_computation.py    # Flight planning and time optimization
│   └── visualization.py       # Flight plan visualization tools
├── main.ipynb                 # Interactive demonstration notebook
└── README.md                  # This file
```

## Results & Analysis

The system provides detailed analysis including:
- **Flight Pattern Visualization**: 2D/3D plots of waypoint paths
- **Speed Analysis**: Impact of camera parameters on maximum speeds
- **Time Optimization**: Comparison with constant-speed approaches
- **Coverage Verification**: Ensures complete area coverage with specified overlap

## Documentation

Detailed documentation and examples are available in the `main.ipynb` notebook, which includes:
- Step-by-step implementation walkthrough
- Parameter sensitivity analysis
- Visualization examples
- Performance comparisons

## Acknowledgements

This project was developed as part of **The Build Fellowship** program by **OpenAvenue Ventures**, a transformative experience that provided the foundation for building real-world engineering solutions. Special thanks to **Ayush Baid** and the **Drone Flight Planner System** team for providing mentorship and support throughout the development process.

This project was originally forked from the [Drone Flight Planner System](https://hub.buildfellowship.com/projects/drone-flight-planner-system-flight-path-for-efficient-data-capture) build project, which served as the foundation for implementing advanced flight planning algorithms and optimization techniques.

