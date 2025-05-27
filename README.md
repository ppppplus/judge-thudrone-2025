# Drone Competition Judging System

A judging system for drone competitions, designed to evaluate teams' performance in drone recognition tasks. The system provides real-time video display, score calculation, and result verification capabilities.

## Features

- Real-time drone video stream display
- Automatic scoring system
- Modern graphical interface
- Support for multi-stage competition scoring
- Real-time team information and score display

## Usage Instructions

### Step 1: Connect Device
Connect the drone and start the tello_state node

### Step 2: Run Demo
```bash
chmod +x demo.sh
./demo.sh
```
After running, the interface will display team name, answers, and images. Related settings can be modified in `config.yaml`.

### Step 3: Scoring
Run `score.py` in the `scripts` directory and send messages of type `std_msgs/String` to the `/judge` topic. The interface will automatically score based on the answers.

Note:
- Takeoff and landing are scored manually
- The judging system only accepts recognition results once
- Please send results after all recognition is complete
- Results will be printed in the command line
- If the system doesn't receive results, judges can manually assign scores based on printed results

## Scoring Rules

### Stage 1
- Takeoff from starting area: 10 points
- Ball recognition: 10 points each (3 balls, in order according to rules)
- Landing: 10 points
- Example: If answer is "RGB" and received "RBG", score is 10 points

### Stage 2
- Takeoff from starting area: 10 points
- Ball recognition: 15 points each (2 balls, Team A placement area - Team B placement area order)
- Landing: 10 points
- Example: If answer is "RG" and received "RB", score is 15 points

## System Requirements

- ROS environment
- Python 3
- PyQt5
- OpenCV

## Configuration

System configuration can be modified in `scripts/config.yaml`, including:
- Team name
- Correct answers
- Competition stage

## License

[License Type]

## Language

[简体中文](README_CN.md) | English