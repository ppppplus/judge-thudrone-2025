# Drone Competition Judging System

This is a judging system for drone competitions, used to evaluate team performance in drone recognition tasks. The system provides real-time video display, score calculation, and result verification functions.

## Features

- Real-time drone video stream display
- Automatic scoring system
- Modern graphical interface
- Multi-stage competition scoring support
- Real-time team information and score display

## Instructions

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
- The judging system only accepts one recognition result
- Please send results after all recognitions are complete
- The system will print results in the command line
- If the system does not receive results, judges can manually assign scores based on printed results

## Scoring Rules

### Basic Movement Scores
- Drone takeoff: 5 points
- Drone landing: 5 points

### Ring Passing Task
- Passing through the ring specified by the opposing team: 10 points
- Double ring positions and heights are specified by teaching assistants

### Image Recognition Task (Total 20 points)
Recognize images at different positions within the time limit:
- Type A images: 3 points each
  * 3 sets of images + positions specified by the opposing team
  * Free choice of horizontal or vertical orientation
- Type B images: 1 point each
  * 5 sets of images at fixed positions specified by teaching assistants
- Type C images: 6 points each
  * Images at rotating positions specified by teaching assistants

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