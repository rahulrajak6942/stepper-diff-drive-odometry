# Stepper Diff Drive Rover Odometry

This repository contains the ROS 2 Python script for wheel odometry of a stepper diff drive rover. The script publishes odometry messages based on motor encoder data, providing information about the rover's position and orientation.

## Features

- Calculates odometry based on motor encoder data.
- Supports different motion modes: spot turn, crab crawl, and regular skid-steering.
- Publishes odometry messages using ROS 2.

## Requirements

- ROS 2 humble
- Python 3

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/your-username/stepper-diff-drive-odometry.git


## Configuration

Adjust the parameters in the script (`stepper_diff_drive_odometry.py`) according to your rover's specifications. Modify the wheel radius, encoder counts, and other relevant parameters to match the characteristics of your rover. Make sure to understand your rover's hardware setup and update the script accordingly.

## Contributing

Contributions are welcome! If you encounter issues, have suggestions, or want to contribute new features, feel free to open GitHub issues or submit pull requests. Your feedback and contributions help improve this project for the community.

Let's work together to make this project even better!
