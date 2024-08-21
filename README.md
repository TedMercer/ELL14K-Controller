# ELL14K-Controller
This is a python based package designed for users to have seamless control over multiple Thorlabs ELL14K Controllers with the use of serial commands and functions. 

This project provides a Python implementation for controlling Thorlabs Elliptec motors using serial communication. It includes functions for motor movement, velocity control, home positioning, and frequency searching, along with utility functions for converting degrees to hexadecimal values and timing motor operations.

## Features

- **Motor Movement**: Control motor movement for both relative and absolute positions.
- **Velocity Control**: Set and adjust the velocity of the motors.
- **Frequency Search**: Automatically find the optimal frequency for each motor.
- **Simultaneous Movement**: Move multiple motors simultaneously using threading.
- **Home Positioning**: Set the current motor position as the new home position.
- **Degree-to-Hex Conversion**: Utility functions for converting degrees to hexadecimal step values.
- **Timing Functionality**: Measure the time taken for motor movements at various velocities.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Functions Overview](#functions-overview)
- [Utils Module](#utils-module)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

## Installation

### Prerequisites

- Python 3.6 or higher
- The `pySerial` package for serial communication
- The `numpy` package for numericals

Install the required dependencies using pip:

```bash
pip install pyserial
pip install numpy as np
```

Clone the repository to your local machine:
```bash
  git clone https://github.com/yourusername/elliptec-motor-controller.git
  cd elliptec-motor-controller
```

## Usage

### Importing the Modules & IDE
- Make sure to import the required classes and the utility functions from the appropriate modules
- This package is designed to run within SPYDER as the IDE (Can be used within the terminal with standard imports)

```bash
from ELL14K_Controller import ElliptecMotorController
from Utils import degrees_to_hex, hex_to_degrees
```

### Creating a Motor Controller Instance
```python
# Create a motor controller instance
motor1 = ElliptecMotorController('COM4', 1)
# See methods available
motor1.list_methods()
# Close Connection
motor1.close()
```

## Functions Overview
#### ElliptecMotorController Class:
- send_command(command): Sends a command to the motor and returns the response.
- move_rel(jogstep, direction='cw'): Moves the motor a relative distance.
- move_abs(position): Moves the motor to an absolute position.
- set_current_position_as_home(): Sets the current position of the motor as the new home position.
- continuous_move(velocity, direction, duration): Moves the motor continuously at the specified velocity and direction.
  
## Utils-Module:
#### The Utils.py file contains utility functions that aid in motor control:
- degrees_to_hex(degrees): Converts degrees to hexadecimal step size.
- hex_to_degrees(hex_value): Converts hexadecimal step size back to degrees.

## Examples

## Contributing
Contributions are welcome! If you'd like to contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Make your changes and test them thoroughly.
4. Submit a pull request.
5. Please ensure your code adheres to the existing style and includes appropriate documentation and tests.

## License

### Explanation:
1. **Features**: Highlights the core features of the project.
2. **Installation**: Provides instructions for setting up the project, including dependencies and how to clone the repository.
3. **Usage**: Shows how to use the `ElliptecMotorController` class and the utility functions in the project.
4. **Functions Overview**: Briefly describes the main functions in both the `ElliptecMotorController` class and the `Utils.py` module.
5. **Examples**: Demonstrates basic usage and timing functions.
6. **Contributing**: Invites contributions from the community, with instructions for how to do so.
7. **License**: Specifies the license for the project.

This `README.md` file provides clear and concise documentation for the project, making it easy for others to understand and use your code.
