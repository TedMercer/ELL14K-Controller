# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 13:21:16 2024

@author: mercer.ed
"""

## -----------------------------------------------------------------------------
## Setup Instructions:
##
## Example Usage:
## 
## #motor 2
## Ana = create_dev('COM4')
##
## # motor 1
## Pm = create_dev('COM5')
##
## # Run some steps to position home positions to be coordinated 
## # Sometimes this is easier ran in the native Elliptec Software
## 
## Ana.home(2)
## Pm.home(1)
## 
## # This will execute 7 rotations of 7 degree steps, waiting an equal amount
## # of time on each step
##
## synchronized_move(Pm, Ana, 40, 360*7, motor1num=1, motor2num=2,
##                   degreestep='00000B28', numeric=7, sleep=0.2)
##
## NOTE: 
## master_sleep = 0.35 and internal sleep is 0.2, so total sleep is 0.55 on each
## iteration
## -----------------------------------------------------------------------------

import time
import serial
import threading
import numpy as np
import inspect
from Utils import degrees_to_hex, hex_to_degrees

## -----------------------------------------------------------------------------
master_sleep = 0.1

## -----------------------------------------------------------------------------
class ElliptecMotorController:
    def __init__(self, port, motornum, baud_rate=9600):
        self.serial_connection = serial.Serial(port, baud_rate, timeout=1)
        self.motornum = motornum
        self.optimal_frequency = None

## -----------------------------------------------------------------------------
    def send_command(self, command):
        """
        Sends a command to the motor via serial communication.
        Returns the response or None if the COM port is unavailable.
        """
        if self.serial_connection.isOpen():
            self.serial_connection.write(command.encode('utf-8'))
            self.serial_connection.write(b'\r\n')
            time.sleep(master_sleep)  # Adjust as necessary based on the command execution time
            response = self.serial_connection.read_all()
            return response.decode('utf-8')
        else:
            print("Serial connection is not open.")
            return None

## -----------------------------------------------------------------------------
    def search_frequency(self):
        """
        Finds the optimal frequency for the motor.
        Sends the search frequency command to the motor and retrieves the result.
        """
        search_command = f'{self.motornum}s1'
        response = self.send_command(search_command)
        print(response)
        if "error" not in response.lower():
            print(f"Optimal frequency found for motor {self.motornum}")
        else:
            print(f"Error during frequency search for motor {self.motornum}")

## -----------------------------------------------------------------------------
    def continuous_move(self, velocity=44, direction='cw', duration=15):
        """
        Starts the continuous movement of the motor at the given velocity.
        Default direction is clockwise ('cw'). VELOCITY RANGE: 32 - 46.
        """
        self.send_command(f'{self.motornum}sj00000000')
        velocity_command = f'{self.motornum}sv{velocity}'
        self.send_command(velocity_command)
        
        move_command = f'{self.motornum}{"fw" if direction == "cw" else "bw"}'
        self.send_command(move_command)

        print(f"Motor {self.motornum} is moving {direction} at velocity {velocity}")
        time.sleep(duration)
        self.stop()

## -----------------------------------------------------------------------------
    def home(self, motornum=None, direction='cw'):
        """
        Homes the motor to the origin position in the specified direction.
        """
        motornum = motornum or self.motornum
        self.send_command(f'{motornum}ho{"0" if direction == "cw" else "1"}')
        print('motor is home')

## -----------------------------------------------------------------------------
    def move_rel(self, jogstep, motornum=None, direction='cw'):
        """
        Moves the motor a relative distance in the specified direction.
        """
        motornum = motornum or self.motornum
        hex_steps = degrees_to_hex(jogstep)
        self.send_command(f'{motornum}sj{hex_steps}')
        self.send_command(f'{motornum}{"fw" if direction == "cw" else "bw"}')
        print(f'motor {motornum} has moved {jogstep} degrees {direction}')

## -----------------------------------------------------------------------------
    def move_abs(self, position, motornum=None):
        """
        Moves the motor to an absolute position based on the home offset.
        """
        motornum = motornum or self.motornum
        pos = degrees_to_hex(position)
        self.send_command(f'{motornum}ma{pos}')
        print(f'Motor is now at an absolute position of {position} relative to home offset')

## -----------------------------------------------------------------------------
    def get_current_position_hex(self):
        """
        Retrieves the current position of the motor in hexadecimal format.
        """
        self.send_command(f'{self.motornum}so00000000')
        position_command = f'{self.motornum}gp'
        response = self.send_command(position_command)
        
        if response:
            lines = response.splitlines()
            for line in lines:
                if 'PO' in line:
                    hex_position = line.split('PO')[1].strip()
                    if len(hex_position) == 8 and all(c in '0123456789ABCDEF' for c in hex_position.upper()):
                        return hex_position
        return None

## -----------------------------------------------------------------------------
    def set_home_offset(self, offset_hex):
        """
        Sets the home offset of the motor using the provided hexadecimal position.
        """
        set_offset_command = f'{self.motornum}so{offset_hex}'
        response = self.send_command(set_offset_command)
        if response:
            print(f"Home offset set to {offset_hex} for motor {self.motornum}")
        else:
            print(f"Failed to set home offset for motor {self.motornum}")

## -----------------------------------------------------------------------------
    def set_current_position_as_home(self):
        """
        Sets the current position of the motor as the new home position.
        """
        current_position_hex = self.get_current_position_hex()
        if current_position_hex:
            self.set_home_offset(current_position_hex)
        else:
            print("Error: Could not retrieve the current position to set as home.")

## -----------------------------------------------------------------------------
    def set_velo(self, velocity, motornum=None):
        """
        Sets the velocity of the motor.
        """
        motornum = motornum or self.motornum
        self.send_command(f'{motornum}sv{velocity}')
        print(f'velocity of motor {motornum} is set to {velocity}')

## -----------------------------------------------------------------------------
    def stop(self, motornum=None):
        """
        Stops the motor.
        """
        motornum = motornum or self.motornum
        self.send_command(f'{motornum}st')
        print(f'motor-{motornum} has stopped')

## -----------------------------------------------------------------------------    
    def list_methods(self):
        """
        Prints all callable methods in the current instance of the class.
        """
        methods = inspect.getmembers(self, predicate=inspect.ismethod)
        print("Available methods in ElliptecMotorController:")
        for name, _ in methods:
            if not name.startswith("__"):  # Exclude built-in methods
                print(f"- {name}")
## -----------------------------------------------------------------------------        
    def close(self):
        """
        Closes the COM port to allow access in other methods.
        """
        self.serial_connection.close()
        print('sucessful closeout')

## -----------------------------------------------------------------------------
def create_dev(com, motornum):
    """
    Creates a motor controller object connected to the specified COM port.
    """
    return ElliptecMotorController(com, motornum)

## -----------------------------------------------------------------------------
def synchronized_stepper_move(motor1, motor2, velocity, total_degrees, motor1num=1, 
                               motor2num=2, numeric=7, sleep=0.2):
    """
    Moves two motors synchronously to a specified total rotation using threading.
    """
    def move_motor(motor, motor_num, degreestep, velocity, sleep_time, counter_iterations):
        motor.send_command(f'{motor_num}sv{velocity}')
        motor.send_command(f'{motor_num}sj{degreestep}')
        for _ in range(counter_iterations):
            motor.send_command(f'{motor_num}fw')
            time.sleep(sleep_time)

    ds = degrees_to_hex(numeric)
    counter_iterations = int(total_degrees / numeric)

    thread1 = threading.Thread(target=move_motor, args=(motor1, motor1num, ds, velocity, sleep, counter_iterations))
    thread2 = threading.Thread(target=move_motor, args=(motor2, motor2num, ds, velocity, sleep, counter_iterations))

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

    rotations_completed = total_degrees / 360
    print(f'Rotation is complete...you have completed {rotations_completed} rotations')

## -----------------------------------------------------------------------------
def synchronized_frequency_search(motor1, motor2):
    """
    Runs a frequency search on both motors and prints the results.
    """
    motor1.search_frequency()
    motor2.search_frequency()

## -----------------------------------------------------------------------------
def simultaneous_move(motor1, motor2, velocity=32, direction='cw', duration=15,
                      offset1=13, offset2=0):
    """
    Moves two motors simultaneously using threading.
    """
    thread1 = threading.Thread(target=motor1.continuous_move, args=(velocity+offset1, direction, duration))
    thread2 = threading.Thread(target=motor2.continuous_move, args=(velocity+offset2, direction, duration))
    
    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

## -----------------------------------------------------------------------------
def move_motor_and_time(motor, degrees, velo):
    """
    Moves the motor by a specified number of degrees and times the movement.
    Waits for the motor to finish the movement before stopping the timer.
    """
    hex_steps = degrees_to_hex(degrees)
    start_time = time.time()

    motor.send_command(f'{motor.motornum}sj{hex_steps}')
    motor.send_command(f'{motor.motornum}sv{velo}')
    motor.send_command(f'{motor.motornum}fw')

    while True:
        status = motor.send_command(f'{motor.motornum}gs')
        if status.startswith(f'{motor.motornum}GS00'):
            break
        time.sleep(0.1)

    elapsed_time = time.time() - start_time
    print(f'{np.round(elapsed_time, 5)} seconds')
    return elapsed_time
## -----------------------------------------------------------------------------
def time_motor_across_velocities(motor, degrees_per_rotation=360, min_velocity=32, max_velocity=46, dur = 3):
    """
    Times how long it takes for the motor to complete a rotation at each velocity from min_velocity to max_velocity.
    
    Parameters:
    -----------
    motor : ElliptecMotorController
        The motor instance to test.
    degrees_per_rotation : int, optional
        The number of degrees for one complete rotation (default is 360).
    min_velocity : int, optional
        The minimum velocity to test (default is 32).
    max_velocity : int, optional
        The maximum velocity to test (default is 46).

    Returns:
    --------
    dict
        A dictionary containing velocities as keys and the time taken to complete a rotation as values.
    """
    results = {}
    
    # Loop over velocities from min_velocity to max_velocity
    for velocity in range(min_velocity, max_velocity + 1):
        print(f"\nTesting velocity {velocity}...")
        
        # Time the motor for one full rotation at the current velocity
        motor.send_command(f'{motor.motornum}ho0')
        elapsed_time = move_motor_and_time(motor, degrees_per_rotation, velocity)
        motor.send_command(f'{motor.motornum}ho0')
        # Store the result
        results[velocity] = np.round(elapsed_time, 5)
        print(f"Velocity: {velocity}, Time: {elapsed_time:.2f} seconds")
        print(f'waiting {dur} seconds before next velo.....')
        time.sleep(dur)
        
    
    return results
## -----------------------------------------------------------------------------
def span_angles(motors, angles, wait = 3):
    """
    Moves an array of motors to their corresponding angles using threading.

    Parameters:
    -----------
    motors : list of ElliptecMotorController
        List of motor instances to move.
    angles : list of int
        List of target angles (in degrees) corresponding to each motor.
    wait : int
        time delay between angles

    Returns:
    --------
    None
    """
    
    def move_motor_through_angles(motor, angles):
        """
        Helper function to move a motor sequentially through all angles.
        """
        for angle in angles:
            motor.move_abs(angle)
            print(f"Motor {motor.motornum} moved to {angle} degrees.")
            time.sleep(wait)

    # Create a list to hold the threads
    threads = []

    # Assign angles to motors and create threads for each motor
    for motor in motors:
        thread = threading.Thread(target=move_motor_through_angles, args=(motor, angles))
        threads.append(thread)

    # Start all threads
    for thread in threads:
        thread.start()

    # Wait for all threads to finish
    for thread in threads:
        thread.join()

    print("All motors have moved through the specified angles.")
## -----------------------------------------------------------------------------
def list_available_functions():
    """
    Prints all functions available in the Master module.
    """
    functions = inspect.getmembers(__import__(__name__), inspect.isfunction)
    print("Available functions in Master module:")
    for name, _ in functions:
        if not name.startswith("__"):
            print(f"- {name}")
