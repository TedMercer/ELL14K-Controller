# -*- coding: utf-8 -*-
"""
Created on Wed Jul 24 13:31:48 2024

@author: mercer.ed
"""

## -----------------------------------------------------------------------------
def degrees_to_hex(degrees):
    increment_per_degree = 398
    value = degrees * increment_per_degree
    hex_value = f'{value:08X}'
    return hex_value

## -----------------------------------------------------------------------------
def hex_to_degrees(hex_value):
    """
    Converts a hexadecimal string back to degrees.
    
    Parameters:
    hex_value (str): The hexadecimal string to convert to degrees.

    Returns:
    float: The equivalent degree value.
    """
    increment_per_degree = 398
    int_value = int(hex_value, 16)
    degrees = int_value / increment_per_degree
    return degrees