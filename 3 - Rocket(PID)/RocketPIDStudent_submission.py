######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################
from typing import Dict, Tuple

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

def pressure_pd_solution(delta_t: float, current_pressure: float, target_pressure: float,
                         data: Dict) -> Tuple[float, Dict]:
    """
    Student solution to maintain pressure in the fuel tank at a level of 100.

    Args:
        delta_t (float): Time step length.
        current_pressure (float): Current pressure level of the fuel tank.
        target_pressure (float): Target pressure level of the fuel tank.
        data (dict): Data passed throughout run. Additional data can be added and existing values modified.
            'ErrorP': Proportional error. Initialized to 0.0
            'ErrorD': Derivative error. Initialized to 0.0

    Returns:
        2 item tuple representing (valve_adjustment, data):
            valve_adjustment (float): amount to adjust flow rate into the fuel tank
            data (dict): (see Args above)
    """
    data['ErrorP'] =  0.5
    data['ErrorD'] =  0.9

    error = current_pressure - target_pressure

    prev_error = data.get('prevError', 0.0)

    error_derivative = (error - prev_error) / delta_t


    valve_adjustment = -(data['ErrorP'] * error + data['ErrorD'] * error_derivative)

    data['prevError'] = error

    return valve_adjustment, data


def rocket_pid_solution(delta_t: float, current_velocity: float, optimal_velocity: float,
                        data: Dict) -> Tuple[float, Dict]:
    """
    Student solution for maintaining rocket throttle through the launch based on an optimal flight path.

    Args:
        delta_t (float): Time step length.
        current_velocity (float): Current velocity of rocket.
        optimal_velocity (float): Optimal velocity of rocket.
        data (dict): Data passed throughout run. Additional data can be added and existing values modified.
            'ErrorP': Proportional error. Initialized to 0.0
            'ErrorI': Integral error. Initialized to 0.0
            'ErrorD': Derivative error. Initialized to 0.0

    Returns:
        Throttle to set, data dictionary to be passed throughout run.
    """
    data['ErrorI'] = 0.0015
    data['ErrorP'] =  47
    data['ErrorD'] =  -8.7

    error = optimal_velocity - current_velocity

    prev_error = data.get('prevError', 0.0)
    integral_error = data.get('integralError', 0.0)

    integral_error += error

    error_derivative = (error - prev_error) / delta_t

    throttle = (data['ErrorP'] * error + data['ErrorI'] * integral_error + data['ErrorD'] * error_derivative)
    throttle = max(0.0, min(1.0, throttle))

    data['prevError'] = error
    data['integralError'] = integral_error

    return throttle, data


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith222).
    whoami = 'pmarneni3'
    return whoami
