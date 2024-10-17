import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from scipy.signal import find_peaks

def analyze_transfer_function(num, den):
    # Create the transfer function
    system = ctrl.TransferFunction(num, den)
    
    # Calculate the gain
    gain = num[0]  # Gain coefficient
    
    # Initialize variables
    time_constant = None
    delay_time = 0  # Assume delay is 0 for simplicity

    # Extract parameters from the denominator
    if len(den) == 3:  # For second order
        a = den[0]  # 4
        b = den[1]  # 2
        c = den[2]  # 1
        
        # Natural frequency and damping ratio
        omega_n = np.sqrt(c / a)  # Natural frequency
        zeta = b / (2 * np.sqrt(a * c))  # Damping ratio
        
        # Time constant (approximately)
        time_constant = 1 / omega_n if omega_n != 0 else None
        
    # Generate time vector
    t = np.linspace(0, 20, 1000)
    
    # Get the system response to a unit step signal
    t_out, y_out = ctrl.step_response(system, T=t)

    # Find peaks to determine oscillation period
    peaks, _ = find_peaks(y_out)
    
    if len(peaks) >= 2:
        periods = np.diff(t_out[peaks])  # Differences between consecutive peaks
        average_period = np.mean(periods)  # Average oscillation period
    else:
        average_period = None
    
    # Plot the graph
    plt.figure()
    plt.plot(t_out, y_out)
    plt.title('System Response to a Unit Step Signal')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid()
    plt.show()
    
    return gain, time_constant, delay_time, average_period

def calculate_pid_parameters(gain, time_constant, zeta):
    if time_constant is None or zeta is None:
        raise ValueError("Valid values for time constant and damping ratio are required.")

    Kp = gain * (1 / zeta)  # Proportional gain
    Ti = time_constant / zeta  # Integral time
    Ki = Kp / Ti              # Integral coefficient
    Td = time_constant         # Derivative time
    Kd = Kp * Td              # Derivative coefficient
    
    return Kp, Ki, Kd

# Example usage of the function for the transfer function 1/(4s^2 + 2s + 1)
numerator = [1]  # Numerator of the transfer function (e.g., for H(s) = 1/(4s^2 + 2s + 1))
denominator = [4, 2, 1]  # Denominator of the transfer function

gain, time_constant, delay_time, average_period = analyze_transfer_function(numerator, denominator)

# PID controller parameters
Kp, Ki, Kd = calculate_pid_parameters(gain, time_constant, (denominator[1] / (2 * np.sqrt(denominator[0] * denominator[2]))))

print(f'Gain coefficient: {gain}')
print(f'Time constant: {time_constant}')
print(f'Delay time: {delay_time}')
print(f'Oscillation period: {average_period}')
print(f'PID controller parameters:')
print(f'Kp: {Kp}')
print(f'Ki: {Ki}')
print(f'Kd: {Kd}')

# Plotting the system response with PID controller (add this part later)
