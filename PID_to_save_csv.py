import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from scipy.optimize import minimize
import pandas as pd  # Import pandas for CSV handling

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
        a = den[0]  # Coefficient of s^2
        b = den[1]  # Coefficient of s
        c = den[2]  # Constant term
        
        # Natural frequency and damping ratio
        omega_n = np.sqrt(c / a)  # Natural frequency
        zeta = b / (2 * np.sqrt(a * c))  # Damping ratio
        
        # Time constant (approximately)
        time_constant = 1 / omega_n if omega_n != 0 else None
        
    return gain, time_constant, zeta

def calculate_pid_parameters(gain, time_constant, zeta):
    Kp = gain * (1 / zeta)  # Proportional gain
    Ti = time_constant / zeta if zeta != 0 else np.inf  # Integral time
    Ki = Kp / Ti if Ti != np.inf else 0  # Integral coefficient
    Td = time_constant         # Derivative time
    Kd = Kp * Td              # Derivative coefficient
    
    return Kp, Ki, Kd

def pid_response(Kp, Ki, Kd, num, den):
    """Simulate PID controller response."""
    
    system = ctrl.TransferFunction(num, den)
    
    # Define PID controller transfer function: C(s) = Kp + Ki/s + Kd*s
    s = ctrl.TransferFunction([1, 0], [1])  # s in Laplace domain
    pid_controller = Kp + Ki/s + Kd*s
    
    closed_loop_system = ctrl.feedback(pid_controller * system)
    
    return closed_loop_system

def cost_function(pid_params, num, den):
    """Calculate error for given PID parameters."""
    
    Kp, Ki, Kd = pid_params
    
    closed_loop_system = pid_response(Kp, Ki, Kd, num, den)
    
    t_out, y_out = ctrl.step_response(closed_loop_system)
    
    desired_response = np.ones_like(y_out)  # Desired response is a step input (1)
    
    mse = np.mean((desired_response - y_out) ** 2)  # Mean Squared Error
    
    overshoot = np.max(y_out) - desired_response[-1] if np.max(y_out) > desired_response[-1] else 0
    
    settling_time_idx = np.where(np.abs(y_out - desired_response[-1]) < 0.02)[0]
    settling_time = t_out[settling_time_idx[-1]] if len(settling_time_idx) > 0 else np.inf
    
    return mse + overshoot + settling_time

def optimize_pid_parameters(num, den):
    """Optimize PID parameters to minimize error."""
    
    gain, time_constant, zeta = analyze_transfer_function(num, den)
    
    initial_guess = calculate_pid_parameters(gain, time_constant, zeta)

    result = minimize(cost_function, initial_guess,
                      args=(num, den),
                      bounds=[(723.8,724), (0.1,0.13), (2052,2052.5)])   # Adjusted bounds for Ki and Kd
    
    optimal_params = result.x
    optimized_error = cost_function(optimal_params, num, den)

    return optimal_params, optimized_error

def plot_closed_loop_response(closed_loop_system):
    """Plot the step response of the closed-loop system."""
    
    t_out, y_out = ctrl.step_response(closed_loop_system)
    
    # Change the limit to allow up to 40 seconds.
    plt.plot(t_out[t_out <= 50], y_out[t_out <= 50])  
    
    plt.title('Closed-Loop Step Response with Optimized PID Parameters')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    
    plt.axhline(y=1.0, color='r', linestyle='--', label='Desired Response')
    
    plt.grid()
    plt.legend()
    
    plt.show()

def save_to_csv(t_out, y_out, pid_params):
    """Save step response data and PID parameters to a CSV file."""
    
    dynamic_error = 1 - y_out  # Calculate dynamic error
    
    data = {
        'Time': t_out,
        'Value of y': y_out,
        'Dynamic error': dynamic_error,
        'P': [pid_params[0]] * len(t_out),
        'I': [pid_params[1]] * len(t_out),
        'D': [pid_params[2]] * len(t_out),
        'Change. P': [0] * len(t_out),
        'Change. I': [0] * len(t_out),
        'Change. D': [0] * len(t_out),
    }
    
    df = pd.DataFrame(data)
    
    file_path = 'C:/Users/Mikhail_Kommel/Scripts/pid_response_data3.csv'
    
    df.to_csv(file_path, mode='w', header=True, index=False)  # Write header

# Example usage of the function for the transfer function with gain of 30: H(s) = 30/(4s^2 + 2s + 1)
numerator = [12]   # Numerator of the transfer function with gain of 30 (e.g., for H(s) = 30/(4s^2 + 2s + 1))
denominator = [30485.1538, 369.6785, 1]  # Denominator of the transfer function

# Optimize PID parameters to minimize error 
optimal_params, optimized_error = optimize_pid_parameters(numerator, denominator)

Kp_optimal, Ki_optimal, Kd_optimal = optimal_params

print(f'Optimized PID controller parameters:')
print(f'Kp: {Kp_optimal}')
print(f'Ki: {Ki_optimal}')
print(f'Kd: {Kd_optimal}')
print(f'Error (Cost Function Value): {optimized_error}')

# Create closed-loop system with optimized PID parameters and plot response
closed_loop_system = pid_response(Kp_optimal, Ki_optimal, Kd_optimal, numerator, denominator)

# Get step response data to save to CSV
t_out_csv , y_out_csv = ctrl.step_response(closed_loop_system)

# Save step response data and PID parameters to CSV file
#save_to_csv(t_out_csv , y_out_csv , [Kp_optimal , Ki_optimal , Kd_optimal])

# Plot the closed-loop response limiting to maximum of up to 40 seconds.
plot_closed_loop_response(closed_loop_system)