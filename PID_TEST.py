import control as ctrl
import matplotlib.pyplot as plt
import numpy as np

# Define the transfer function for the system
numerator = [12]
denominator = [30485.1538, 369.6785, 1]
system = ctrl.TransferFunction(numerator, denominator)

# Define PID controller parameters
#Kp = 280
#Ki = 1
#Kd = 1653.9
#2
#Kp = 380
#Ki = 10
#Kd = 3000
#3
Kp = 200 #700
Ki = 10
Kd = 3500

# Create the PID controller
pid_controller = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])

# Create the closed-loop system
closed_loop_system = ctrl.feedback(pid_controller * system )

# Define the time vector for 40 seconds
time = np.linspace(0, 40, 1000)  # 1000 points from 0 to 40 seconds

# Generate the step response
time, response = ctrl.step_response(closed_loop_system, T=time)

# Plot the step response
plt.figure()
plt.plot(time, response)
plt.title('Step Response of Closed-Loop Control System (First 40 Seconds)')
plt.xlabel('Time (seconds)')
plt.ylabel('Response')
plt.xlim(0, 40)  # Set x-axis limit to 40 seconds
plt.grid()
plt.show()