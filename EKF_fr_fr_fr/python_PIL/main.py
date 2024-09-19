import serial
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque

# Adjust 'COM3' to the serial port your Arduino is connected to
ser = serial.Serial('/dev/tty.usbmodem101', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# Global variables to store data
times = deque()
altitudes = deque()
velocities = deque()

real_altitudes = deque()
real_velocities = deque()


initial_velocity = 9.81*20
step_time = 0.05


def send_data():

    z_meas = 0
    z_real = 0
    current_velocity = initial_velocity

    start_time = time.time()
    total_time = 25  # Total simulation time in seconds
    while True:
        elapsed_time = time.time() - start_time

        if elapsed_time > total_time:
            break

        a_meas = - 9.81
        current_velocity = current_velocity + a_meas * step_time

        ds = max(0, (current_velocity*step_time))

        z_meas += ds
        z_real += ds

        real_altitudes.append(z_real)
        real_velocities.append(current_velocity)
                
                
        a_meas += np.random.normal(0, 3)  # Add relative noise
        z_meas += np.random.normal(0, 3)  # Add relative noise

        # add disturbance
        if elapsed_time > 4 and elapsed_time < 6:
            z_meas += 1

        if elapsed_time > 6 and elapsed_time < 8:
            z_meas -= 1

        # Send data to Arduino
        data_string = f"{z_meas},{a_meas}\n"
        ser.write(data_string.encode('utf-8'))

        # Wait before sending the next data point
        time.sleep(step_time)  # 20 Hz data rate

def read_data():
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                altitude_str, velocity_str = line.split(',')
                altitude = float(altitude_str)
                velocity = float(velocity_str)

                times.append(time.time())
                altitudes.append(altitude)
                velocities.append(velocity)
            except ValueError:
                pass  # Ignore any lines that can't be parsed

# Start threads for sending and receiving data
sender_thread = threading.Thread(target=send_data)
receiver_thread = threading.Thread(target=read_data)
sender_thread.start()
receiver_thread.start()

# Set up real-time plotting
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))  # Adjust figsize to make the plot bigger
# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))  # Adjust figsize to make the plot bigger

def animate(i):
    if len(times) > 0:
        N = 2000  # Number of data points to display
        times_list = list(times)[-N:]
        altitudes_list = list(altitudes)[-N:]
        real_altitudes_list = list(real_altitudes)[:-1]
        real_velocities_list = list(real_velocities)[:-1]

        error_altitudes = [a - b for a, b in zip(altitudes_list, real_altitudes_list)]

        velocities_list = list(velocities)[-N:]

        t0 = times_list[0]
        times_plot = [t - t0 for t in times_list]

        ax1.clear()
        ax2.clear()

        ax1.plot(times_plot, altitudes_list, 'b')
        # ax1.plot(times_plot, error_altitudes, 'b')
        
        # add real altitudes
        ax1.plot(times_plot, real_altitudes_list, 'r--')
        ax1.set_ylabel('Altitude (m)')
        ax1.set_title('Estimated Altitude')

        ax2.plot(times_plot, velocities_list)
        # add real velocities
        ax2.plot(times_plot, real_velocities_list, 'r--')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_xlabel('Time (s)')
        ax2.set_title('Estimated Velocity')

ani = animation.FuncAnimation(fig, animate, interval=2000)
plt.show()

# Close threads and serial port after plotting
sender_thread.join()
receiver_thread.join()
ser.close()