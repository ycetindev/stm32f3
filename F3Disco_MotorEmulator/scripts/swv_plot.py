import numpy as np
import matplotlib.pyplot as plt

def plot_log_data(file_path):
    data = np.loadtxt(file_path, delimiter=',', skiprows=2) 
    
    time = np.arange(len(data))
    time = data[:, 0] * (1/10000) # logging at 10kHz
    voltage = np.ones(len(data))
    voltage[0] = 0
    current = data[:, 1]
    omega = data[:, 2]
    
    plt.figure(figsize=(10, 6))
    plt.plot(time, voltage, label='Voltage (V)', color='r')
    plt.plot(time, current, label='Current (A)', color='b')
    plt.plot(time, omega, label='Speed (rad/s)', color='g')
    
    plt.xlabel('Time (secs)')
    plt.title('Motor model states (logged at 10kHz)')
    plt.legend()
    plt.grid()
    plt.show()

file_path = "emulator_verify_final_10kHz.txt"  
plot_log_data(file_path)