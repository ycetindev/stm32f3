import numpy as np
import matplotlib.pyplot as plt

def plot_log_data(file_path):
    data = np.loadtxt(file_path, delimiter=',', skiprows=11) 
    
    time = np.arange(len(data)) 
    time =  data[:, 0]
    current = data[:, 1]
    omega = data[:, 2]
    voltage = data[:, 3]
    
    plt.figure(figsize=(10, 6))
    plt.plot(time, voltage, label='Voltage (V)', color='r')
    plt.plot(time, current, label='Current (A)', color='b')
    plt.plot(time, omega, label='Speed (rad/s)', color='g')
    
    plt.xlabel('Time (secs)')
    plt.title('Captured scope waveforms from DAC outputs')
    plt.legend()
    plt.grid()
    plt.show()

file_path = "scope_waveform.csv"
plot_log_data(file_path)