import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Connect to the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("raspberrypi", 5000))

# Create a figure and axis
fig, ax = plt.subplots()
x_data, y_data, avg_data = [], [], []
line, = ax.plot(x_data, y_data, label='Original Data')
avg_line, = ax.plot(x_data, avg_data, label='Rolling Average', linestyle='--')

# Buffer for rolling average
buffer_size = 60
buffer = []

# Function to update the data
def update_data(new_y):
    x_data.append(len(x_data))  # Incremental x value
    y_data.append(new_y)  # New y value
    buffer.append(new_y)
    
    if len(buffer) > buffer_size:
        buffer.pop(0)
    
    rolling_avg = np.mean(buffer)
    avg_data.append(rolling_avg)

# Function to animate the graph
def animate(i):
    # Get new data (in a real scenario, replace this with your live data source)
    try:
        new_y = float(s.recv(1024).decode("utf-8"))  # Example: random data
    except:
        new_y = y_data[-1] if y_data else 0
    update_data(new_y)
    
    # Update the line data
    line.set_data(x_data, y_data)
    avg_line.set_data(x_data, avg_data)
    
    # Rescale the axis
    ax.relim()
    ax.autoscale_view()
    
    return line, avg_line

# Create an animation object
ani = animation.FuncAnimation(fig, animate, interval=10)  # Update every 100 milliseconds

# Display the plot
plt.legend()
plt.show()