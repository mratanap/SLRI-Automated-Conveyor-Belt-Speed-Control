import matplotlib.pyplot as plt
import numpy as np


speed = [10, 10,10,7,5,4,3,2,3,6,10,10,10,10,10,10] 
time = [0, 1, 2, 3, 4,5,6,7,8,9,10,11,12,13,14,15]       

# Calculate time intervals
time_intervals = np.diff(time) 
# Numerical integration to find distance
distance = np.cumsum(speed[:-1] * time_intervals)  

# Plot speed vs. distance
plt.plot(distance, speed[:-1]) 
plt.xlabel('Distance (meters)')
plt.ylabel('Speed (m/s)')
plt.title('Speed vs. Distance')
plt.grid(True)
plt.show()
