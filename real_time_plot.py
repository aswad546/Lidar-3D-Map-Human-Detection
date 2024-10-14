import serial
import matplotlib.pyplot as plt
import numpy as np

# Setup serial connection (adjust COM port as needed)
ser = serial.Serial('/dev/cu.usbserial-1410', 115200)  # Replace with your port

# Initialize interactive mode for non-blocking plotting
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
line, = ax.plot([], [], marker='o', linestyle='-', color='b')  # Line that will be updated

# Ensure distance axis remains consistent with a max distance of 100 meters
ax.set_ylim([0, 100])

# Setup the polar plot
ax.set_theta_zero_location('N')  # Set 0 degrees at the top
ax.set_theta_direction(-1)       # Set clockwise direction for angles
ax.set_title("Lidar Sweep Data: Distance vs Angle")

# Function to update the plot with new data
def update_plot(angles, distances):
    angles_rad = np.deg2rad(angles)
    line.set_xdata(angles_rad)
    line.set_ydata(distances)
    
    ax.relim()  # Recalculate limits
    ax.autoscale_view()  # Autoscale to fit new data
    plt.draw()  # Redraw the plot
    plt.pause(0.001)  # Pause to allow the plot to update

# Function to read and parse the serial data
def read_sweep_data_from_serial():
    sweep_data = []
    while True:
        line = ser.readline().decode('utf-8').strip()
        # print(f"Raw Serial Line: {line}")  # Debug log of the raw line

        if "Sweep Time" in line:
            print(line)
            if sweep_data:
                # Extract angles and distances from the sweep data
                angles = [entry[1] for entry in sweep_data]
                distances = [entry[2] for entry in sweep_data]

                # Update the plot with new data
                update_plot(angles, distances)
                sweep_data = []  # Clear the data for the next sweep

        else:
            try:
                # Parse labeled format: "Time: XXXX, Angle: XX.XX, Distance: XX.XX"
                parts = line.split(',')
                
                if len(parts) == 3:
                    # Extract and clean each part
                    time_str = parts[0].split(':')[1].strip()  # Extract value after "Time:"
                    angle_str = parts[1].split(':')[1].strip()  # Extract value after "Angle:"
                    distance_str = parts[2].split(':')[1].strip()  # Extract value after "Distance:"
                    
                    # Convert to float
                    time = float(time_str)
                    angle = float(angle_str)
                    distance = float(distance_str)
                    
                    # Append the parsed values to the sweep_data
                    sweep_data.append((time, angle, distance))
                    print(f"Parsed: Time={time}, Angle={angle}, Distance={distance}")

            except (ValueError, IndexError) as e:
                print(f"Error parsing data: {e}")
                print(f"Invalid format: {line}")
                continue

# Call the function to start reading from serial and plot the sweep data
read_sweep_data_from_serial()
