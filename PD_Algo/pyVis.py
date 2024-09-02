import matplotlib
matplotlib.use('Agg')  # Use Agg backend for non-interactive plotting
import matplotlib.pyplot as plt
import os

# Function to read and transpose data from a file
def read_and_transpose(file_path):
    data = []
    if os.path.exists(file_path):
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    row = line.strip().split(',')
                    data.append([float(x) for x in row])
        except Exception as e:
            print(f"Error reading file {file_path}: {e}")
            exit(1)
    else:
        print(f"File not found: {file_path}")
        exit(1)
    return list(zip(*data))

# File paths
# file_path1 = '/mnt/c/Users/zhasi/ControlAlgos/output.txt'
file_path1 = '/mnt/c/Users/zhasi/ControlAlgos/output1.txt'
file_path2 = '/mnt/c/Users/zhasi/ControlAlgos/output2.txt'
file_path3 = '/mnt/c/Users/zhasi/ControlAlgos/output3.txt'

# Read and transpose data from files
# data = read_and_transpose(file_path1)
data1 = read_and_transpose(file_path1)
data2 = read_and_transpose(file_path2)
data3 = read_and_transpose(file_path3)

# Plot each column
plt.figure()
# plt.plot(data[0], label="response 1")
plt.plot(data1[0], label="z = 1, Wn = 0.5")
plt.plot(data2[0], label="z = 1, Wn = 1")
plt.plot(data3[0], label="z = 1, Wn = 1.5")

# Add labels and legend
plt.xlabel('Time (s * 10)')
plt.ylabel('Degrees from vertical')
plt.title('Responses with varying natural frequencies')
plt.legend()
plt.grid(True)

# Save the plot as an image file
output_path = '/mnt/c/Users/zhasi/ControlAlgos/output_plot.png'
plt.savefig(output_path)
print(f"Plot saved as '{output_path}'")
