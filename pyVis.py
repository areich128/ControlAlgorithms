import matplotlib
matplotlib.use('Agg')  # Use Agg backend for non-interactive plotting
import matplotlib.pyplot as plt
import os

file_path = '/mnt/c/Users/zhasi/ControlAlgos/output.txt'

# Check if the file exists
if os.path.exists(file_path):
    print("File exists and is accessible")
else:
    print("File not found")
    exit(1)

# Read data from file
data = []
try:
    with open(file_path, 'r') as file:
        for line in file:
            row = line.strip().split(',')
            data.append([float(x) for x in row])
except Exception as e:
    print(f"Error reading file: {e}")
    exit(1)

# Ensure data was read correctly
# print("Data read from file:")
# print(data)

# Transpose data to separate columns
data = list(zip(*data))

# Ensure data transposition is correct
# print("Transposed data:")
# print(data)

# Plot each column
plt.figure()
titles = ['Phi', 'Theta']
for i, (column, title) in enumerate(zip(data, titles)):
    plt.plot(column, label=title)

# Add labels and legend
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Data from output.txt')
plt.legend()
plt.grid(True)

# Save the plot as an image file
plt.savefig('/mnt/c/Users/zhasi/ControlAlgos/output_plot.png')
print("Plot saved as 'output_plot.png'")
