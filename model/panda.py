import numpy as np

# Load the .txt file
filename = '../maps/output/heightmap_z_values.txt'  # Replace with your file path

# Assuming the data is tab or space-delimited
data = np.loadtxt(filename)

# Check the dimensions
rows, columns = data.shape
print(f"Rows: {rows}, Columns: {columns}")
