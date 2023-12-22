import matplotlib.pyplot as plt

def plot_curves_from_txt(file_paths):
    try:
        # Determine the number of rows and columns for subplots
        num_plots = len(file_paths)
        num_rows = (num_plots + 3) // 4  # 4 subplots per row
        num_cols = min(num_plots, 4)

        # Create subplots with shared y-axis
        fig, axes = plt.subplots(num_rows, num_cols, figsize=(15, 15), sharey=True)
        fig.suptitle('Curves from TXT Files', fontsize=16)

        # Read data from each file and plot on subplots
        for i, file_path in enumerate(file_paths):
            with open(file_path, 'r') as file:
                y_values = [float(line.strip()) for line in file if line.strip()]

            # Use index as x values
            x_values = list(range(1, len(y_values) + 1))

            # Determine subplot location
            row = i // num_cols
            col = i % num_cols

            # Plot the curve on the corresponding subplot
            axes[row, col].plot(x_values, y_values)
            axes[row, col].set_title(f'Curve {i + 1}')
            axes[row, col].set_xlabel('X-axis Label')
            axes[row, col].set_ylabel('Y-axis Label')

        # Adjust layout
        plt.tight_layout(rect=[0, 0, 1, 0.97])

        # Show the plots
        plt.show()

    except Exception as e:
        print(f"An error occurred: {e}")

# Replace with your actual file paths (16 paths)
file_paths = [
    '/home/zy/debug/1/error_0_x.txt',
    '/home/zy/debug/1/error_1_x.txt',
    '/home/zy/debug/1/error_2_x.txt',
    '/home/zy/debug/1/error_3_x.txt',
    '/home/zy/debug/1/error_4_x.txt',
    '/home/zy/debug/1/error_5_x.txt',
    '/home/zy/debug/1/error_6_x.txt',
    '/home/zy/debug/1/error_7_x.txt',
    '/home/zy/debug/1/error_8_x.txt',
    '/home/zy/debug/1/error_9_x.txt',
    '/home/zy/debug/1/error_10_x.txt',
    '/home/zy/debug/1/error_11_x.txt',
    '/home/zy/debug/1/error_12_x.txt',
    '/home/zy/debug/1/error_13_x.txt',
    '/home/zy/debug/1/error_14_x.txt',
    '/home/zy/debug/1/error_15_x.txt',
    # Add paths for file3.txt to file16.txt
]

plot_curves_from_txt(file_paths)
