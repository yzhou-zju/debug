import matplotlib.pyplot as plt

def plot_curve_from_txt(file_path):
    try:
        # 打开文件并读取数据
        with open(file_path, 'r') as file:
            # 读取所有行并将其转换为浮点数列表
            y_values = [float(line.strip()) for line in file]

        # 使用索引作为 x 值
        x_values = list(range(1, len(y_values) + 1))

        # 绘制曲线图
        plt.plot(x_values, y_values, label='Curve')

        # 设置图表标题和标签
        plt.title('Curve from TXT File')
        plt.xlabel('X-axis Label')
        plt.ylabel('Y-axis Label')

        # 显示图例
        plt.legend()

        # 显示图表
        plt.show()

    except Exception as e:
        print(f"An error occurred: {e}")

# 请替换为你的实际文件路径
file_path = '/home/zy/debug/1/error_0.txt'
plot_curve_from_txt(file_path)
