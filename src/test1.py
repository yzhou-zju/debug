import matplotlib.pyplot as plt

# 读取txt文件数据
def read_data_from_txt(file_path):
    y = []
    with open(file_path, 'r') as file:
        for line in file:
            data = float(line.strip())  # 假设每行只有一个数据
            y.append(data)
    x = list(range(1, len(y) + 1))  # 使用行号作为x轴数据
    return x, y

# 绘制曲线图
def plot_curves(x1, y1, x2, y2, label1, label2):
    plt.plot(x1, y1, label=label1)
    plt.plot(x2, y2, label=label2)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Curves')
    plt.legend()
    plt.show()

# 主函数
def main():
    file_path1 = '/home/z/rrt_rotor/src/drone_.txt'  # 第一个txt文件路径
    file_path2 = '/home/z/rrt_rotor/src/drone_1.txt'  # 第二个txt文件路径
    x1, y1 = read_data_from_txt(file_path1)
    x2, y2 = read_data_from_txt(file_path2)
    plot_curves(x1, y1, x2, y2, label1='Group 0  8 16 24 32 40 48 56 64', label2='Group 0  8 20 24 32 40 48 56 64')

if __name__ == '__main__':
    main()