import numpy as np
import networkx as nx
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

# 构建星星形状的五角星编队图
G = nx.Graph()
num_agents = 20

# 假设五角星编队的节点连接关系
edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5),
         (5, 6), (6, 7), (7, 8), (8, 9), (9, 10),
         (10, 11), (11, 12), (12, 13), (13, 14), (14, 15),
         (15, 16), (16, 17), (17, 18), (18, 19), (19, 0)]  # 修正这里，最后一个边连接第19个节点和第0个节点
G.add_edges_from(edges)

# 提取节点坐标
phi = (1 + np.sqrt(5)) / 2  # 黄金比例
node_positions = {
    0: (0, 0), 1: (1, 0.5), 2: (2, 1),
    3: (3, 0.5), 4: (4, 0),
    5: (3.5, 1), 6: (3, 2), 7: (3.5, 3),
    8: (4, 4), 9: (3.5, 4), 10: (3, 4), 11: (2.5, 4.5),
    12: (2, 5), 13: (1.5, 4.5), 14: (1, 4), 15: (0.5, 4), 16: (0, 4),
    17: (0.5, 3), 18: (1, 2), 19: (0.5, 1)
}

# 获取节点的坐标值
node_positions_values = np.array(list(node_positions.values()))

# 使用 K-means 进行聚类
n_clusters = 5
kmeans = KMeans(n_clusters=n_clusters, random_state=42)
labels = kmeans.fit_predict(node_positions_values)

# 可视化
plt.figure(figsize=(8, 8))
nx.draw(G, pos=node_positions, node_color=labels, cmap='viridis', with_labels=True, font_weight='bold')
plt.title('K-means Clustering of Star-Shaped Formation')
plt.show()

