import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean
import networkx as nx
from sklearn.cluster import AgglomerativeClustering

# 五角星形成的全连接图的节点坐标
node_positions = np.array([
    (0, 0), (1, 0.5), (2, 1),
    (3, 0.5), (4, 0),
    (3.5, 1), (3, 2), (3.5, 3),
    (4, 4), (3.5, 4), (3, 4), (2.5, 4.5),
    (2, 5), (1.5, 4.5), (1, 4), (0.5, 4), (0, 4),
    (0.5, 3), (1, 2), (0.5, 1)
])

# 计算节点间的欧式距离作为权重
num_nodes = len(node_positions)
adjacency_matrix = np.zeros((num_nodes, num_nodes))
for i in range(num_nodes):
    for j in range(i + 1, num_nodes):
        distance = euclidean(node_positions[i], node_positions[j])
        adjacency_matrix[i, j] = distance
        adjacency_matrix[j, i] = distance

# 计算度矩阵
degree_matrix = np.diag(np.sum(adjacency_matrix, axis=1))

# 计算拉普拉斯矩阵
laplacian_matrix = degree_matrix - adjacency_matrix

# 计算特征值和特征向量
eigenvalues, eigenvectors = np.linalg.eigh(laplacian_matrix)

# 设置聚类数目为5
num_clusters = 4

# 创建一个大图展示所有结果
plt.figure(figsize=(15, 15))

for i, selected_vector in enumerate(eigenvectors.T):
    # 谱聚类（使用K-Means）
    kmeans = AgglomerativeClustering(n_clusters=num_clusters)
    #kmeans = KMeans(n_clusters=num_clusters, random_state=42)
    clusters = kmeans.fit_predict(selected_vector.reshape(-1, 1))

    # 构建图并设置节点位置
    G = nx.from_numpy_matrix(adjacency_matrix)
    pos = {i: pos for i, pos in enumerate(node_positions)}

    # 子图位置
    plt.subplot(4, 5, i + 1)

    # 可视化聚类结果
    node_colors = plt.cm.get_cmap('viridis', num_clusters)(clusters / (num_clusters - 1))
    nx.draw(G, pos=pos, node_color=node_colors, with_labels=True)
    plt.title(f'Eigenvector {i + 1}')

plt.tight_layout()
plt.show()

