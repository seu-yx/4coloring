import cv2
import numpy as np
import sys
import matplotlib as plt


try:
    MAP_IMAGE_PATH = sys.argv[1]
except IndexError:
    print("Error: please specify an image.")
    exit(0)

image = cv2.imread(MAP_IMAGE_PATH, cv2.IMREAD_COLOR)
height = len(image)
width = len(image[0])

DX = [-1, +1, 0, 0]
DY = [0, 0, -1, +1]
# 颜色表
COLORING_COLORS = [(150, 0, 0), (0, 150, 0), (0, 0, 150), (0, 150, 150)]

# 像素所属区域号 mark[y][x] = 区域号
mark = [[-1 for i in range(width)] for j in range(height)]

# 区域i像素集合(x,y) region[i] = [(x,y)...]
regions = [[] for i in range(50)]

# 区域i边界像素集合(x,y)  regions_border[i] = [(x,y)...]
regions_border = [[] for i in range(50)]

# 区域号，结点 nodes[i] = Node(i, x, y)
nodes = []

# 结点颜色
nodes_color = [-1 for i in range(50)]

class Node:
    def __init__(self, node_id, node_x, node_y):
        self.id = node_id
        self.x = node_x
        self.y = node_y
        self.adj = []

    def add_edge(self, node):
        self.adj.append(node.id)


def same_pixel_colors(image, x1, y1, x2, y2):
    if x1 < 0 or x1 >= width or y1 < 0 or y1 >= height or x2 < 0 or x2 >= width or y2 < 0 or y2 >= height:
        return False
    b1, g1, r1 = image[y1][x1]
    b2, g2, r2 = image[y2][x2]
    r1, g1, b1 = int(r1), int(g1), int(b1)
    r2, g2, b2 = int(r2), int(g2), int(b2)
    diff = abs(r1 - r2) + abs(g1 - g2) + abs(b1 - b2)
    return diff <= 100

# 区域涂色
def color_region(idx, color):
    region_idx = mark[nodes[idx].y][nodes[idx].x]
    for j in range(len(regions[region_idx])):
        x = regions[region_idx][j][0]
        y = regions[region_idx][j][1]
        image[y][x] = color
    cv2.imshow('4color', image)
    key = cv2.waitKey(1000)
    if key == 27:
        cv2.destroyAllWindows()
        exit()
    return

def colorize_map(node_index):
    # 终止
    if node_index == len(nodes):
        key = cv2.waitKey(0)
        if key == 27:
            cv2.destroyAllWindows()
            exit()
        return
    for i in range(len(COLORING_COLORS)):
        is_color_valid = True
        # 遍历相邻结点，判断该颜色是否可用，可用则执行着色，否则查询下一个颜色
        for u in nodes[node_index].adj:
            if nodes_color[u] == i:
                is_color_valid = False
                break
        if is_color_valid:
            nodes_color[node_index] = i
            color_region(node_index, COLORING_COLORS[i])
            colorize_map(node_index + 1) # 递归下一个
            nodes_color[node_index] = -1 # 回溯
            color_region(node_index, (255, 255, 255)) # 涂成白色


if __name__=="__main__":
    # 识别地图轮廓
    print("----正在识别地图轮廓----")
    for y in range(height):
        for x in range(width):
            # 黑色像素视为地图轮廓，跳过
            if np.all(image[y][x]<10):
                continue
            if mark[y][x] == -1:
                queue = [(x, y)]
                mark[y][x] = len(nodes)
                while queue:
                    x1, y1 = queue.pop(0)
                    regions[len(nodes)].append((x1, y1))
                    mark[y1][x1] = len(nodes)
                    # 向四个方向查询，将区域内像素加入，否则将当前像素标记为边界
                    for k in range(4):
                        x2 = x1 + DX[k]
                        y2 = y1 + DY[k]
                        if x2 >= 0 and x2 < width and y2 >= 0 and y2 < height and mark[y2][x2] == -1:
                            if same_pixel_colors(image, x1, y1, x2, y2):
                                mark[y2][x2] = len(nodes)
                                queue.append((x2, y2))
                            else:
                                regions_border[len(nodes)].append((x1, y1))
                # 当前区域增长结束，添加区域号和区域坐标
                nodes.append(Node(len(nodes), x, y))

    # 添加地图的边关系
    print("----正在添加边缘关系----")
    # 0为外围，本实验不考虑
    for i in range(1, len(nodes)):
        for j in range(1, len(nodes)):
            if i == j:
                nodes[i].add_edge(nodes[j])
                continue
            start_x, start_y = nodes[i].x, nodes[i].y
            end_x, end_y = nodes[j].x, nodes[j].y
            sx, sy, ex, ey = 0, 0, 0, 0
            min_distance_sqr = 1e10
            # i区域边界像素坐标 u(x,y), j区域边界像素坐标 v(x,y)
            for u in regions_border[mark[start_y][start_x]]:
                for v in regions_border[mark[end_y][end_x]]:
                    # distance_sqr = (x1-x2)^2 + (y1-y2)^2 通过边界判断相邻
                    tmp_distance_sqr = (u[0] - v[0]) * (u[0] - v[0]) + (u[1] - v[1]) * (u[1] - v[1])
                    if tmp_distance_sqr < min_distance_sqr:
                        min_distance_sqr = tmp_distance_sqr
                        sx, sy = u[0], u[1]
                        ex, ey = v[0], v[1]
            dx, dy = ex - sx, ey - sy
            if abs(dx) + abs(dy) < 10:
                nodes[i].add_edge(nodes[j])
                nodes[j].add_edge(nodes[i])

    print("----边缘添加完成----")
    input("回车执行着色")
    print("----正在运行图着色算法----")
    # 填充颜色，0为外边界，从1开始
    colorize_map(1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()




