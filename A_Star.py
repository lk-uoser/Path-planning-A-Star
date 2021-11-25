import numpy as np
import matplotlib.pyplot as plt


class Point:
    def __init__(self, x, y, g, h, father):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.father = father
    def getNeighbor(self, mapdata, endx, endy):
        x = self.x
        y = self.y
        result = []
        if x - 1 >= 0 and mapdata[x - 1][y] != 1:
            upNode = Point(x - 1, y, self.g + 10, np.sqrt(abs(x - 1 - endx) ** 2 + abs(y - endy) ** 2) * 10, self)
            result.append(upNode)
        # 下
        if x + 1 <= len(mapdata) - 1 and mapdata[x + 1][y] != 1:
            downNode = Point(x + 1, y, self.g + 10, np.sqrt(abs(x + 1 - endx) ** 2 + abs(y - endy) ** 2) * 10, self)
            result.append(downNode)
        # 左
        if y - 1 >= 0 and mapdata[x][y - 1] != 1:
            leftNode = Point(x, y - 1, self.g + 10, np.sqrt(abs(x - endx) ** 2 + abs(y - 1 - endy) ** 2) * 10, self)
            result.append(leftNode)
        # 右
        if y + 1 <= len(mapdata[0]) - 1 and mapdata[x][y + 1] != 1:
            rightNode = Point(x, y + 1, self.g + 10, np.sqrt(abs(x - endx) ** 2 + abs(y + 1 - endy) ** 2) * 10, self)
            result.append(rightNode)
        # 西北
        if x - 1 >= 0 and y - 1 >= 0 and mapdata[x - 1][y - 1] != 1:
            wnNode = Point(x - 1, y - 1, self.g + 14, np.sqrt(abs(x - 1 - endx) ** 2 + abs(y - 1 - endy) ** 2) * 10,
                           self)
            result.append(wnNode)
        # 东北
        if x - 1 >= 0 and y + 1 <= len(mapdata[0]) - 1 and mapdata[x - 1][y + 1] != 1:
            enNode = Point(x - 1, y + 1, self.g + 14, np.sqrt(abs(x - 1 - endx) ** 2 + abs(y + 1 - endy) ** 2) * 10,
                           self)
            result.append(enNode)
        # 西南
        if x + 1 <= len(mapdata) - 1 and y - 1 >= 0 and mapdata[x + 1][y - 1] != 1:
            wsNode = Point(x + 1, y - 1, self.g + 14, np.sqrt(abs(x + 1 - endx) ** 2 + abs(y - 1 - endy) ** 2) * 10,
                           self)
            result.append(wsNode)
        # 东南
        if x + 1 <= len(mapdata) - 1 and y + 1 <= len(mapdata[0]) - 1 and mapdata[x + 1][y + 1] != 1:
            esNode = Point(x + 1, y + 1, self.g + 14, np.sqrt(abs(x + 1 - endx) ** 2 + abs(y + 1 - endy) ** 2) * 10,
                           self)
            result.append(esNode)
        return result

    def hasNode(self, worklist):
        for i in worklist:
            if i.x == self.x and i.y == self.y:
                return True
        return False

    # 在存在的前提下
    def changeG(self, worklist):
        for i in worklist:
            if i.x == self.x and i.y == self.y:
                if i.g + i.h > self.g + self.h:
                    i.g = self.g
                    i.h = self.h


class Map:
    def __init__(self, array_map):
        self.map = array_map
        self.path = []
        self.len = 0


def obstacles_set(map, start_x, start_y, end_x, end_y):
    map[start_x:end_x, start_y:end_y] = 1


def intable(point: Point, table):
    for i in table:
        if point.x == i.x and point.y == i.y:
            return True
    return False


def Key_Sort(Fn: Point):
    return Fn.g + Fn.h


def A_star(Map, start_point: Point, goal_point: Point):
    if Map.map[goal_point.x][goal_point.y] == 1:
        return None
    open_table = [start_point]
    close_table = []
    while len(open_table) != 0:
        Node = open_table.pop(0)
        close_table.append(Node)
        if Node.x == goal_point.x and Node.y == goal_point.y:
            Map.len = Node.g
            while (Node.father != None):
                Map.path.append((Node.x, Node.y))
                Node = Node.father
            Map.path.append((Node.x, Node.y))
            break
        else:
            workList = Node.getNeighbor(Map.map, goal_point.x, goal_point.y)
            for i in workList:
                if (intable(i, open_table) is True) and (intable(i, close_table) is False):
                    i.changeG(open_table)
                if (intable(i, open_table) is False) and (intable(i, close_table) is False):
                    open_table.append(i)
            open_table.sort(key=Key_Sort)  # 根据f排序


def make_WeightMap(stop_point, pos_map):
    WeightMap = np.zeros((len(stop_point), len(stop_point)))
    WayMap = np.zeros((len(stop_point), len(stop_point))).tolist()
    for i, (sx, sy) in enumerate(stop_point):
        for j, (gx, gy) in enumerate(stop_point):
            tm = Map(pos_map)
            A_star(tm, Point(sx, sy, 0, 0, None), Point(gx, gy, 0, 0, None))
            WeightMap[i][j] = tm.len
            WayMap[i][j] = tm.path
    return WeightMap, WayMap


def visualize(map, waypath, bestpath, stop):
    obstacles_x = []
    obstacles_y = []
    path_x = []
    path_y = []
    stop_x = []
    stop_y = []
    for a in range(len(bestpath)):
        if a == len(bestpath) - 1:
            temp_m = Map(map)
            temp_m.path = waypath[bestpath[a]][bestpath[0]]
            subvisualize(temp_m, a)
        else:
            temp_m = Map(map)
            temp_m.path = waypath[bestpath[a]][bestpath[a + 1]]
            subvisualize(temp_m, a)

    plt.grid(True)
    for i in range(len(map)):
        for j in range(len(map[0])):
            if (map[i][j] == 1):
                obstacles_x.append(j)
                obstacles_y.append(i)
    plt.scatter(obstacles_x, obstacles_y, c='k', s=120, marker='s')
    for a in range(len(bestpath)):
        if (a == len(bestpath) - 1):
            for i, j in waypath[bestpath[a]][bestpath[0]]:
                path_x.append(j)
                path_y.append(i)
            plt.scatter(path_x, path_y, c='r', s=100, marker='s')
        else:
            for i, j in waypath[bestpath[a]][bestpath[a + 1]]:
                path_x.append(j)
                path_y.append(i)
            plt.scatter(path_x, path_y, c='r', s=100, marker='s')
    for i, j in stop:
        stop_x.append(j)
        stop_y.append(i)
    plt.scatter(stop_x, stop_y, c='g', s=100, marker='s')

    x_labels = range(0, len(map[0]), 1)
    y_labels = range(0, len(map), 1)
    # plt.xlim(0, None)
    # plt.ylim(0, None)
    ax = plt.gca()
    ax.xaxis.set_ticks_position('top')
    plt.xticks(x_labels, fontsize=10)
    ax.invert_yaxis()
    plt.yticks(y_labels, fontsize=10)
    ax.set_aspect("equal")
    plt.title("Map", fontsize=12)
    plt.savefig("Map" + ".png")
    plt.show()


def subvisualize(Map: Map, index):
    index = (str)(index)
    obstacles_x = []
    obstacles_y = []
    path_x = []
    path_y = []
    stop_x = []
    stop_y = []
    plt.grid(True)
    for i in range(len(Map.map)):
        for j in range(len(Map.map[0])):
            if (Map.map[i][j] == 1):
                obstacles_x.append(j)
                obstacles_y.append(i)
    plt.scatter(obstacles_x, obstacles_y, c='k', s=120, marker='s')
    for i, j in Map.path:
        path_x.append(j)
        path_y.append(i)
    plt.scatter(path_x, path_y, c='r', s=100, marker='s')

    x_labels = range(0, len(Map.map[0]), 1)
    y_labels = range(0, len(Map.map), 1)
    # plt.xlim(0, None)
    # plt.ylim(0, None)
    ax = plt.gca()
    ax.xaxis.set_ticks_position('top')
    ax.invert_yaxis()
    ax.set_aspect("equal")
    plt.xticks(x_labels, fontsize=10)
    plt.yticks(y_labels, fontsize=10)
    plt.title("Map", fontsize=12)
    plt.savefig("Map" + index + ".png")
    plt.show()


if __name__ == '__main__':
    '''array = np.zeros((10, 10))
    obstacles_set(array, 0, 1, 3, 3)
    obstacles_set(array, 5, 6, 8, 8)
    print(array)
    array = array.tolist()
    m = Map(array)'''
    '''mymap = [
        [0, 1, 1, 0, 1, 0],
        [1, 0, 1, 0, 1, 0],
        [1, 1, 0, 1, 1, 0],
        [1, 1, 1, 0, 1, 0],
        [1, 1, 1, 1, 0, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 0]
    ]'''
    q = np.zeros((20, 20))
    # obstacles_set(q, 0, 1, 4, 4)
    obstacles_set(q, 5, 0, 10, 2)
    obstacles_set(q, 1, 5, 8, 8)
    obstacles_set(q, 3, 10, 16, 20)
    obstacles_set(q, 14, 2, 20, 9)
    # q = q.tolist()
    '''mymap = np.zeros((10, 10))
    obstacles_set(mymap,1,1,4,4)
    obstacles_set(mymap,1,5,8,9)
    mymap = mymap.tolist()'''
    q = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]
    m = Map(q)
    # print(q)

    A_star(m, Point(0, 0, 0, 0, None), Point(2, 1, 0, 0, None))
    if len(m.path) == 0:
        print("There is not a way to access goal or your goal is obstacles")
    else:
        m.path.reverse()
        print(m.path)
        subvisualize(m, 'test')

    m2 = Map(q)
    subvisualize(m2,'Model')
