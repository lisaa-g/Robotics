# Brendan Griffiths 2426285
# Nihal Ranchod 2427378
# Lisa Godwin 2437980

# Chose Probabilistic Roadmap for following reasons:
# * Simple Configuration Space - The C-Space only has rectangular obstacles, making connectedness a cheap computation
# * Algorithm is easier to write
# * If no path can be found, the graph can be cheaply regenerated
# * Relies on A* for pathfinding, with good heauristic of euclidean distance making it reliable
# * K-Nearest Neigbours is fairly cheap especially with good implementation and only graph sizes

import numpy as np
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import math as m
import queue
from dataclasses import dataclass, field

num_samples = 50
num_neighbours = 4
allowed_navigation_attempts = 10

def two_line_intersection(lineA: np.array, lineB: np.array) -> bool:
    """
        LINE A: [ x0, x1, y0, y1 ]
        LINE B: [ x2, x3, y2, y3 ]

        Politely stolen from http://jeffreythompson.org/collision-detection/line-line.php
    """

    x1 = lineA[0]
    x2 = lineA[1]
    x3 = lineB[0]
    x4 = lineB[1]

    y1 = lineA[2]
    y2 = lineA[3]
    y3 = lineB[2]
    y4 = lineB[3]

    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom != 0:
        directionA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        directionB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
    else:
        directionA = 0
        directionB = 0

    return 0 < directionA and directionA <= 1 and 0 < directionB and directionB <= 1


def line_rect_intersection(line: np.array, rect: np.array) -> bool:
    """
        LINE: [ x0, x1, y0, y1 ]
        RECT: [ x0, x1, y0, y1 ]

        Politely pilfered from http://jeffreythompson.org/collision-detection/line-rect.php
    """

    left = two_line_intersection(line, np.array([ rect[0], rect[0], rect[2], rect[3] ]))
    right = two_line_intersection(line, np.array([ rect[1], rect[1], rect[2], rect[3] ]))
    bottom = two_line_intersection(line, np.array([ rect[0], rect[1], rect[2], rect[2] ]))
    top = two_line_intersection(line, np.array([ rect[0], rect[1], rect[3], rect[3] ]))

    return left or right or bottom or top


class QueueItem:
    h: float
    g: float
    origin_idx: int
    item_idx: int

    def __init__(self, item_idx, origin_idx, h, g):
        self.item_idx = item_idx
        self.origin_idx = origin_idx
        self.h = h
        self.g = g

    def __eq__(self, o):
        return self.item_idx == o.item_idx

    def __lt__(self, o):
        return self.h + self.g < o.h + o.g

    def __hash__(self):
        return hash(self.item_idx)

    def __str__(self):
        return f"Parent {self.origin_idx} -> Point {self.item_idx} | {self.g + self.h} | {self.g} | {self.h}"


def a_star(start: int, goal: int, graph: list[list[int]], points: np.array) -> list[int]:
    goal_p = points[goal]

    def distance(a, b):
        return m.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def heuristic(p):
        return m.sqrt((p[0] - goal_p[0])**2 + (p[1] - goal_p[1])**2)

    frontier = queue.PriorityQueue()
    frontier.put(QueueItem(start, -1, heuristic(points[start]), 0))
    size_frontier = 1
    explored = [None] * len(points)

    while not frontier.empty():
        if size_frontier >= len(points) * 1e4:
            return []

        current = frontier.get()
        size_frontier -= 1

        if explored[current.item_idx] is None:
            explored[current.item_idx] = current
        elif explored[current.item_idx].g + explored[current.item_idx].h > current.g + current.h:
            explored[current.item_idx] = current

        for child in graph[current.item_idx]:

            if child == goal:
                path = [child]

                backtrace = current
                while backtrace.item_idx != start:
                    path.append(backtrace.item_idx)
                    backtrace = explored[backtrace.origin_idx]

                path.append(start)
                path.reverse()
                return path

            frontier.put(QueueItem(child, current.item_idx, heuristic(points[child]), current.g + distance(points[child], points[current.item_idx])))
            size_frontier += 1

    return []


def main():

    start_goal = str(input("start_x,start_y;goal_x,goal_y: ")).split(";")
    start_str = start_goal[0].split(",")
    start = ( int(start_str[0]), int(start_str[1]) )

    end_str = start_goal[1].split(",")
    goal = ( int(end_str[0]), int(end_str[1]) )

    obstacles = []

    input_str = input("Rectangle: ")
    while input_str != "-1":
        input_str = input_str.split(";")
        begin_str = input_str[0].split(",")
        begin = ( int(begin_str[0]), int(begin_str[1]) )

        end_str = input_str[1].split(",")
        end = ( int(end_str[0]), int(end_str[1]) )

        obstacles.append( [begin[0], begin[1], end[0], end[1]] )

        input_str = input("Rectangle: ")

    # start = np.array([10, 10])
    # goal = np.array([80, 30])

    # obstacles = np.array([
    #     [20, 20, 10, 50],
    #     [20, 90, 50, 50],
    #     [30, 40, 30, 40]
    # ])

    fig, ax = plt.subplots()

    for ob in obstacles:
        ax.add_patch(Rectangle((
            ob[0], ob[2]), ob[1] - ob[0], ob[3] - ob[2],
            edgecolor = "black",
            fill = False
        ))

    path = []
    nav_attempts = 0
    while len(path) == 0 and nav_attempts < allowed_navigation_attempts:
        nav_attempts += 1
        sample_points = np.random.uniform(.0, 100., (num_samples, 2))

    # values = np.arange(0, 100, 20)
    # sample_points = np.array([[a, b] for a in values for b in values])

        points_are_valid = np.ones(len(sample_points), dtype=bool)

        for i in range(len(sample_points)):
            sp = sample_points[i]

            invalid_point = False
            for ob in obstacles:
                if ob[0] <= sp[0] and sp[0] <= ob[1] and ob[2] <= sp[1] and sp[1] <= ob[3]:
                    invalid_point = True
                    break

            if invalid_point:
                points_are_valid[i] = False

        valid_points = sample_points[points_are_valid]
        invalid_points = sample_points[np.invert(points_are_valid)]

        valid_points = np.append(valid_points, [start, goal], axis=0)

        knn = NearestNeighbors(n_neighbors=num_neighbours + 1)
        knn.fit(valid_points)
        _, neigbours_mat = knn.kneighbors(valid_points)

        graph = []
        for i in range(len(neigbours_mat)):
            neigbours = neigbours_mat[i, 1:]
            adjacencies = np.zeros(num_neighbours, dtype=bool)

            for j in range(len(neigbours)):
                n = neigbours[j]
                line = np.array([ valid_points[i][0], valid_points[n][0], valid_points[i][1], valid_points[n][1] ])
                no_intersection = True
                for ob in obstacles:
                    no_intersection = no_intersection and not line_rect_intersection(line, ob)
                    if not no_intersection:
                        break

                if no_intersection:
                    adjacencies[j] = True

            graph.append(neigbours[adjacencies])

        path = a_star(len(valid_points) - 2, len(valid_points) - 1, graph, valid_points) 

    if len(path) == 0:
        print("No path could be found")

    for p_i in path:
        print(valid_points[p_i][0], valid_points[p_i][1])

    ax.scatter(valid_points[:, 0], valid_points[:, 1], color="orange")
    ax.scatter(invalid_points[:, 0], invalid_points[:, 1], color="b")
    ax.scatter(start[0], start[1], color="g")
    ax.scatter(goal[0], goal[1], color="r")

    for i in range(len(valid_points)):
        ax.text(valid_points[i][0], valid_points[i][1], str(i))

    for i in range(len(graph)):
        for n in graph[i]:
            line = np.array([valid_points[i][0], valid_points[n][0], valid_points[i][1], valid_points[n][1]])
            ax.plot(line[:2], line[2:4], color="pink")

    for i in range(len(path) - 1):
        line = np.array([valid_points[path[i]][0], valid_points[path[i+1]][0], valid_points[path[i]][1], valid_points[path[i+1]][1]])
        ax.plot(line[:2], line[2:4], color="cyan")

    plt.show()

if __name__ == "__main__":
    main()
