import rospy
from path_planning.srv import a_star, a_starResponse


import numpy as np






def callback(request):
    """ This is being used
    world = np.array([
        [0, 0, 0, 0, 1],
        [1, 1, 0, 0, 1],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0]
    ])
    """
    world = np.load("./src/path_planning/world/world.npy")
    start = tuple(np.array(request.src))
    goal = tuple(np.array(request.dest))
    path = astar(world, start, goal)
    
    return a_starResponse(np.array(path).flatten())
    

def astar(world, start, goal):
    # Define the heuristic function (Euclidean distance)
    def heuristic(a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    # Define the cost function (Euclidean distance)
    def cost(a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    # Define the get_neighbors function (4-connectivity)
    def get_neighbors(node):
        neighbors = []
        if node[0] > 0 and world[node[0]-1][node[1]] == 0:
            neighbors.append((node[0]-1, node[1]))
        if node[1] > 0 and world[node[0]][node[1]-1] == 0:
            neighbors.append((node[0], node[1]-1))
        if node[0] < world.shape[0]-1 and world[node[0]+1][node[1]] == 0:
            neighbors.append((node[0]+1, node[1]))
        if node[1] < world.shape[1]-1 and world[node[0]][node[1]+1] == 0:
            neighbors.append((node[0], node[1]+1))
        return neighbors

    # Define the main function
    closed_set = set()
    open_set = {start}
    came_from = {}

    g_score = {start: 0}
    h_score = {start: heuristic(start, goal)}
    f_score = {start: h_score[start]}

    while open_set:
        current = min(open_set, key=lambda node: f_score[node])

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in get_neighbors(current):
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + cost(current, neighbor)

            if neighbor not in open_set:
                open_set.add(neighbor)
            elif tentative_g_score >= g_score[neighbor]:
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            h_score[neighbor] = heuristic(neighbor, goal)
            f_score[neighbor] = g_score[neighbor] + h_score[neighbor]

    return None

def heuristic(node, goal):
    # Use the Manhattan distance as the heuristic
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])


def a_star_service():
    rospy.init_node('a_star')
    rospy.Service("a_star", a_star, callback)
    world = np.load("./src/path_planning/world/world.npy")
    rospy.loginfo("Map of the world\n" + str(world))
    rospy.spin()




if __name__ == '__main__':
    a_star_service()
