import heapq
import math


def callback(data):
    my_map = data.data

    width = data.info.width # uint
    height = data.info.height # uint 
    resolution = data.info.resolution # float

    rospy.loginfo(str(width) + ' ' + str(height) + ' ' + str(resolution))
    
    #Use A* here
    #Need 2d grid data 
    #Position in x ,y
    #Destination Location in x, y




class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end): #A* algo implementation
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []
    heapq.heappush(open_list, (start_node.f, start_node))

    while len(open_list) > 0:
        current_node = heapq.heappop(open_list)[1]
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # adjacent squares
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[0]) - 1) or node_position[1] < 0:
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:
            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = math.sqrt(((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
            child.f = child.g + child.h

            for open_node in open_list:
                if child == open_node[1] and child.g > open_node[1].g:
                    continue

            heapq.heappush(open_list, (child.f, child))

    return None
    
def listener():
    rospy.init_node('grid_drawer', anonymous=True)
    rospy.Subscriber('map', OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
