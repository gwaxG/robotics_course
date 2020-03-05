import rospy, time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray

'''
rostopic pub /points std_msgs/Int32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
- 250
- 250
- 270
- 210 " 

'''

class Finder:
    def __init__(self):
        rospy.init_node("path_finder")
        self.map = []
        self.map_init = []
        self.width = 0
        self.height = 0
        self.visit = []
        self.parent = []
        self.finished = True
        self.cost = []  
        self.points = []
        self.map2pub = OccupancyGrid()
        self.c = 8 # rospy.get_param("connectivity")
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        rospy.Subscriber("/points", Int32MultiArray, self.update_points)
        self.pub_map = rospy.Publisher("/map", OccupancyGrid)
        rospy.spin()
        
       
    def update_map(self, msg):
        print('Map received')
        self.map_init = msg.data
        self.map2pub.info = msg.info
        self.map2pub.data = msg.data
        if self.width == 0:
            self.width = msg.info.width
            self.height = msg.info.height
        if self.map != msg.data:
            self.map = list(msg.data)
        if self.points != [] and self.finished:
            self.find_path()
        
    def update_points(self, msg):
        print('Points received', msg.data)
        if self.points != msg.data:
            self.points = msg.data
        if self.map != []:
            self.find_path()
        
            
    def find_path(self):
        self.finished = False	
        INF = 2000
        f = lambda p: p[1]*self.width + p[0]
        if self.c == 4: 
            print('Mask 4')
            mask = lambda p: [
                [p[0]-1, p[1]], 
                [p[0], p[1]-1], 
                [p[0]+1, p[1]],
                [p[0], p[1]+1]
            ]
        elif self.c == 8: 
            print('Mask 8')
            mask = lambda p: [
                [p[0]-1, p[1]], 
                [p[0], p[1]-1], 
                [p[0]+1, p[1]],
                [p[0], p[1]+1],
                [p[0]-1, p[1]-1], 
                [p[0]+1, p[1]+1], 
                [p[0]+1, p[1]-1],
                [p[0]-1, p[1]+1]
            ]
        start = [self.points[0], self.points[1]]
        goal = [self.points[2], self.points[3]]
        self.map[f(start)] = 80
        self.map[f(goal)] = 80
        queue = [start]
        self.visit = [False for i in range(self.width*self.height)]
        self.cost = [INF for i in range(self.width*self.height)]
        self.parent = [[-1,-1] for i in range(self.width*self.height)]
        self.visit[f(start)] = True
        self.cost[f(start)] = 0
        self.parent[f(start)] = [start]
        self.parent[f(goal)] = [goal]
        cnt = 0
        print('Start at traversability', self.map[f(start)])
        print('Goal at traversability', self.map[f(goal)])
        if self.map[f(start)] == 100 or self.map[f(goal)] == 100:
            exit()
        while True:
            if len(queue) == 0:
                print('No path')
                break
            current = queue.pop(0)
            added_nodes = 0
            for neighbor in mask(current):
                node = f(neighbor)
                if self.visit[node] == False and node not in queue and self.map[node] != 100 and self.map[node] != -1:
                    added_nodes += 1
                    self.visit[node] = True
                    self.parent[node] = current
                    queue.append(neighbor)
                    self.map[node] = 20
            # print('Number of added nodes to the queue', added_nodes)
            if current == start:
                continue
            else:
                fnode = f(current)
                self.visit[fnode] = True
                self.cost[fnode] = self.cost[f(self.parent[fnode])]+1
                self.map[fnode] = 50
                cnt += 1
                if cnt == 2000:
                    cnt = 0
                    print('Publish map')
                    self.map2pub.data = self.map
                    self.pub_map.publish(self.map2pub)
                if current == goal:
                    print("At the goal")
                    break
        path = [goal]
        BFS = False
        while True:
            current = path[-1]
            if current == start:
                break
            if BFS:
                path += [self.parent[f(current)]]
                self.map[f(current)] = 100
            else:
                # Dijkstra
                ns = mask(current)
                min_cost = min([self.cost[f(n)] for n in ns])
                for n in ns:
                    if self.cost[f(n)] == min_cost:
                        self.map[f(n)] = 100
                        path += [n]   
                        break                 
        self.map2pub.data = self.map
        self.pub_map.publish(self.map2pub)
        print('Found path', path)
        print('Finished')
        self.finished = True
        time.sleep(5)

            
            
if __name__ == '__main__':
    Finder()

