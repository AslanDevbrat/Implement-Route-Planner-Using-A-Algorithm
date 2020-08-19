import math
import heapq


"""
    This Implement the A* algorithm to find the shortest and cheapest distance
    between two nodes.
"""

def shortest_path(M,start,goal):
    #print('//////////////////////////////////////////////////////////////////')
    heuristic_dict = dict()
    """
        This fuction calculate distance between two points.
        params  maps : The whole input map
        params start: Origin point
        returns: dictionary containing distance from the origin node 
                      eg:{1:0.98, 2 : 3.23, 3 :0} where key is the node and 
                      value is the distace to the origin node. 3 is the origin
                      therfore value is 0
    """
    def get_heuristic(maps,node, goal):
        
        if node in heuristic_dict:
            return heuristic_dict[node]
        

        # x co-ordinate of origin
        x1 = maps.intersections[goal][0]

        # y co-ordinate of origin
        y1 = maps.intersections[goal][1]
        
        x2 = maps.intersections[node][0]
        y2 = maps.intersections[node][1]
        heuristic_dict[node] = math.hypot(x2 - x1, y2 - y1)
        return heuristic_dict[node]
    
  
    
    # this stores the distances
    distance_dict = {}
    distance_dict[start] = 0
    
    # keep track of the path to the goal
    path = {}
    
    minHeap = []
    heapq.heappush(minHeap,(float('inf'),0,start))
    
    frontier = set()
    frontier.add(start)
    
    explored =set()
    
    unexplored = set()
    unexplored.update(node for node in M.intersections)

    
    """I have made a minimun Heap with (f, distance from source node, node )
        Where f = g + h as in A* algorithm
    """
    
    while len(frontier)>0:
        
        """ extrating from minHeap with least f = g+h value"""
        function_value,node_distance, current_node = heapq.heappop(minHeap)
        
        if current_node not in unexplored:
            continue
        
        
        """
            frontier: keep trak of frontier nodes
            explored: keep track of explored nodes
            unexplored: kepp trak of unexplored nodes
        """
        frontier.remove(current_node)
        explored.add(current_node)
        unexplored.remove(current_node)
        
        #print('dictionary',heuristic_dict)
        """When you hits the Bull's eye"""
        if current_node == goal:
            #print('found')
            break
            
            
        for road in M.roads[current_node]:                     

            
            if road in unexplored:
                frontier.add(road)
                distance_between_node = get_distance(M,road,current_node)
                current_f =node_distance+ distance_between_node+get_heuristic(M,road,goal)

                
                if road not in distance_dict:
                    distance_dict[road] = float('inf')
               
                if distance_dict[road]>current_f:
                    distance_dict[road] = current_f
                    heapq.heappush(minHeap,(current_f,node_distance+distance_between_node,road))
                   
                    path[road] = current_node

    
    temp = goal
    final =[]
    
    # extracting path from nodes that have been seen

    while temp is not start:
        #print(path[temp])
        temp = path[temp]
        final.insert(0,temp)
    final.append(goal)
    #print('final',final)
   
    return final
            

"""
    This return the distance between two nodes
"""
def get_distance(M,nodeA,nodeB):
    return math.hypot(M.intersections[nodeB][0] - M.intersections[nodeA][0], 
                      M.intersections[nodeB][1] - M.intersections[nodeA][1])





