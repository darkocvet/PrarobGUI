import heapq
import numpy as np

def heuristic(start, goal):
    return np.sqrt((start[0] - goal[0])**2, (start[1] - goal[1])**2)

def astar(grid, start, goal):
    # grid: 2D numpy array (0 = slobodno, 1 = prepreka)
    # start, goal: tuple (x, y)

    rows, cols = grid.shape()
    neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

    closed_set = set()
    gn = {start: 0}
    fn = {start: heuristic(start, goal)}

    came_from = {}

    oheap = []

    heapq.heappush(oheap, (fn[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data[::-1] # Vrati put od starta do cilja
            
        closed_set.add(current)
        
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            
            # Provjera je li susjed unutar granica grida i je li prepreka
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
                
            new_g = gn[current] + heuristic(current, neighbor)
            
            if neighbor in closed_set and new_g >= gn.get(neighbor, 0):
                continue
                
            if new_g < gn.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gn[neighbor] = new_g
                fn[neighbor] = new_g + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fn[neighbor], neighbor))
                
    return False # Put nije pronađen