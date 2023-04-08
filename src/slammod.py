import numpy as np
import heapq
import open3d as o3d

# Heuristic function
def heuristic(a, b):
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)


# A* algorithm
def astar(start, goal, obstacles):
    visited = set()
    pq = []
    heapq.heappush(pq, (0, start))
    g = {start: 0}
    parent = {start: None}
    while pq:
        _, current = heapq.heappop(pq)
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]
        visited.add(current)
        neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]), (current[0], current[1]+1), (current[0], current[1]-1),
                     (current[0]+1, current[1]+1), (current[0]-1, current[1]-1), (current[0]-1, current[1]+1), (current[0]+1, current[1]-1)]
        neighbors = [n for n in neighbors if n not in obstacles and n not in visited and n[0] <= 9 and n[0] >= 0 and n[1] <= 9 and n[1] >= 0]
        for neighbor in neighbors:
            cost = g[current] + 1
            if neighbor not in g or cost < g[neighbor]:
                g[neighbor] = cost
                parent[neighbor] = current
                priority = cost + heuristic(neighbor, goal)
                heapq.heappush(pq, (priority, neighbor))
    return None


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("slam1.pcd")
    #o3d.visualization.draw_geometries([pcd])

    downpcd = pcd.voxel_down_sample(voxel_size=0.2)

    #downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    xyz = np.asarray(downpcd)

    #o3d.visualization.draw_geometries([downpcd])

    print(xyz)

    xy = []
    last1 = 0.00000000
    last2 = 0.00000000
    for n in xyz:
        if last1 == n[0] and last2 == n[1]:
            continue
        if n[2] <= 0.1000000:
            xy.append( [n[0], n[1]] ) 
            last1 = n[0]
            last2 = n[1]

    # Run path finding algorithm.
    start = (0, 9)
    end = (0, 0)

    #path = astar(start, end, xy)
    #print(path)
