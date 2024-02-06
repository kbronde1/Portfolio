import numpy as np

# Class to store mesh surface information
class Mesh(object):
    
    def __init__(self, v_coors, v_inds, n_inds):
        """
            
        Parameters
        ----------
        v_coors : numpy array (N_vertices x 3)
            Coordinates of the triangles' vertices
        v_inds : numpy array (N_triangles x 3)
            Indices of the triangles' vertices
        n_inds : numpy array (N_triangles x 3)
            Indices of the (opposite) neighboring triangles' vertices
    
        Returns
        -------
        None.
    
        """
        self.v_coors = v_coors # V
        self.v_inds = v_inds # i
        self.n_inds = n_inds # n

    def getVerticesOfTriangle(self, t_idx):
        '''

        Parameters
        ----------
        t_idx : INTEGER
            triangle's index.

        Returns
        -------
        list
            Each item in the list is one of the 3 vertices of the triangle. 
            Each vertex is its own 1x3 array of xyz coordinates.

        '''
        t_v = self.v_inds[t_idx]
        return [self.v_coors[int(t_v[0])].transpose(), self.v_coors[int(t_v[1])].transpose(), self.v_coors[int(t_v[2])].transpose()]
    
    def calcCentroid(self, vertices):
        '''

        Parameters
        ----------
        vertices : list of 3 1x3 arrays
            3 vertices of the triangle, as outputted by getVerticesOfTriangle().

        Returns
        -------
        1x3 array
            xyz coordinates of the centroid of the provided ttriangle vertices.

        '''
        x = 0
        y = 0
        z = 0
        for i in range(3):
            x += vertices[i][0]
            y += vertices[i][1]
            z += vertices[i][2]
        x /= 3
        y /= 3
        z /= 3
        return np.array([x,y,z])

# The following code is contributed by Prajwal Kandekar, as available on
# GeeksforGeeks and has been modified as necessary to accomodate the Mesh class

# Number of dimensions
k = 3

'''
Using a generic KD tree starter code, I modified it so that the node's point
is the centroid of its triangle and gave it a triangle variable to then hold
the vertices' coordinate information. To find the nearest node, the given point
is compared to each triangle's centroid to find the nearest one, then I call
findClosestPointOnTriangle(...) to calculate the true distance of the point
from the triangle, rather than distance from the centroid.
'''
# A structure to represent node of kd tree
class Node:
	def __init__(self, point, triangle, idx):
            self.point = point # centroid of triangle
            self.triangle = triangle
            self.left = None
            self.right = None
            self.idx = idx

# Inserts a new node and returns root of modified tree
# The parameter depth is used to decide axis of comparison
def insert(root, point, triangle, depth, idx):
    if not root: # Empty tree
        return Node(point, triangle, idx) # Create root

    cd = depth % k # current (axis) dimension

    if point[cd] < root.point[cd]:
        root.left = insert(root.left, point, triangle, depth + 1, idx)
    else:
        root.right = insert(root.right, point, triangle, depth + 1, idx)

    return root

# Euclidean distance function
def distance_squared(point1, point2):
    return sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2))

# Recursive helper function to search a tree for the nearest node to point
def searchTree(root, point, depth, best_node, best_distance):
    if not root or root is None:
        return best_node, best_distance

    cd = depth % k

    if point[cd] < root.point[cd]: # if < root at current dimension
        best_node, best_distance = searchTree(root.left, point, depth + 1, best_node, best_distance)
    else: # if >= root at current dimension
        best_node, best_distance = searchTree(root.right, point, depth + 1, best_node, best_distance)

    current_distance = distance_squared(root.point, point)

    if current_distance < best_distance: # update params if closer node is found
        best_node = root
        best_distance = current_distance
    
    # verify that we made the right tree traversal decision, otherwise update
    other_child = root.right if (point[cd] < root.point[cd]) else root.left
    if other_child:
        other_child_distance = (point[cd] - root.point[cd]) ** 2
        if other_child_distance < best_distance:
            best_node, best_distance = searchTree(other_child, point, depth + 1, best_node, best_distance)

    return best_node, best_distance

# Returns nearest node to given point in root's tree 
def search(root, point):
    nearest_node, __ = searchTree(root, point, 0, None, float('inf'))
    return nearest_node
