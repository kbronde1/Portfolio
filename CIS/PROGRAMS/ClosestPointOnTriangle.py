import numpy as np

# Helper function for findClosestPointOnTriangle(...)
# Returns closest point (c) on the line segment between x and y
def projectOnSegment(c, x, y):
    # find closest point on the edge of triangle
    c = c.transpose()
    l = (((c - x).dot(y - x))/((y - x).dot(y - x)))[0]
    l_seg = max(0, min(l, 1))
    c = x + l_seg*(y - x)
    return c

def findClosestPointOnTriangle(a, v_coors):
    '''

    Parameters
    ----------
    a : array of point coordinates
        xyz coordinates of A body w.r.t. tracker coordinates for N_s samples
    v_coors : NumPy array of 3 points
        ([x, y z], [x, y, z], [x, y, z])
        Coordinates of the triangles' vertices

    Returns
    -------
    NumPy array (1 x 3)
        xyz coordinates of the closest point on the given triangle.

    '''
    p, q, r = v_coors
    b = (a - p).transpose()[..., np.newaxis]
    A = np.array([q-p, r-p]).transpose()
    x = np.linalg.inv(A.transpose() @ A) @ (A.transpose() @ b)
    l = x[0][0]
    u = x[1][0]
    c = p + l*(q - p) + u*(r - p)
    c = c.transpose()[..., np.newaxis]
    if l >= 0 and u >= 0 and l + u <= 1:
        # closest point is within the triangle
        return c[:,0]
    elif l < 0:
        return projectOnSegment(c, r, p)
    elif u < 0:
        return projectOnSegment(c, p, q)
    elif l + u > 0:
        return projectOnSegment(c, q, r)
