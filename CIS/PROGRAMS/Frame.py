import numpy as np
from Point3d import Point3d

class Frame(object):
    
    def __init__(self, name, R, p, neighbors=[]):
        """
            
        Parameters
        ----------
        name : string or character
            Name of the frame (e.g., 'A').
        R : numpy ndarray
            Frame's rotation matrix.
        p : numpy array
            Frame's origin point.
        neighbors : list, optional
            List containing all adjacent Frames. The default is [].
    
        Returns
        -------
        None.
    
        """
        self.name = name
        self.R = R
        self.p = p
        self.neighbors = neighbors
    
    def inverse(self): # F^(-1) = [R^-1, -R^-1 * p]
        invR = np.transpose(self.R) # R^-1 = R^T
        invP_coords = (-1) * (np.transpose(self.R)).dot(self.p.coords)
        invP = Point3d(self.name, invP_coords)
        out = Frame(self.name, invR, invP, self.neighbors)
        return out
    
    def __mul__(self, other): # self * other
        if isinstance(other, Frame): # F_self * F_other
            R = np.matmul(self.R, other.R)
            rotated_p = Point3d(self.name, self.R.dot(other.p.coords))
            p = rotated_p + self.p
            name = self.name+" from "+ other.name
            F_transformation = Frame(name,R,p) # set no neighbors?
            return F_transformation
        elif isinstance(other, Point3d): # F_self * p_other
            return Point3d(self.name, (self.R.dot(other.coords) + self.p.coords))
        else: # check for array of coordinates
            try:
                out = np.empty(other.shape)
                for i in range(3): # loop through rows (x,y,z)
                    out[i,:] = self.R.dot(other[i]) + self.p.coords
            except:
                raise TypeError('Cannot multiply Frame by non-Point object')
    
    def __findFrame__(self,point,path,visited):
        '''

        Parameters
        ----------
        point : Point3d
            Point whose frame we want to find.
        path : list of Frames
            Contains all Frames between self (initial) and point's (target).
            Empty in first iteration.
        visited : list of Frames
            Contains all Frames that have been searched.

        Returns
        -------
        path : list of Frames
            Updated once the target Frame is found.

        '''
        visited.append(self)
        for frame in self.neighbors: # iterate through all neighbors
            if frame not in visited:
                visited.append(frame)
                if frame.name == point.frame: # if target frame is a neighbor
                    path.append(frame)
                    return path
                else: # otherwise search neighbors' children
                    frame.__findFrame__(point, path, visited)
                    if path:
                        path.insert(0,frame) # push to front
            
        return path
    
    def transformPoint(self,point):
        '''

        Parameters
        ----------
        point : Point3d
            Point to be transformed into current (self) frame.

        Returns
        -------
        Point3d
            Original point but with respect to the current (self) frame.

        '''
        # check frame
        if point.__getattr__('frame') == self.name:
            return point
        
        # transform point to current frame
        path = self.__findFrame__(point,[],[])
        ## Since F_BA = F_B^-1 * F_A then don't we only need initial 
        ## and target frame to calculate the new point?
        # point_B = F_BA * point_A
        F = self.inverse() * path[-1]
        new_point = F*point
        print('Point:', new_point.coords, 'in frame:',new_point.frame)
        return new_point
    
    def visualizePoint(self, point):
        p = self.transformPoint(point)
        x = p.__getattr__('x')
        y = p.__getattr__('y')
        z = p.__getattr__('z')
        return x,y,z
    
    def addNeighbor(self, frame):
        self.neighbors.append(frame)
        frame.neighbors.append(self)
