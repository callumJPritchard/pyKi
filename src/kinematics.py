import numpy as np
import scipy.optimize
class kinematicsResponse:
    def __init__(self, angles, error, optRes):
        self.angles = angles
        self.error = error
        self.optimizeResult = optRes
    
class Link:
    def __init__(self, translation, rotation, bounds=(-999,999)):
        self.translation = translation[:]
        self.rotation = rotation[:]
        self.bounds = bounds
        
    def getTransMatrix(self, angle): #add the translation part of the matrix to the rotation matrix
        mat = getRotMat(self.rotation[0], self.rotation[1], self.rotation[2], angle)
        mat[0][3] = self.translation[0]
        mat[1][3] = self.translation[1]
        mat[2][3] = self.translation[2]
        return mat

class Chain:
    def __init__(self, links):
        self.linkArray = links[:]
        self.bounds = []
        for l in self.linkArray:
            self.bounds.append(l.bounds)
            
    def forward_kinematics(self, jointAngles):            
        frameMatrix = np.eye(4) #start with identity matrix
        for index, (link, jAngle) in enumerate(zip(self.linkArray, jointAngles)):
            frameMatrix = np.dot(frameMatrix, link.getTransMatrix(jAngle)) #apply the affine matrices in order
        return frameMatrix #return the completed matrix
    
    def optimizeFunc(self, x, target, orientation):
        return getPositionError(target, self.forward_kinematics(x), orientation=orientation)
    
    def inverse_kinematics(self, target, orientation):
        res = scipy.optimize.minimize(self.optimizeFunc, [0]*len(self.linkArray), args=(target, False), method='SLSQP', bounds=self.bounds)
        if orientation==False:
            return kinematicsResponse(res.x, res.fun, res)
        res = scipy.optimize.minimize(self.optimizeFunc, res.x, args=(target, True), method='SLSQP', bounds=self.bounds)
        return kinematicsResponse(res.x, res.fun, res)
    
def getRotMat(rX, rY, rZ, angle): #convert rotationa; axis in format [0,1,0] or [-1,0,0] to a rotation matrix
    ang = angle + 0
    if rX != 0:
        if rX < 0:
            angle *= -1
        return np.array([
            [1., 0., 0., 0.],
            [0., np.cos(angle), -np.sin(angle), 0.],
            [0., np.sin(angle), np.cos(angle), 0.],
            [0., 0., 0., 1]
        ])
    if rY != 0:
        if rY < 0:
            angle *= -1
        return np.array([
            [np.cos(angle), 0., np.sin(angle), 0.],
            [0., 1., 0., 0.],
            [-np.sin(angle), 0., np.cos(angle), 0.],
            [0., 0., 0., 1]
        ])
    if rZ != 0:
        if rZ < 0:
            angle *= -1
        return np.array([
            [np.cos(angle), -np.sin(angle), 0., 0.],
            [np.sin(angle), np.cos(angle), 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1]
        ])
    
def getPositionError(target, current, orientation):
    originDist = np.linalg.norm(target[:3,3] - current[:3,3]) **2 #distance of tcp target to current
    if orientation == False:
        return originDist #return squared distance if we arent doing orientation
    #use virtual points distance as rotations may be equivalent - use one for each axis to ensure equal weight
    targVirtX = np.dot(target, [1,0,0,1])
    currVirtX = np.dot(current, [1,0,0,1])
    targVirtY = np.dot(target, [0,1,0,1])
    currVirtY = np.dot(current, [0,1,0,1])
    targVirtZ = np.dot(target, [0,0,1,1])
    currVirtZ = np.dot(current, [0,0,1,1])
    
    virtXdist = np.linalg.norm(targVirtX - currVirtX) **2
    virtYdist = np.linalg.norm(targVirtY - currVirtY) **2
    virtZdist = np.linalg.norm(targVirtZ - currVirtZ) **2
    #return error
    ret = virtYdist + virtXdist + virtZdist
    #print(ret)
    return ret