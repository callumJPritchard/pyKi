# pyKi
### - a simple python kinematics library

 - [x] create custom chains
 - [x] forward kinematics position
 - [x] forward kinematics rotation *
 - [x] inverse kinematics position
 - [x] inverse kinematics rotation *
 - [ ] better docs

*rotations are relative to the starting pose of the chain

## How to use
setup

    from pyKi import Chain, Link
    
    # create a chain of 6 joints
    chain = Chain([
	    # relative position, rotation axis, joint limits in rad
	    # use negative axis indices if joints rotate in reverse
	    Link([0,0,0], [0,0,-1], bounds=(-3.14,3.14)), 
	    Link([0.05,0,0], [0,1,0], bounds=(-3.14,3.14)),
	    Link([0,0,0.6], [0,-1,0], bounds=(-3.14,3.14)), 
	    Link([0,0,0.2], [1,0,0], bounds=(-3.14,3.14)),
	    Link([0.7,0,0], [0,1,0], bounds=(-3.14,3.14)),
	    Link([0.07,0,0], [1,0,0], bounds=(-3.14,3.14))
    ])
    
	#get transformation matrix for a set of angles (in rad)
	transMat = chain.forward_kinematics([0,0,0,0,0,0])

making use of the returned matrix:

    xyz = transMat[:3,3]
    from scipy.spatial.transform import Rotation as R
    rotMat = transMat[:3,:3]
    r1 = R.from_dcm(rotMat)
	# change the rotation order as you require
    rotation = r1.as_euler("xyz", degrees=True)
    # remember this will be relative to the starting rotation

using inverse kinematics:

    # lets use our existing matrix as the target pose
    # pass False if you want to ignore orientation
    res = chain.inverse_kinematics(transMat, True)
    joint angles = res.angles
