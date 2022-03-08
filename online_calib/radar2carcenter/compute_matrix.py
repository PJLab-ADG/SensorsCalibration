
import numpy as np

def rotationMatrixToEulerAngles(R):
    yaw = np.arctan2(R[1][0], R[0][0])
    pitch = np.arctan2(-R[2][0], R[0][0]*np.cos(yaw) + R[1][0] * np.sin(yaw))
    roll = np.arctan2(R[0][2]*np.sin(yaw) - R[1][2]*np.cos(yaw), -R[0][1]*np.sin(yaw)+R[1][1]*np.cos(yaw))
    yaw *= 180.0 / np.pi
    pitch *= 180.0 / np.pi
    roll *= 180.0 / np.pi
    # sy = np.sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0])
    # if (sy > 1e-6):
    #     roll = np.arctan(R[2][1] / R[2][2]) * 180 / np.pi
    #     pitch = np.arctan(-R[2][0] / sy) * 180 / np.pi
    #     yaw = np.arctan(R[1][0] / R[0][0]) * 180 / np.pi
    print ("euler angle:")
    print (yaw, pitch, roll)

def RtoAxisAngle(R):
    tr = np.max((-1, np.min(((R[0][0]+R[1][1]+R[2][2]-1)*0.5, 1))))
    a = np.arccos(tr)

    axis = np.array([R[2][1]-R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1]])
    n2 = np.linalg.norm(axis)
    scale = -a/n2
    axis = scale * axis * 180 / np.pi
    print ("axis:")
    print (axis)

def setCurrentTrans(roll, pitch, yaw):
    roll = roll * np.pi / 180
    roll_mat = np.array([[1,0,0], [0,np.cos(roll),-np.sin(roll)], [0,np.sin(roll),np.cos(roll)]])
    pitch_mat = np.array([[np.cos(pitch),0,np.sin(pitch)], [0,1,0], [-np.sin(pitch),0,np.cos(pitch)]])
    yaw_mat = np.array([[np.cos(yaw),-np.sin(yaw),0], [np.sin(yaw),np.cos(yaw),0], [0,0,1]])
    current_trans = yaw_mat.dot(pitch_mat).dot(roll_mat)
    print ("current_trans:")
    print (current_trans)
    return current_trans

lidar2carcenter = np.array([
                    [
                        0.9999714864759456,
                        0.007518403162870026,
                        -0.0007070116470588245,
                        -0.3397000129853962
                    ],
                    [
                        -0.00752275075414192,
                        0.9999514655581148,
                        -0.006361978133435486,
                        -0.016113036951238856
                    ],
                    [
                        0.0006591454161117209,
                        0.006367115403314981,
                        0.9999795124745113,
                        1.2307225780053024
                    ],
                    [
                        0.0,
                        0.0,
                        0.0,
                        1.0
                    ]
                ])

radar2lidar = np.array([
                    [
                        0.998629,
                        -0.0523359,
                        0,
                        1.87103
                    ],
                    [
                        0.0523359,
                        0.998629,
                        -0,
                        -0.264026
                    ],
                    [
                        0,
                        0,
                        1,
                        -1.18
                    ],
                    [
                        -0,
                        0,
                        -0,
                        1
                    ]
                ])

# lidar2carcenter = np.array([
#                     [
#                         0.9996423984469269,
#                         0.02322392145899537,
#                         0.013256120878590951,
#                         -0.2818166351869025
#                     ],
#                     [
#                         -0.02307671689694978,
#                         0.9996714988769938,
#                         -0.011151659593461849,
#                         0.033728974261904784
#                     ],
#                     [
#                         -0.013510751493972212,
#                         0.010841763993555981,
#                         0.999849946615779,
#                         1.3127425802219055
#                     ],
#                     [
#                         0.0,
#                         0.0,
#                         0.0,
#                         1.0
#                     ]
#                 ])

radar2car  =lidar2carcenter.dot(radar2lidar)

print "lidar2carcenter"
print lidar2carcenter
rotationMatrixToEulerAngles(lidar2carcenter)
print "radar2lidar"
print radar2lidar
rotationMatrixToEulerAngles(radar2lidar)
print "radar2car"
print radar2car
rotationMatrixToEulerAngles(radar2car)
RtoAxisAngle(radar2car)