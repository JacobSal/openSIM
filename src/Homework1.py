#%% Imports
import opensim as osim
from os.path import join, abspath, dirname
import numpy as np
from rotateDef import r1, r2, r3, e123_dcm, getRotationMatrix
from vector3d.vector import Vector
from scipy.spatial.transform import Rotation
#%% Path
global cfpath, omArm26Dir
cfpath = dirname(__file__)
omArm26Dir = join(cfpath,"Models","Arm26")

#%% Definitions
# convert degrees to radians
def d2r(degrees):
    return degrees * (np.pi / 180)
# get axis unit vector from dcm rows

def get_vector_from_dcm(dcm, axis, o, label, color):
    """
    dcm: direction cosine matrix (3x3 numpy array)
    axis: basis vector to extract (0 = x, 1 = y, 2 = z)
    """
    return Vector(dcm[axis, :][0], dcm[axis, :][1], dcm[axis, :][2], origin=o, text=label, color=color)

def formatTrans(state,frame,tmpF):
    tmpMat = frame.findTransformBetween(state,tmpF)
    tmpp = tmpMat.p().to_numpy()
    tmpT = tmpMat.T().to_numpy()
    tmpR = tmpMat.R().toString()
    print(f"{frame.getName()}-{tmpF.getName()}:")
    print(f"position: {tmpp}")
    print(f"Translation: {tmpT}")
    print(f"Rotation: {tmpR}")
    return tmpMat

#%% MAIN
if __name__ == '__main__':
    # Load a Model from file
    myModel = osim.Model(join(omArm26Dir,"arm26.osim"))
    # states_sto_fname = join()
    state = myModel.initSystem()
    # myModel.getWorkingState()
    bodies = []
    val = [30, 20] #r_shoulder_elev, r_elbow_flexion
    count = 0
    # states = osim.StatesTrajectory()
    for cord in myModel.getCoordinateSet():
        print(cord.getName())
        print(f"current value: {cord.getValue(state)}")
        cord.setValue(state,val[count])
        print(f"new value: {cord.getValue(state)}")
        count += 1
    #endfor
    positions = []
    for body in myModel.getBodyList():
        print(body.getName())
        print(f"inertia: {body.get_inertia()}")
        print(f"mass center: {body.get_mass_center()}")
        print(f"mass: {body.get_mass()}")
        pp = body.getPositionInGround(state)
        print(f"current position: {pp}")
        positions.append(pp.to_numpy())
        bodies.append(body)
        print("\n")
    #endfor
    vec3 = []
    count = 0
    tmpF = osim.Frame
    for f in bodies:
        body = f
        vec3.append(body.getName())
        if len(vec3)>1:
            formatTrans(state,body,tmpF)
            formatTrans(state,tmpF,body)
            # transMat.append((f"{f[0]}-{tmpF.getName()}",tmpT,tmpR))
        #endif
        print("\n")        
        tmpF = body
        count += 1
    #endfor
    frames = []
    tmpF = osim.Frame
    frameNames = ['r_ulna_radius_hand_offset','r_humerus_offset','base_offset']
    for frame in myModel.getFrameList():
        frames.append(frame)
    #endfor
    for mark in myModel.getMarkerSet():
        print(mark.getName())
        print(f"current location: {mark.get_location().to_numpy()}")
        for f in frames:
            print(f"location in {f.getName()}: {mark.findLocationInFrame(state,f)}")
        #endfor
        print("\n")
    #endfor
    q10Transform = np.array([[0.8365,-0.2241,0.5000,0],
                            [0.3388,0.9288,-0.1504,0], 
                            [-0.4307,0.2952,0.8529,0],
                            [0,0,0,1]])
    outq10 = Rotation.from_matrix(q10Transform[0:3,0:3])
    print(outq10.as_euler('zyx', degrees=True))
    
    # print(getRotationMatrix(positions[0],positions[1]))
    # print(getRotationMatrix(positions[1],positions[2]))
    # vec3 = []
    # count = 0
    # fNames = []
    # transMat = []
    # frames = []
    # tmpF = osim.Frame
    # frameNames = ['r_ulna_radius_hand_offset','r_humerus_offset','base_offset']
    # for frame in myModel.getFrameList():
    #     frames.append(frame)
    #     fNames.append(frame.getName())
    #     # fName = [x for i,x in enumerate(frameNames) if x == frame.getName()]
    #     # if fName:
    #     #     frames.append([fName[0],frame])
    #     # #endif
    # #endfor

    # for f in frames:
    #     frame = f
    #     if len(vec3)>1:
    #         formatTrans(state,frame,tmpF)
    #         formatTrans(state,tmpF,frame)
    #         # transMat.append((f"{f[0]}-{tmpF.getName()}",tmpT,tmpR))
    #     #endif
    #     vec3.append(frame.getName())
    #     print("\n")        
    #     tmpF = frame
    #     count += 1
    # #endfor

# %%
