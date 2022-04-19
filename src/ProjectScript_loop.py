#%% Imports
import opensim as osim
from opensim import Vec3
from os.path import join, abspath, dirname
from os import mkdir
import numpy as np
import matplotlib.pyplot as plt
#%% Path
global cfpath, omArm26Dir
cnt = 0
cfpath = dirname(__file__)
prjPath = join(cfpath,'_data','final_project_saves')
#%% Definitions
def funExp(d,ddot):
    out = [] # force in N
    c = 0.01 # N*m^2
    for i in d:
        out.append(c/(i*i)+ddot)
    #endfor
    return out
#enddef

# def pertTurb(val=list(),perturbs=list()):
#     global cnt, defaultVals
#     if cnt < 1:
#         defaultVals = val
#     else:
#         val = defaultVals
#     #endif
#     return [j+perturbs for j in val]
# #enddef
#%% PARAMS
global pelvisWidth, thighLength, shankLength
pelvisWidth = 35*(0.01) #page 70, https://apps.dtic.mil/sti/pdfs/ADA016485.pdf
thighLength = 46*(0.01) #page 84, https://apps.dtic.mil/sti/pdfs/ADA016485.pdf
shankLength = 38*(0.01) #page 88, https://apps.dtic.mil/sti/pdfs/ADA016485.pdf
#%% PARAMS 2 CHANGE
# realistic mass ratios: https://www.d.umn.edu/~mlevy/CLASSES/ESAT3300/LABS/LAB8_COM/bsp.htm
# Thigh mass percent (M/F): 10.50/11.75
# Shank mass percent (M/F): 4.75/5.35
# Thigh mass center (M/F): 43.3/42.8
# Shank mass center (M/F): 43.4/41.9
# foot mass & mass center (M/F): 1.43/1.33 & 50/50
#   bnd
IxT = (1140*10**3)*(0.0098)/(100**2)
IyT = (1160*10**3)*(0.0098)/(100**2)
IzT = (200*10**3)*(0.0098)/(100**2)
IxS = (400*10**3)*(0.0098)/(100**2)
IyS = (400*10**3)*(0.0098)/(100**2)
IzS = (30*10**3)*(0.0098)/(100**2)
rThighI = osim.Inertia(IxT,IyT,IzT,0,0,0)
lThighI = osim.Inertia(IxT,IyT,IzT,0,0,0)
rShankI = osim.Inertia(IxS,IyS,IzS,0,0,0)
lShankI = osim.Inertia(IxS,IyS,IzS,0,0,0)
rTm = 6800*0.001
lTm = 6800*0.001
rSm = 2680*0.001
lSm = 2680*0.001
# HUNTCROSLEY forces (6 contact spheres): Forces -> Adjust contact stiffness and damping
stiffness           = [999000,999000,999000,999000,999000,999000]
dissipation         = [2.0,2.0,2.0,2.0,2.0,2.0]
staticFriction      = [0.8,0.8,0.8,0.8,0.5,0.5]
dynamicFriction     = [0.4,0.4,0.4,0.4,0.4,0.4]
viscousFriction     = [0.4,0.4,0.4,0.4,0.4,0.4]
transitionVelocity  = [0.5,0.5,0.2,0.2,0.2,0.2]
# TORQUE LIMIT (4 limits): • Forces -> Adjust joint limit range, stiffness and damping
upperStiffness = [1,1,0.5,0.5]
lowerStiffness = [1,1,0.5,0.5]
hipUpperLimit = [90,90,0,0]
hipUpperLimit = [np.deg2rad(j) for j in hipUpperLimit]
hipLowerLimit = [-90,-90,-135,-135]
hipLowerLimit = [np.deg2rad(j) for j in hipLowerLimit]
damping = [0.025,0.025,0.025,0.025]
transition = [5,5,5,5]
# https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4080802/
#%% MAIN
adj =  10000
var = 3
tmp = stiffness
tmp = [j+(adj*var) for j in tmp]
varName = 'stiffness'
perturb = np.arange(tmp[1]-(var*adj),tmp[1]+(var*adj),adj).astype('double')
if __name__ == '__main__':
    for i in range(0,len(perturb)):
        stiffness = [perturb[i] for j in range(0,len(stiffness))]
        saveDir = join(prjPath,varName)
        try: mkdir(saveDir)
        except FileExistsError: print(f'{saveDir} already exists')
        modSavePaths = join(saveDir,f'DynamicWalkerModel_'+varName+f'_{perturb[i]}.osim')

        # Load a Model from file
        myModel = osim.Model()
        myModel.setName('DynamicWalkerModel')
        ground = myModel.get_ground()
        myModel.set_gravity(Vec3(0, -9.80665, 0))

        # TODO: Construct Bodies and Joints Here
        platform = osim.Body()
        platform.setName('Platform')
        platform.setMass(1)
        platform.setInertia(osim.Inertia(1,1,1,0,0,0))
        
        platformGeometry = osim.Brick(Vec3(10,0.05,1))
        platformGeometry.setColor(Vec3(0.8,0.1,0.1))
        platform.attachGeometry(platformGeometry)
        # add body to model
        myModel.addBody(platform)

        # Section: Create the Platform Joint
        # Make and add a Pin joint for the Platform Body
        locationInParent    = Vec3(0,0,0)
        orientationInParent = Vec3(0,0,0)
        locationInChild     = Vec3(0,0,0)
        orientationInChild  = Vec3(0,0,0)
        platformToGround    = osim.PinJoint('PlatformToGround', # Joint Name
                                        ground,                 # Parent Frame
                                        locationInParent,       # Translation in Parent Frame
                                        orientationInParent,    # Orientation in Parent Frame
                                        platform,               # Child Frame
                                        locationInChild,        # Translation in Child Frame
                                        orientationInChild)     # Orientation in Child Frame
    
        # Add the PlatformToGround Joint to the Model
        myModel.addJoint(platformToGround)

        # TODO: Set the coordinate properties of the Pin Joint 
        platform_rz = platformToGround.upd_coordinates(0)
        platform_rz.setRangeMax(np.deg2rad(100))
        platform_rz.setRangeMax(np.deg2rad(-100))
        platform_rz.setName('platform_rz')
        platform_rz.setDefaultValue(np.deg2rad(-10))
        platform_rz.setDefaultSpeedValue(0)
        platform_rz.setDefaultLocked(True)

        # Section: Create the Pelvis
        # Make and add a Pelvis Body
        pelvis = osim.Body()
        pelvis.setName('Pelvis')
        pelvis.setMass(2)
        pelvis.setInertia(osim.Inertia(1,1,1,0,0,0))
        # Add geometry for display
        pelvis.attachGeometry(osim.Sphere(pelvisWidth))
        # Add Body to the Model
        myModel.addBody(pelvis)
        # Make and add a Planar joint for the Pelvis Body
        pelvisToPlatform = osim.PlanarJoint('PelvisToPlatform', platform, pelvis)
        # Update the coordinates of the new joint
        Pelvis_rz = pelvisToPlatform.updCoordinate(0) # Rotation about z
        Pelvis_rz.setRangeMax(np.pi)
        Pelvis_rz.setRangeMin(-np.pi)
        Pelvis_rz.setName('Pelvis_rz')
        Pelvis_rz.setDefaultValue(0)
        
        Pelvis_tx = pelvisToPlatform.updCoordinate(1); # Translation about x
        Pelvis_tx.setRangeMax(10)
        Pelvis_tx.setRangeMin(-5)
        Pelvis_tx.setName('Pelvis_tx')
        Pelvis_tx.setDefaultValue(-2)
        Pelvis_tx.setDefaultSpeedValue(0)
        
        Pelvis_ty = pelvisToPlatform.updCoordinate(2); # Translation about y
        Pelvis_ty.setRangeMax(5)
        Pelvis_ty.setRangeMin(-5)
        Pelvis_ty.setName('Pelvis_ty')
        Pelvis_ty.setDefaultValue(thighLength + shankLength)
        Pelvis_ty.setDefaultSpeedValue(0)

        myModel.addJoint(pelvisToPlatform)

        #%% LOWER LIMB BODIES
        lowerLimbBodies = []
        namesLLB = ['RightThigh','LeftThigh','RightShank','LeftShank']
        massLLB = [rTm,lTm,rSm,lSm]
        inertiaLLB = [rThighI,
                    lThighI,
                    rShankI,
                    lShankI]
        geomLLB = [osim.Ellipsoid(thighLength/10,thighLength/2,thighLength/10),
                    osim.Ellipsoid(thighLength/10,thighLength/2,thighLength/10),
                    osim.Ellipsoid(shankLength/10,shankLength/2,shankLength/10),
                    osim.Ellipsoid(shankLength/10,shankLength/2,shankLength/10)]
                    
        for i in range(0,len(namesLLB)):
            lowerLimbBodies.append(osim.Body())
            lowerLimbBodies[i].setMassCenter(Vec3(0,0,0))
            lowerLimbBodies[i].setName(namesLLB[i])
            lowerLimbBodies[i].setMass(massLLB[i])
            lowerLimbBodies[i].setInertia(inertiaLLB[i])
            lowerLimbBodies[i].attachGeometry(geomLLB[i])
            myModel.addBody(lowerLimbBodies[i])
        #endfor
        del namesLLB, massLLB, inertiaLLB, geomLLB

        #%% LOWER LIMB JOINTS
        lowerLimbJoints = []
        pervBod = [pelvis,pelvis,lowerLimbBodies[0],lowerLimbBodies[1]]
        childBod = [lowerLimbBodies[0],lowerLimbBodies[1],lowerLimbBodies[2],lowerLimbBodies[3]]
        namesLLJ = ['RightThighToPelvis','LeftThighToPelvis','RightShankToThigh','LeftShankToThigh']
        
        #%% JOINT COORDS
        rotations_rz = []
        rangeCoords = [(np.deg2rad(100),np.deg2rad(-100)),
                    (np.deg2rad(100),np.deg2rad(-100)),
                    (np.deg2rad(0),np.deg2rad(-100)),
                    (np.deg2rad(0),np.deg2rad(-100))]
        nameCoords = ['RHip_rz','LHip_rz','RKnee_rz','LKnee_rz']
        txName = ['RHip_tx','LHip_tx','RKnee_tx','LKnee_tx']
        tyName = ['RHip_ty','LHip_ty','RKnee_ty','LKnee_ty']
        defaultCoords = [np.deg2rad(30),
                        np.deg2rad(-10),
                        np.deg2rad(-30),
                        np.deg2rad(-30)]
        tx_off = [0,0,0,0]
        ty_off = [0,0,0,0]
        locNpar = [Vec3(0,0,pelvisWidth),Vec3(0,0,-pelvisWidth),Vec3(0,-thighLength/2,0),Vec3(0,-thighLength/2,0)]
        oriNpar = [Vec3(0,0,0),Vec3(0,0,0),Vec3(0,0,0),Vec3(0,0,0)]
        locNchi = [Vec3(0,thighLength/2,0),Vec3(0,thighLength/2,0),Vec3(0,shankLength/2,0),Vec3(0,shankLength/2,0)]
        oriNchi = [Vec3(0,0,0),Vec3(0,0,0),Vec3(0,0,0),Vec3(0,0,0)]

        for i in range(0,len(namesLLJ)):
            lowerLimbJoints.append(osim.PinJoint(namesLLJ[i],
                                                    pervBod[i],
                                                    locNpar[i],
                                                    oriNpar[i],
                                                    childBod[i],
                                                    locNchi[i],
                                                    oriNchi[i]))
            rotations_rz = lowerLimbJoints[i].updCoordinate(0) # Rotation about z
            rotations_rz.setRangeMax(rangeCoords[i][0])
            rotations_rz.setRangeMin(rangeCoords[i][1])
            rotations_rz.setName(nameCoords[i])
            rotations_rz.setDefaultValue(defaultCoords[i])
            myModel.addJoint(lowerLimbJoints[i])
        #endfor
        del childBod, pervBod, namesLLJ, rangeCoords, nameCoords, defaultCoords

        # TODO: Construct ContactGeometry and HuntCorssleyForces Here
        contactSphereRadius = 0.05    
        # Make a Contact Half Space
        groundContactLocation = Vec3(0,0.025,0)
        groundContactOrientation = Vec3(0,0,-1.57)
        groundContactSpace = osim.ContactHalfSpace(groundContactLocation,
                                                groundContactOrientation,
                                                platform)
        groundContactSpace.setName('PlatformContact')
        myModel.addContactGeometry(groundContactSpace)

        #%% SPHERE CONTACTS
        contactSphere = []
        cntctRadii = [contactSphereRadius,
                    contactSphereRadius,
                    contactSphereRadius,
                    contactSphereRadius,
                    contactSphereRadius,
                    contactSphereRadius]
        cntctLocation = [Vec3(0,0,pelvisWidth),
                        Vec3(0,0,-pelvisWidth),
                        Vec3(0,shankLength/2,0),
                        Vec3(0,shankLength/2,0),
                        Vec3(0,-shankLength/2,0),
                        Vec3(0,-shankLength/2,0)]
        cntctFrame = [pelvis,
                    pelvis,
                    lowerLimbBodies[2],
                    lowerLimbBodies[3],
                    lowerLimbBodies[2],
                    lowerLimbBodies[3]]
        cntctName = ['RHipContact','LHipContact','RKneeContact','LKneeContact','RFootContact','LFootContact']

        for i in range(0,len(cntctName)):
            contactSphere.append(osim.ContactSphere())
            contactSphere[i].setRadius(cntctRadii[i])
            contactSphere[i].setLocation(cntctLocation[i])
            contactSphere[i].setFrame(cntctFrame[i])
            contactSphere[i].setName(cntctName[i])
            myModel.addContactGeometry(contactSphere[i])
        #endfor

        #%% ADD HUNTCROSELEY FORCES    
        forcesHuntCrosley = []
        forcesNames = ['RHipForce','LHipForce','RKneeForce','LKneeForce','RFootForce','LFootForce']
        forcesGeom1 = ['RHipContact','LHipContact','RKneeContact','LKneeContact','RFootContact','LFootContact']
        forcesGeom2 = ['PlatformContact','PlatformContact','RHipContact','LHipContact','RKneeContact','LKneeContact']
        for i in range(0,len(forcesNames)):
            forcesHuntCrosley.append(osim.HuntCrossleyForce())
            forcesHuntCrosley[i].setName(forcesNames[i])
            forcesHuntCrosley[i].addGeometry(forcesGeom1[i])
            forcesHuntCrosley[i].addGeometry('PlatformContact')
            forcesHuntCrosley[i].setStiffness(stiffness[i])
            forcesHuntCrosley[i].setDissipation(dissipation[i])
            forcesHuntCrosley[i].setStaticFriction(staticFriction[i])
            forcesHuntCrosley[i].setDynamicFriction(dynamicFriction[i])
            forcesHuntCrosley[i].setViscousFriction(viscousFriction[i])
            forcesHuntCrosley[i].setTransitionVelocity(transitionVelocity[i])
            myModel.addForce(forcesHuntCrosley[i])
        #endfor

        # TODO: Construct CoordinateLimitForces Here
        # Define Coordinate Limit Force Parameters
        limitTorque = []
        torqueNames = ['RHipLimitTorque','LHipLimitTorque','RKneeLimitTorque','LKneeLimitTorque']
        torqueCoords = ['RHip_rz','LHip_rz','RKnee_rz','LKnee_rz']
        for i in range(0,len(torqueNames)):
            limitTorque.append(osim.CoordinateLimitForce())
            limitTorque[i].setName(torqueNames[i])
            limitTorque[i].set_coordinate(torqueCoords[i])
            limitTorque[i].setUpperStiffness(upperStiffness[i])
            limitTorque[i].setLowerStiffness(lowerStiffness[i])
            limitTorque[i].setUpperLimit(hipUpperLimit[i])
            limitTorque[i].setLowerLimit(hipLowerLimit[i])
            limitTorque[i].setDamping(damping[i])
            limitTorque[i].setTransition(transition[i])
            myModel.addForce(limitTorque[i])
        #endfor

        # Define magnet forces about knee joints (contraction)
        magnetForce = []
        magnetNames = ["RKnee_pointtopoint","LKnee_pointtopoint"]
        body1 = [str(lowerLimbBodies[0].getName()),str(lowerLimbBodies[1].getName())]
        point1 = [Vec3(0,0,0),Vec3(0,0,0)] #[Vec3(0,thighLength/2,0),Vec3(0,thighLength/2,0)]
        body2 = [str(lowerLimbBodies[2].getName()),str(lowerLimbBodies[3].getName())]
        point2 = [Vec3(0,0,0),Vec3(0,0,0)] #[Vec3(0,shankLength/2-0.05,0),Vec3(0,shankLength/2-0.05,0)]
        expression = ['0.01/(d*d)','0.01/(d*d)']
        # d = np.arange(0.1,5,0.01)
        # y = funExp(d,0)
        # plt.figure("knee forces")
        # plt.plot(d,y)
        # plt.ylabel('Force output (N/m)')
        # plt.xlabel('Distance from thigh to shank')
        # plt.show()
        for i in range(0,len(expression)):
            print(f"Body 1: {body1[i]}")
            print(f"Body 2: {body2[i]}")
            magnetForce.append(osim.ExpressionBasedPointToPointForce())
            magnetForce[i].setName(magnetNames[i])
            magnetForce[i].setBody1Name(body1[i])
            magnetForce[i].setBody2Name(body2[i])
            magnetForce[i].setPoint1(point1[i])
            magnetForce[i].setPoint2(point2[i])
            magnetForce[i].setExpression(expression[i])
            myModel.addForce(magnetForce[i])
        #endfor

        # Define magnet forces about knee joints (extension)
        # magnetForce = []
        # magnetNames = ["RHip_dynP2P","LHip_dynP2P"]
        # body1 = [str(lowerLimbBodies[0].getName()),str(lowerLimbBodies[1].getName())]
        # point1 = [Vec3(0,0,0),Vec3(0,0,0)]
        # body2 = [str(lowerLimbBodies[2].getName()),str(lowerLimbBodies[3].getName())]
        # point2 = [Vec3(0,0,0),Vec3(0,0,0)]
        # expression = ['0.01/(d*d)','0.01/(d*d)']
        # for i in range(0,len(expression)):
        #     print(f"Body 1: {body1[i]}")
        #     print(f"Body 2: {body2[i]}")
        #     magnetForce.append(osim.ExpressionBasedPointToPointForce())
        #     magnetForce[i].setName(magnetNames[i])
        #     magnetForce[i].setBody1Name(body1[i])
        #     magnetForce[i].setBody2Name(body2[i])
        #     magnetForce[i].setPoint1(point1[i])
        #     magnetForce[i].setPoint2(point2[i])
        #     magnetForce[i].setExpression(expression[i])
        #     myModel.addForce(magnetForce[i])
        # #endfor

        # Initialize the System
        state = myModel.initSystem()
        # save the model to a file
        try: mkdir(prjPath)
        except FileExistsError: print(f'{prjPath} already exists')

        myModel.printToXML(modSavePaths)
        print(f'DynamicWalkerModel.osim printed!')

        positions = []
        bodies = []
        for body in myModel.getBodyList():
            print(body.getName())
            # print(f"inertia: {body.get_inertia()}")
            # print(f"mass center: {body.get_mass_center()}")
            # print(f"mass: {body.get_mass()}")
            pp = body.getPositionInGround(state)
            # print(f"current position: {pp}")
            positions.append(pp.to_numpy())
            bodies.append(body)
            print("\n")
        #endfor
        print(f"{np.linalg.norm(positions[2]-positions[4])}")
        print(f"{np.linalg.norm(positions[3]-positions[5])}")
    #endfor