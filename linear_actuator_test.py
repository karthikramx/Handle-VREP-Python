import vrep
import time
from pid import *
from full_body_kinematics import *
import time
import math


vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
i = 0;

if clientID != -1:

    vrep.simxSynchronous(clientID,True)
    returnCode=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
    print("connection successful!!")

    #vrep object handles
    errorCode,linear_act = vrep.simxGetObjectHandle(clientID,'Prismatic_joint',vrep.simx_opmode_blocking)
    errorCode,linear_act0 = vrep.simxGetObjectHandle(clientID,'Prismatic_joint0',vrep.simx_opmode_blocking)

    while True:
        i = i + 0.005 * 2 * math.pi
        try:
            vrep.simxPauseCommunication(clientID,1)

            #vrep.simxSetJointTargetPosition(clientID,linear_act,0.07*math.sin(i),vrep.simx_opmode_oneshot)
            #vrep.simxSetJointTargetPosition(clientID,linear_act0,0.07*math.sin(i),vrep.simx_opmode_oneshot)

            vrep.simxPauseCommunication(clientID,0)
            vrep.simxSynchronousTrigger(clientID)

        except KeyboardInterrupt:
            returnCode=vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
            vrep.simxFinish(clientID)
            print "simulation stopped!"
            exit(-1)

else:
     print("connection failed!!")
     sys.exit()
