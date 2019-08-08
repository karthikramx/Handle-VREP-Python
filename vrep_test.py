import vrep
import time
from pid import *

#PID
#def __init__(self,Kp=1.0,Ki=0.0,Kd=0.0,setpoint=0.0)
#control(self,setpoint=0,process_variable=0,dt=1)

PID_Position = PID(1,0,0.01,0)
PID_Orientation = PID(47,0.5,0.1,0)
PID_fast = PID(80,0.0,0.001,0)

vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

jointHandles = []
eulerAngles  = []
i=0
if clientID != -1:
    vrep.simxSynchronous(clientID,True)
    returnCode=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
    print("connection successful!!")

    #vrep object handles
    errorCode,joint1 = vrep.simxGetObjectHandle(clientID,'Revolute_joint1'    ,vrep.simx_opmode_blocking)
    errorCode,joint2 = vrep.simxGetObjectHandle(clientID,'Revolute_joint2'    ,vrep.simx_opmode_blocking)
    errorCode,joint3 = vrep.simxGetObjectHandle(clientID,'Revolute_joint3'    ,vrep.simx_opmode_blocking)
    errorCode,body   = vrep.simxGetObjectHandle(clientID,'body'               ,vrep.simx_opmode_blocking)
    errorCode,Dummy  = vrep.simxGetObjectHandle(clientID,'ResizableFloor_5_25',vrep.simx_opmode_blocking)

    jointHandles.append(joint1)
    jointHandles.append(joint2)
    jointHandles.append(joint3)


    while True:
        try:
            vrep.simxPauseCommunication(clientID,1)
            i = i+1
            #Get Robot State
            if i==1:
                errorCode,eulerAngles=vrep.simxGetObjectOrientation(clientID,body,-1,vrep.simx_opmode_streaming)
                errorCode,position   =vrep.simxGetObjectPosition(clientID,body,-1,vrep.simx_opmode_streaming)
            else:
                errorCode,eulerAngles=vrep.simxGetObjectOrientation(clientID,body,-1,vrep.simx_opmode_buffer)
                errorCode,position   =vrep.simxGetObjectPosition(clientID,body,-1,vrep.simx_opmode_buffer)

            #print('p1:'+str(position[0])+' p2:'+str(position[1])+' p3:'+str(position[2]))
            #print('p1:'+str(eulerAngles[0])+' p2:'+str(eulerAngles[1])+' p3:'+str(eulerAngles[2]))

            #PID Control
            # position_control = PID_Position.control(0.00005*i,position[0],1)
            #
            # if(eulerAngles[1]<(3*3.1416/180)):
            #     orientation_control = PID_Orientation.control(position_control,eulerAngles[1],1)
            # elif(eulerAngles[1]>=(3*3.1416/180)):
            #     orientation_control = PID_Orientation.control(0,eulerAngles[1],1)
            #     print("beyond 15!:" + str(orientation_control))


            #position_control = PID_Position.control(0.0005*i,position[0],1)


            orientation_control = PID_Orientation.control(0,eulerAngles[1],1)

            # if(eulerAngles[1]>3*3.1416/180):
            #     orientation_control = PID_fast.control(0,eulerAngles[1],1)
            # if(eulerAngles[1]<-3*3.1416/180):
            #     orientation_control = PID_fast.control(0,eulerAngles[1],1)

            #if(eulerAngles[1]>(1*3.1416/180) and eulerAngles[1]<(-1*3.1416/180)):
            #    orientation_control = 0

            #Set body Orientation
            vrep.simxSetJointTargetPosition(clientID,jointHandles[2],-3.3*eulerAngles[1],vrep.simx_opmode_oneshot)


            #Set Motor Velocity
            vrep.simxSetJointTargetVelocity(clientID,jointHandles[0],orientation_control,vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID,jointHandles[1],orientation_control,vrep.simx_opmode_oneshot)

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
