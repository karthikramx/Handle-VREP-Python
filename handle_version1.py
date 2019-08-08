import vrep
import time
from pid import *
from full_body_kinematics import *
import time

FBK = FBK([5.5,4.0,5.5],'3RP')

J  = FBK.inverse_kinematics_3R(0.0,15.0,90.0,-1)

#PID functions
# def __init__(self,Kp=1.0,Ki=0.0,Kd=0.0,setpoint=0.0)
# control(self,setpoint=0,process_variable=0,dt=1)

PID_Position = PID(1,0,0.01,0)
PID_Orientation = PID(47,0.5,0.1,0)
PID_fast = PID(80,0.0,0.001,0)

goto = 10.0
updated = 0
push = 0
balancer_gain = 8

vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

jointHandles = []
eulerAngles  = []
position = []
i=0
if clientID != -1:
    vrep.simxSynchronous(clientID,True)
    returnCode=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
    print("connection successful!!")

    #vrep object handles
    errorCode,wheel1         = vrep.simxGetObjectHandle(clientID,'Revolute_joint1',       vrep.simx_opmode_blocking)
    errorCode,wheel2         = vrep.simxGetObjectHandle(clientID,'Revolute_joint2',       vrep.simx_opmode_blocking)
    errorCode,left_joint1    = vrep.simxGetObjectHandle(clientID,'left_joint1',           vrep.simx_opmode_blocking)
    errorCode,left_joint2    = vrep.simxGetObjectHandle(clientID,'left_joint2',           vrep.simx_opmode_blocking)
    errorCode,right_joint1   = vrep.simxGetObjectHandle(clientID,'right_joint1',          vrep.simx_opmode_blocking)
    errorCode,right_joint2   = vrep.simxGetObjectHandle(clientID,'right_joint2',          vrep.simx_opmode_blocking)
    errorCode,body           = vrep.simxGetObjectHandle(clientID,'left_link2',            vrep.simx_opmode_blocking)
    errorCode,Dummy          = vrep.simxGetObjectHandle(clientID,'ResizableFloor_5_25',   vrep.simx_opmode_blocking)
    errorCode,COM            = vrep.simxGetObjectHandle(clientID,'COM',                   vrep.simx_opmode_blocking)
    errorCode,balancer       = vrep.simxGetObjectHandle(clientID,'balancer',              vrep.simx_opmode_blocking)
    errorCode,balancer_joint = vrep.simxGetObjectHandle(clientID,'Balancer_joint',        vrep.simx_opmode_blocking)
    errorCode,origin         = vrep.simxGetObjectHandle(clientID,'Origin',                vrep.simx_opmode_blocking)

    # vrep.simxSetJointTargetPosition(clientID,left_joint1 ,-J[2],vrep.simx_opmode_oneshot)
    # vrep.simxSetJointTargetPosition(clientID,right_joint1,-J[2],vrep.simx_opmode_oneshot)
    # vrep.simxSetJointTargetPosition(clientID,left_joint2 ,-J[1],vrep.simx_opmode_oneshot)
    # vrep.simxSetJointTargetPosition(clientID,right_joint2,-J[1],vrep.simx_opmode_oneshot)

    errorCode,left_joint1_position  = vrep.simxGetJointPosition(clientID,left_joint1,vrep.simx_opmode_streaming)
    errorCode,right_joint1_position = vrep.simxGetJointPosition(clientID,right_joint1,vrep.simx_opmode_streaming)
    errorCode,left_joint2_position  = vrep.simxGetJointPosition(clientID,left_joint2,vrep.simx_opmode_streaming)
    errorCode,right_joint2_position = vrep.simxGetJointPosition(clientID,right_joint2,vrep.simx_opmode_streaming)
    errorCode,wheel1_position        = vrep.simxGetJointPosition(clientID,wheel1,vrep.simx_opmode_streaming)
    errorCode,wheel2_position        = vrep.simxGetJointPosition(clientID,wheel2,vrep.simx_opmode_streaming)

    print str(left_joint1_position) + ' ' + str(left_joint2_position) + ' ' + str(right_joint1_position) + ' ' + str(right_joint2_position) + ' ' + str(wheel1_position) + ' ' + str(wheel2_position)
    timer = time.time()
    while True:

        try:
            vrep.simxPauseCommunication(clientID,1)
            i = i+1

            #Get Robot State
            if i==1:
                errorCode,eulerAngles=vrep.simxGetObjectOrientation(clientID,body,-1,vrep.simx_opmode_streaming)
                errorCode,position   =vrep.simxGetObjectPosition(clientID,body,-1,vrep.simx_opmode_streaming)
                errorCode,COM_position = vrep.simxGetObjectPosition(clientID,COM,body,vrep.simx_opmode_streaming)
                errorCode,bot_Velocity,bot_angularVelocity = vrep.simxGetObjectVelocity(clientID,body,vrep.simx_opmode_streaming)
            else:
                errorCode,eulerAngles=vrep.simxGetObjectOrientation(clientID,body,-1,vrep.simx_opmode_buffer)
                errorCode,position   =vrep.simxGetObjectPosition(clientID,body,-1,vrep.simx_opmode_buffer)
                errorCode,COM_position  = vrep.simxGetObjectPosition(clientID,COM,body,vrep.simx_opmode_buffer)
                errorCode,bot_Velocity,bot_angularVelocity = vrep.simxGetObjectVelocity(clientID,body,vrep.simx_opmode_buffer)

            print(bot_Velocity[0])
            #print str(COM_position[0]) #+ ' ' + str(COM_position[1]) + ' ' + str(COM_position[2])

            orientation_control = PID_Orientation.control(0,eulerAngles[1],1)

            # position_error = goto  - position[0]
            # print(position_error)
            # if(position[0]>goto and updated == 0):
            #     goto = 0.0
            #     updated = 1

            # errorCode,left_joint1_position  = vrep.simxGetJointPosition(clientID,left_joint1,vrep.simx_opmode_buffer)
            # errorCode,right_joint1_position = vrep.simxGetJointPosition(clientID,right_joint1,vrep.simx_opmode_buffer)
            # errorCode,left_joint2_position  = vrep.simxGetJointPosition(clientID,left_joint2,vrep.simx_opmode_buffer)
            # errorCode,right_joint2_position = vrep.simxGetJointPosition(clientID,right_joint2,vrep.simx_opmode_buffer)
            # errorCode,wheel1_position        = vrep.simxGetJointPosition(clientID,wheel1,vrep.simx_opmode_buffer)
            # errorCode,wheel2_position        = vrep.simxGetJointPosition(clientID,wheel2,vrep.simx_opmode_buffer)
            errorCode,balancer_joint_position  = vrep.simxGetJointPosition(clientID,balancer_joint,vrep.simx_opmode_buffer)
            # print str(left_joint1_position) + ' ' + str(left_joint2_position) + ' ' + str(right_joint1_position) + ' ' + str(right_joint2_position) + ' ' + str(wheel1_position) + ' ' + str(wheel2_position)

            #Set body Orientation
            vrep.simxSetJointTargetPosition(clientID,left_joint1,0,vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(clientID,right_joint1,0,vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(clientID,left_joint2,0,vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(clientID,right_joint2,0,vrep.simx_opmode_oneshot)

            vrep.simxSetJointTargetPosition(clientID,balancer_joint,balancer_gain*eulerAngles[1],vrep.simx_opmode_oneshot)
            #vrep.simxSetJointTargetPosition(clientID,balancer_joint, -position_error*(3.1416/180),vrep.simx_opmode_oneshot)
            if(((time.time() - timer) > 7) and ((time.time() - timer) < 8) and push == 0):
                vrep.simxSetJointTargetPosition(clientID,balancer_joint,24*eulerAngles[1],vrep.simx_opmode_oneshot)
                if((time.time() - timer)>7.5):
                    print "##############################################"
                    push = 1

            if(((time.time() - timer) > 11) and push == 1):
                vrep.simxSetJointTargetPosition(clientID,balancer_joint,32*eulerAngles[1],vrep.simx_opmode_oneshot)
                if((time.time() - timer)>11.5):
                    push = 0
                    print "##############################################"

            if(bot_Velocity[0]>1):
                vrep.simxSetJointTargetPosition(clientID,left_joint1 ,0.2,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint1,0.2,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,left_joint2 ,0.2,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint2,0.2,vrep.simx_opmode_oneshot)
            elif(bot_Velocity[0] < -1):
                vrep.simxSetJointTargetPosition(clientID,left_joint1 ,-0.2,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint1,-0.2,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,left_joint2 ,-0.2,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint2,-0.2,vrep.simx_opmode_oneshot)
            elif(bot_Velocity[0]>2):
                vrep.simxSetJointTargetPosition(clientID,left_joint1 ,0.4,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint1,0.4,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,left_joint2 ,0.4,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint2,0.4,vrep.simx_opmode_oneshot)
            elif(bot_Velocity[0] < -2):
                vrep.simxSetJointTargetPosition(clientID,left_joint1 ,-0.4,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint1,-0.4,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,left_joint2 ,-0.4,vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint2,-0.4,vrep.simx_opmode_oneshot)
                print "##############################################"
            else:
                vrep.simxSetJointTargetPosition(clientID,left_joint1 ,-J[2],vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint1,-J[2],vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,left_joint2 ,-J[1],vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetPosition(clientID,right_joint2,-J[1],vrep.simx_opmode_oneshot)



            #ik test

            #vrep.simxSetJointTargetPosition(clientID,wheel1      ,-J[0],vrep.simx_opmode_oneshot)
            #vrep.simxSetJointTargetPosition(clientID,wheel2      , J[0],vrep.simx_opmode_oneshot)

            #Set Motor Velocity
            vrep.simxSetJointTargetVelocity(clientID,wheel1,-orientation_control,vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID,wheel2,orientation_control,vrep.simx_opmode_oneshot)

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
