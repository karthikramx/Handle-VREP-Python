import vrep
import sys
import math
import time
import os
import subprocess

cwd = os.getcwd()
nino_ttt=cwd+"/nino.ttt"
print(nino_ttt)

# 
# cmd_line = "sudo ./vrep/vrep.sh"
# p = subprocess.Popen(cmd_line, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
# out = p.communicate()[0]
# print out

# time.sleep(1)

vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    res = vrep.simxLoadScene(clientID, nino_ttt, 0xFF, vrep.simx_opmode_oneshot)
    returnCode=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
    print("connection successful!!")
    #time.sleep(2)


    while True:
        try:
            pass
        except KeyboardInterrupt:
            returnCode=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
            raise

else:
     print("connection failed!!")
     sys.exit()
