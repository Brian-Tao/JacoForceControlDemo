
import sim
import time
import math
#from scipy.spatial.transform import Rotation as R


def calculateXY(startTime, currentTime, radius):
    '''
    @brief function to calculate x,y coordinates, centered at (0, 0)
    @param time startTime
    @param time currentTime
    @param int radius
    @return int x
    @return int y
    '''
    t = currentTime - startTime
    x = 0 + radius * math.cos(0.1 * t + math.pi/2)
    y = 0 + radius * math.sin(0.1 * t + math.pi/2)
    return x, y

def measureStaticZForce(clientID, forceHandle):
    '''
    @brief function to measure z direction force at static state
    @param clientID
    @param forceHandle: handle of force sensor
    @return static z force
    '''
    startTime = time.time()
    sim.simxReadForceSensor(clientID, forceHandle, sim.simx_opmode_streaming)
    totForce = 0
    measureCount = 0
    while time.time() - startTime <= 2:
        res, state, force, torque = sim.simxReadForceSensor(clientID, forceHandle, sim.simx_opmode_blocking)
        totForce += force[2]
        measureCount += 1
        time.sleep(0.05)
    return totForce/measureCount

def main():
    print ('Program started')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    
    # initialize object handles
    _, robotHandle = sim.simxGetObjectHandle(clientID, "Jaco", sim.simx_opmode_blocking)
    print(_)
    _, targetHandle = sim.simxGetObjectHandle(clientID, "Jaco_target", sim.simx_opmode_blocking)
    print(_)
    _, forceHandle = sim.simxGetObjectHandle(clientID, "Jaco_connection", sim.simx_opmode_blocking)

    # move to the predefined pos
    targetPos = [0.0, 0.5, 0.4]
    sim.simxSetObjectPosition(clientID, targetHandle, sim.sim_handle_parent, targetPos, sim.simx_opmode_oneshot)

    targetRot = [0, 0, 0]
    sim.simxSetObjectOrientation(clientID, targetHandle, -1, targetRot, sim.simx_opmode_oneshot)
    time.sleep(2)

    # measure static z force
    staticZForce = measureStaticZForce(clientID, forceHandle)
    print("static vertical force is ", staticZForce, "N")
    targetPos = [0.0, 0.5, 0.05]
    sim.simxSetObjectPosition(clientID, targetHandle, sim.sim_handle_parent, targetPos, sim.simx_opmode_oneshot)
    time.sleep(2)

    # parameters to tune
    targetForce = 5.0
    periodTime = 0.05
    K = 0.01
    radius = 0.5

    # main loop
    startTime = time.time()
    currentTime = 0
    while currentTime <= 20:
        currentTime = time.time() - startTime
        res, state, force, torque = sim.simxReadForceSensor(clientID, forceHandle, sim.simx_opmode_blocking)
        zForce = staticZForce - force[2]
        print(zForce, currentTime)
        
        # main calculation and actuation
        targetPos[0], targetPos[1] = calculateXY(startTime, time.time(), radius)
        diffForce = targetForce - zForce
        targetPos[2] = targetPos[2] - K * periodTime *diffForce
        sim.simxSetObjectPosition(clientID, targetHandle, sim.sim_handle_parent, targetPos, sim.simx_opmode_oneshot)

        while time.time() - startTime - currentTime < periodTime:
            pass

   
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
    

if __name__ == "__main__":
    main()


