import numpy as np
import robot_params

try:
  import sim
except:
  print ('--------------------------------------------------------------')
  print ('"sim.py" could not be imported. This means very probably that')
  print ('either "sim.py" or the remoteApi library could not be found.')
  print ('Make sure both are in the same folder as this file,')
  print ('or appropriately adjust the file "sim.py"')
  print ('--------------------------------------------------------------')
  print ('')

client_ID = []


def sim_init():
  global sim
  global client_ID
  
  #Initialize sim interface
  sim.simxFinish(-1) # just in case, close all opened connections
  client_ID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim    
  if client_ID!=-1:
    print ('Connected to remote API server')
    return True
  else:
    return False

def get_handles():
  #Get the handles to the sim items

  global wheel_handles 
  global arm_handles
  global youbot_handle

  # Get handles
  res , youbot_handle = sim.simxGetObjectHandle(client_ID, "/youBot", sim.simx_opmode_blocking)
  wheel_handles = [-1, -1, -1, -1]
  res,  wheel_handles[0] = sim.simxGetObjectHandle(client_ID, "/youBot/rollingJoint_fl", sim.simx_opmode_blocking)
  res,  wheel_handles[1] = sim.simxGetObjectHandle(client_ID, "/youBot/rollingJoint_rl", sim.simx_opmode_blocking)
  res,  wheel_handles[2] = sim.simxGetObjectHandle(client_ID, "/youBot/rollingJoint_rr", sim.simx_opmode_blocking)
  res,  wheel_handles[3] = sim.simxGetObjectHandle(client_ID, "/youBot/rollingJoint_fr", sim.simx_opmode_blocking)
  arm_handles = [-1, -1, -1, -1, -1]
  res,  arm_handles[0] = sim.simxGetObjectHandle(client_ID, "/youBot/youBotArmJoint0", sim.simx_opmode_blocking)
  res,  arm_handles[1] = sim.simxGetObjectHandle(client_ID, "/youBot/youBotArmJoint1", sim.simx_opmode_blocking)
  res,  arm_handles[2] = sim.simxGetObjectHandle(client_ID, "/youBot/youBotArmJoint2", sim.simx_opmode_blocking)
  res,  arm_handles[3] = sim.simxGetObjectHandle(client_ID, "/youBot/youBotArmJoint3", sim.simx_opmode_blocking)
  res,  arm_handles[4] = sim.simxGetObjectHandle(client_ID, "/youBot/youBotArmJoint4", sim.simx_opmode_blocking)
  
  # Get the position of the YouBot for the first time in streaming mode
  res , youbot_1_Position = sim.simxGetObjectPosition(client_ID, youbot_handle, -1 , sim.simx_opmode_streaming)
  res , youbot_1_Orientation = sim.simxGetObjectOrientation(client_ID, youbot_handle, -1 , sim.simx_opmode_streaming)
  
  # Stop all joint actuations:Make sure Youbot is stationary
  for i in range(4):
      res = sim.simxSetJointTargetVelocity(client_ID, wheel_handles[i], 0.0, sim.simx_opmode_streaming)
  
  #Set arm to staright up
  for i in range(5):
      res = sim.simxSetJointTargetPosition(client_ID, arm_handles[i], 0.0, sim.simx_opmode_streaming)
  
  print ("Succesfully obtained handles")

  return

def start_simulation():
  global sim
  global client_ID

  ###Start the Simulation: Keep printing out status messages!!!
  res = sim.simxStartSimulation(client_ID, sim.simx_opmode_oneshot_wait)

  if res == sim.simx_return_ok:
    print ("---!!! Started Simulation !!! ---")
    return True
  else:
    return False

def localize_robot():
  #Function that will return the current location of youbot
  #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS        
  global sim
  global client_ID
  global youbot_handle
  
  res , youbot_Position = sim.simxGetObjectPosition(client_ID, youbot_handle, -1 , sim.simx_opmode_buffer)
  res , youbot_Orientation = sim.simxGetObjectOrientation(client_ID, youbot_handle, -1 , sim.simx_opmode_buffer)
  
  x = youbot_Position[0]
  y = youbot_Position[1]
  theta  = youbot_Orientation[1]

  return [x,y,theta]           

def set_vel_youbot(Vx, Vy, W):
  #Function to set the linear and rotational velocity of youbot
  global sim
  global client_ID
  global wheel_handles
  
  print("Velcities", Vx, Vy, W)
          
  # Set velocity
  sim.simxSetJointTargetVelocity(client_ID, wheel_handles[0], -Vx -Vy -W, sim.simx_opmode_oneshot_wait)
  sim.simxSetJointTargetVelocity(client_ID, wheel_handles[1], -Vx +Vy -W, sim.simx_opmode_oneshot_wait)
  sim.simxSetJointTargetVelocity(client_ID, wheel_handles[2], -Vx -Vy +W, sim.simx_opmode_oneshot_wait)
  sim.simxSetJointTargetVelocity(client_ID, wheel_handles[3], -Vx +Vy +W, sim.simx_opmode_oneshot_wait)
  
  return  

def set_arm_position_youbot(theta_desired):
  global sim
  global client_ID
  global arm_handles
  
  #Set arm to desired configuration
  for i in range(5):
      sim.simxSetJointTargetPosition(client_ID, arm_handles[i], np.deg2rad(theta_desired[i]), sim.simx_opmode_oneshot_wait)
      
  return
      
def sim_shutdown():
  #Gracefully shutdown simulation

  global sim
  global client_ID

  #Stop simulation
  res = sim.simxStopSimulation(client_ID, sim.simx_opmode_oneshot_wait)
  if res == sim.simx_return_ok:
    print ("---!!! Stopped Simulation !!! ---")

  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
  sim.simxGetPingTime(client_ID)

  # Now close the connection to CoppeliaSim:
  sim.simxFinish(client_ID)      

  return            

