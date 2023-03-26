import robot_params
import numpy as np 
import sim_interface

def at_goal(goal_state):    
    
    robot_state = sim_interface.localize_robot()
    flag_dist_thershold = False
    flag_orientation_threshold = False
    
    #check distance to goal point 
    d = np.sqrt(((goal_state[0] - robot_state[0])**2) + ((goal_state[1] - robot_state[1])**2))
    
    if d <= robot_params.goal_dist_threshold:
        flag_dist_thershold = True
    
    #check orientation diff 
    delta_theta = goal_state[2] - robot_state[2]
    #restrict angle to (-pi,pi)
    delta_theta = ((delta_theta + np.pi)%(2.0*np.pi)) - np.pi
    
    if np.abs(delta_theta) < robot_params.goal_orientation_threshold:
        flag_orientation_threshold = True
        
    if flag_dist_thershold and flag_orientation_threshold:
        return True
    else:
        return False

def gtg(goal_state):  
    #The Go to goal controller
    
    robot_state = sim_interface.localize_robot()
    
    #check distance to goal point 
    d = np.sqrt(((goal_state[0] - robot_state[0])**2) + ((goal_state[1] - robot_state[1])**2))
    
    if d > robot_params.goal_dist_threshold:
        #Only linear motion
        W = 0.0
        
        #Get actuation in global frame
        if np.abs(goal_state[1] - robot_state[1]) > (robot_params.goal_dist_threshold/2.0):
            Vx = np.copysign(robot_params.youbot_max_V, goal_state[1] - robot_state[1])
        else:
            Vx = 0.0
        
        if np.abs(goal_state[0] - robot_state[0]) > (robot_params.goal_dist_threshold/2.0):
            Vy = np.copysign(robot_params.youbot_max_V, goal_state[0] - robot_state[0])
        else:
            Vy = 0.0                
        
        #Get current orientation and get actuation in local frame
        local_Vx = Vx*np.cos(-robot_state[2]) - Vy*np.sin(-robot_state[2])
        local_Vy = Vx*np.sin(-robot_state[2]) + Vy*np.cos(-robot_state[2])
        
        #request robot to execute velocity
        sim_interface.set_vel_youbot(local_Vx, local_Vy, W)
        
        return
    else:
        #check orientation diff 
        delta_theta = goal_state[2] - robot_state[2]
        #restrict angle to (-pi,pi)
        delta_theta = ((delta_theta + np.pi)%(2.0*np.pi)) - np.pi
        
        if np.abs(delta_theta) > robot_params.goal_orientation_threshold:
            #Only angular motion 
            Vx = 0.0
            Vy = 0.0 
            W = np.copysign(robot_params.youbot_max_W, delta_theta)
            
            #request robot to execute velocity
            sim_interface.set_vel_youbot(Vx, Vy, W)
            
            return
        
        else:
            #request robot to stop
            sim_interface.set_vel_youbot(0.0, 0.0, 0.0)
            
            return
    
   
                                       
                   
