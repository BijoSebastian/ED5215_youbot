#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time

#Import files
import sim_interface
import control
import numpy as np

def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):
            
            #Obtain robots position
            robot_state = sim_interface.localize_robot()
            
            #Set goal state
            goal_state = [1.0, 1.0, np.pi/2.0]
            
            while not control.at_goal(goal_state):
                print("Robot state:", sim_interface.localize_robot())
                control.gtg(goal_state)
                time.sleep(0.5)
            
            #request robot to stop
            sim_interface.set_vel_youbot(0.0, 0.0, 0.0)
            
            
            #Set arm position
            sim_interface.set_arm_position_youbot([0.0, 0.0, 52.0, 72.0, 0.0])
            time.sleep(5)

        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    sim_interface.set_vel_youbot(0.0, 0.0, 0.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 