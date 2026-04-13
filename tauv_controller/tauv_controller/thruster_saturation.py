from tauv_msgs.msg import ThrusterSetpoint
from tauv_controller.force_optimizer import solve_thrusts
from geometry_msgs.msg import Wrench
import numpy as np


MAXFORCE = 37.0

def resolve_wrenches(wrenches_dict):
    """
    Vectorized priority-based thrust allocation
    """
    cum_wrench = np.zeros(6)
    cum_force = np.zeros(8)

    # Priority order: Heave > Roll/Pitch > Yaw > Surge/Sway
    prioritized_wrenches = [ 
        wrenches_dict['z'], 
        wrenches_dict['roll'] + wrenches_dict['pitch'], 
        wrenches_dict['yaw'], 
        wrenches_dict['x'] + wrenches_dict['y'] 
    ]
    
    for wrench in prioritized_wrenches:
        new_force = solve_thrusts(wrench)
        
        s_array = np.ones(8)
        
        pos_mask = new_force > 1e-6
        s_array[pos_mask] = (MAXFORCE - cum_force[pos_mask]) / new_force[pos_mask]
        
        neg_mask = new_force < -1e-6
        s_array[neg_mask] = (-MAXFORCE - cum_force[neg_mask]) / new_force[neg_mask]
        
        s = np.clip(np.min(s_array), 0.0, 1.0)

        cum_wrench += (s * wrench)
        cum_force += (s * new_force)

    thruster_msg = ThrusterSetpoint()
    thruster_msg.thrust = cum_force.tolist()

    wrench_msg = Wrench()
    wrench_msg.force.x = cum_wrench[0]
    wrench_msg.force.y = cum_wrench[1]
    wrench_msg.force.z = cum_wrench[2]
    wrench_msg.torque.x = cum_wrench[3]
    wrench_msg.torque.y = cum_wrench[4]
    wrench_msg.torque.z = cum_wrench[5]

    return thruster_msg, wrench_msg