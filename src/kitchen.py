import os, sys 
import time
import pybullet as p
import pybullet_data  



class Kitchen:
    def __init__(self): 
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/models') 
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        p.setGravity(0, 0, -9.81)  
        self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
        self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
        self.table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)  
        self.drawer_to_joint_id = {1: 18, 
                                   2: 22, 
                                   3: 27, 
                                   4: 31,
                                   5: 37, 
                                   6: 40, 
                                   7: 48, 
                                   8: 53, 
                                   9: 56, 
                                   10: 58, 
                                   11: 14}
        self.drawer_to_joint_limits = {1: (0, 1.57), 
                                       2: (-1.57, 0), 
                                       3: (-1.57, 0), 
                                       4: (0, 1.57),
                                       5: (0.0, 0.4), 
                                       6: (0.0, 0.4), 
                                       7: (0, 1.57), 
                                       8: (-1.57, 0), 
                                       9: (0.0, 0.4), 
                                       10: (0.0, 0.4), 
                                       11: (0, 1.57)}
        self.handle_joints = [i for i in range(p.getNumJoints(self.kitchen)) if 'handle' in p.getJointInfo(self.kitchen, i)[1].decode('UTF-8')]
        self.drawer_joints = [i for i in range(p.getNumJoints(self.kitchen)) if 'drawer' in p.getJointInfo(self.kitchen, i)[1].decode('UTF-8')]
        self.drawer_joints_prismatic = [i for i in range(p.getNumJoints(self.kitchen)) if 'drawer' in p.getJointInfo(self.kitchen, i)[1].decode('UTF-8') and p.getJointInfo(self.kitchen, i)[2] == p.JOINT_PRISMATIC]
        # num_joints = p.getNumJoints(self.kitchen)
        # for i in range(num_joints):
        #     joint_info = p.getJointInfo(self.kitchen, i)
        #     print(f"Joint index: {i}")
        #     print(f"Joint name: {joint_info[1].decode('UTF-8')}")
        #     print(f"Joint type: {joint_info[2]}")
        # import sys
        # sys.exit()
        

    def apply_force(self, joint_id, force_magnitude):
        print("Joint id: ", joint_id)
        print("------")
        force = [force_magnitude, 0, 0]  # Apply a force of 100 in the z-direction.
        position = [0, 0, 0]  # Apply the force at the origin of the joint's coordinate frame.

        # Start the simulation and apply the force for 1000 time steps.
        p.applyExternalForce(self.kitchen, joint_id, force, position, p.LINK_FRAME)
        p.stepSimulation()
    
    def asses(self, joint_id):
        # Get the link state.
        print("Joint id: ", joint_id)
        print("------")
        link_state = p.getLinkState(self.kitchen, joint_id)

        # Extract the position and orientation.
        position = link_state[0]
        orientation = link_state[1]

        print("Position: ", position)
        print("Orientation: ", orientation)

        euler_angles = p.getEulerFromQuaternion(orientation)
        print("Euler angles: ", euler_angles)
        
            
    def open_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        open_angle = self.drawer_to_joint_limits[drawer_id][1]
        p.setJointMotorControl2(bodyIndex=self.kitchen, jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=open_angle, maxVelocity=0.5) 
        # p.setJointMotorControl2()
        p.stepSimulation()


    # def step(self, action):

    #     joint_id = self.drawer_to_joint_id[action]
    #     open_angle = self.drawer_to_joint_limits[action][1]
    #     p.setJointMotorControl2(bodyIndex=self.kitchen, jointIndex=joint_id, controlMode=p.VELOCITY_CONTROL targetPosition=open_angle, maxVelocity=0.5)



    
        

    # def open_drawer_with_force(self, drawer_id, force_magnitude):
    #     joint_id = self.drawer_to_joint_id[drawer_id]
    #     # p.applyExternalForce(objectUniqueId=self.kitchen, 
    #     #                     linkIndex=joint_id, 
    #     #                     forceObj=[force_magnitude, 0, 0],  # Change the direction of force as needed
    #     #                     posObj=[0, 0, 0],  # Apply force at the center of mass
    #     #                     flags=p.WORLD_FRAME)
    #     p.applyExternalForce(objectUniqueId=self.kitchen, 
    #                         linkIndex=joint_id, 
    #                         forceObj=[force_magnitude, 0, 0],  # Change the direction of force as needed
    #                         posObj=[0, 0, 0],  # Apply force at the center of mass
    #                         flags=p.LINK_FRAME)
    #     # p.applyExternalForce(objectUniqueId=self.kitchen, 
    #     #                     linkIndex=joint_id, 
    #     #                     forceObj=[0, force_magnitude, 0],  # Change the direction of force as needed
    #     #                     posObj=[0, 0, 0],  # Apply force at the center of mass
    #     #                     flags=p.WORLD_FRAME)
        
    #     p.stepSimulation()

    # def close_drawer_with_force(self, drawer_id, force_magnitude):
    #     joint_id = self.drawer_to_joint_id[drawer_id]
    #     p.applyExternalForce(objectUniqueId=self.kitchen, 
    #                         linkIndex=joint_id, 
    #                         forceObj=[-force_magnitude, 0, 0],  # Change the direction of force as needed
    #                         posObj=[0, 0, 0],  # Apply force at the center of mass
    #                         flags=p.WORLD_FRAME)
    #     p.stepSimulation()

    def step(self):

        p.stepSimulation()

    def close(self):
        p.disconnect(self.client)

    def reset(self):
        pass

    def check_is_done(self):
        return False

    def close_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        close_angle = self.drawer_to_joint_limits[drawer_id][0]
        p.setJointMotorControl2(bodyIndex=self.kitchen, jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=close_angle, maxVelocity=0.5) 
        p.stepSimulation()


### Example usage
if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)

    #initialize kitchen object
    kitchen = Kitchen()
    print ("Handle joints: ", kitchen.handle_joints)
    time.sleep(5)
    force = [1000, 0, 0]  # Apply a force of 100 in the positive x-direction.
    position = [0, 0, 0]  # Apply the force at the origin of the joint's coor
    torque = [1, 0, 0]
    debug_line_ids = [None]*len(kitchen.handle_joints)

    for joint in kitchen.drawer_joints_prismatic:
        contacts = p.getContactPoints(bodyA=kitchen.kitchen, bodyB=kitchen.kitchen, linkIndexA=joint)
        if contacts:
            print(f"Joint {joint} is in contact with another part of the model.")

    for joint in kitchen.drawer_joints_prismatic:
        info = p.getJointInfo(kitchen.kitchen, joint)
        print(f"Joint {joint}: Lower Limit = {info[8]}, Upper Limit = {info[9]}")


    # for joint in kitchen.drawer_joints:
    #     info = p.getDynamicsInfo(kitchen.kitchen, joint)
    #     print(f"Joint {joint}: Mass = {info[0]}, Inertia = {info[2]}")

    # for joint in kitchen.drawer_joints:
    #     info = p.getJointInfo(kitchen.kitchen, joint)
    #     print(f"Joint {joint}: Type = {info[2]}")

    # for _ in range(10000):
    #     # p.setGravity(0,0,-9.8)
    #     for j in range(len(kitchen.handle_joints)):
    #         # Remove existing debug line.
    #         if debug_line_ids[j] is not None:
    #             p.removeUserDebugItem(debug_line_ids[j])
    #         joint_info = p.getJointInfo(kitchen.kitchen, kitchen.handle_joints[j])
    #         print(f"Joint index: {kitchen.handle_joints[j]}")
    #         print(f"Joint name: {joint_info[1].decode('UTF-8')}")
    #         print(f"Joint type: {joint_info[2]}")
    #         print(f"Joint limits: {joint_info[8:10]}")
    #         print(f"Joint position: {p.getJointState(kitchen.kitchen, kitchen.handle_joints[j])[0]}")
    #         print(f"Joint velocity: {p.getJointState(kitchen.kitchen, kitchen.handle_joints[j])[1]}")
    #         print(f"Joint reaction forces: {p.getJointState(kitchen.kitchen, kitchen.handle_joints[j])[2]}")

    #         # Apply force.
    #         p.applyExternalForce(kitchen.kitchen, kitchen.handle_joints[j], force, position, p.LINK_FRAME)
    #         # Add new debug line.
    #         link_state = p.getLinkState(kitchen.kitchen, kitchen.handle_joints[j])
    #         debug_line_ids[j] = p.addUserDebugLine(link_state[0], 
    #             [link_state[0][0]+force[0], link_state[0][1]+force[1], link_state[0][2]+force[2]], 
    #             [1, 0, 0], 2)
    #     p.stepSimulation()
    #     time.sleep(1./240.)

    for _ in range(10000):
        for j in range(len(kitchen.drawer_joints_prismatic)): # assuming drawer_joints is a list of joint indices for the drawers
            # Remove existing debug line.
            if debug_line_ids[j] is not None:
                p.removeUserDebugItem(debug_line_ids[j])
            # Apply force.
            # p.applyExternalForce(kitchen.kitchen, kitchen.drawer_joints_prismatic[j], force, position, p.LINK_FRAME)
            p.applyExternalTorque(kitchen.kitchen, kitchen.drawer_joints_prismatic[j], torque, p.LINK_FRAME)

            # Add new debug line.
            link_state = p.getLinkState(kitchen.kitchen, kitchen.drawer_joints_prismatic[j])
            debug_line_ids[j] = p.addUserDebugLine(link_state[0], 
                [link_state[0][0]+force[0], link_state[0][1]+force[1], link_state[0][2]+force[2]], 
                [1, 0, 0], 2)
        p.stepSimulation()
        # for joint in kitchen.drawer_joints:
        #     state = p.getJointState(kitchen.kitchen, joint)
        #     print(f"Joint {joint}: Position = {state[0]}, Velocity = {state[1]}")
        time.sleep(1./240.)

    # for drawer_id in range(1, 12):
    #     kitchen.asses(joint_id = kitchen.drawer_to_joint_id[drawer_id])

    # # for drawer_id in range(1, 12):
    # for _ in range(1000):
    #     kitchen.apply_force(joint_id=57, force_magnitude=200)
    #     time.sleep(1/240)
    #     print ("applied force of 100 to drawer_id: ", drawer_id)
    # time.sleep(5)


    # # open all drawers
    # for i in range(10):
    #     drawer_id = i+1
    #     kitchen.open_drawer_with_force(drawer_id, force_magnitude=20)  # Use appropriate force magnitude
    #     time.sleep(0.2)

    # #close all drawers
    # for i in range(10):
    #     drawer_id = i+1
    #     kitchen.close_drawer_with_force(drawer_id, force_magnitude=10)  # Use appropriate force magnitude
    #     time.sleep(0.2)
    
    #open all drawers
    # for i in range(10):
    #     drawer_id = i+1
    #     kitchen.open_drawer(drawer_id)
    #     time.sleep(0.2)

    # time.sleep(1)

    # #close all drawers
    # for i in range(10):
    #     drawer_id = i+1
    #     kitchen.close_drawer(drawer_id)
    #     time.sleep(0.2)

    # time.sleep(1)




