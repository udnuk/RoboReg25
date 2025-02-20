import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np

p.connect(p.GUI)

p.setAdditionalSearchPath("/urdf/")

Ball_Location = [0.8, 0.1, 0.75] #Adjust ball location

p.loadURDF("plane.urdf")
ball_id = p.loadURDF("ball.urdf", basePosition=Ball_Location, useFixedBase=False)
robot_id = p.loadURDF("arm.urdf", basePosition=[-1, 0, 0], useFixedBase=True)

p.setGravity(0, 0, -9.81)

mass = p.getDynamicsInfo(ball_id, -1)[0]
gravity_enabled = False

# Camera settings
width, height = 640, 480
fov = 60
aspect = width / height
near, far = 0.02, 10

sphere_link4_index = 7

while True:
    if not gravity_enabled:
        contacts = p.getContactPoints(ball_id)
        if contacts:
            print("Collision detected! Gravity enabled.")
            gravity_enabled = True
        else:
            p.applyExternalForce(
                ball_id, 
                -1, 
                forceObj=[0, 0, mass * 9.81], 
                posObj=[0, 0, 0],
                flags=p.WORLD_FRAME
            )

    p.stepSimulation()
    
    link_state = p.getLinkState(robot_id, sphere_link4_index)
    pos, orn = link_state[4], link_state[5]

    # print(link_state)
    
    rotation = p.getMatrixFromQuaternion(orn)
    forward = np.array([rotation[2], rotation[5], rotation[8]])  # Z-axis
    up = np.array([-rotation[0], -rotation[3], -rotation[6]])       # X-axis
    target = pos + forward  # Look along the link's Z-axis
    
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=pos,
        cameraTargetPosition=target,
        cameraUpVector=up
    )
    
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    
    images = p.getCameraImage(width, height, view_matrix, projection_matrix, 
                             renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgb = images[2]  # Shape: (height, width, 3)
    
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.imshow("Camera Feed", bgr)
    cv2.waitKey(1)
    
    time.sleep(1./240.)


# # Run the simulation
# for _ in range(10000):
#     p.stepSimulation()
#     time.sleep(1./240.)  # Maintain real-time simulation at 240Hz

# num_joints = p.getNumJoints(robot_id)
# for i in range(num_joints):
#     print(p.getJointInfo(robot_id, i))

p.disconnect()
cv2.destroyAllWindows()