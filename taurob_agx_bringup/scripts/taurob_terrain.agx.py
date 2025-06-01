########################################
# Name: tutorial_pick_and_place.agxPy
# Description:
# This tutorial shows how a robot can be controlled to pick up some boxes
# and place them on top of each other.
#
# The tutorial uses two "controllers" for driving the robot motion:
# - A high level controller that handles tasks such as open/close gripper and
#   computing the trajectory that the robot should follow when moving.
# - A low level controller that is responsible for tracking target joint angles.
#   - Target angles are received from the high level controller.
#   - The controller has access to a "force/torque sensor" to estimate the load from what the tool picks up
#   - The controller has feedback (can read current joint angles and joint velocities)
#
########################################

# AGX Dynamics imports
import agx
import agxCollide
import agxSDK
import agxModel
import agxOSG
import agxRender
import agxUtil
import agxIO
import agxCable

from agxPythonModules.utils.environment import simulation, init_app, application, root
from agxPythonModules.robots.panda import Panda  # noqa

import math
import numpy as np

from collections import namedtuple

def disable_collisions(panda, linkA, linkB):
    simulation().getSpace().setEnablePair(linkA, linkB, False)
    for lA in panda.assembly.getRigidBody(linkA).getGeometries(): 
      for lB in panda.assembly.getRigidBody(linkB).getGeometries():
        simulation().getSpace().setEnableCollisions(lA, lB, False)

def set_robot_initial_pose(panda, chain, pose):

    # Compute transforms for each link relative the base give the provided joint angles
    # The second argument clamps any joint angles within its limits.
    #
    # The tool fingers are not part of the chain and need to be positioned manually.
    #
    # The fingers and hand are all part of the same assembly and hence have that assemblys frame as
    # frame parent. Therefore we can work with the local matrix for hand/fingers when computing and
    # updating where the fingers are relative the hand.
    # Updating the frame hierarchy this way saves some computations compared using getMatrix / setMatrix.
    #
    hand = panda.assembly.getRigidBody('panda_hand')
    hand_matrix_inverse = hand.getFrame().getLocalMatrix().inverse()
    fingers = [panda.assembly.getRigidBody(name) for name in ['panda_rightfinger', 'panda_leftfinger']]
    finger_relative_transforms = [finger.getFrame().getLocalMatrix() * hand_matrix_inverse for finger in fingers]

    status, transforms = chain.computeForwardKinematicsAll(pose, True)

    if status is not agxModel.KinematicChain.SUCCESS:
        print(f"Error. Forward computation failed with status: {status}")
    else:
        # Set the local model frame matrix for each link/body.
        # Using local matrix to handle if entire robot was moved by e.g. setting a translation
        # in the robot-assembly-frame.
        for transform, link in zip(transforms, panda.links):
            link.getFrame().setLocalMatrix(transform)

    # The tool/finger positioning, maintain same relative transform to the hans as prior to the robot was repositioned.
    hand_matrix = hand.getFrame().getLocalMatrix()
    for finger, rel_transform in zip(fingers, finger_relative_transforms):
        finger.getFrame().setLocalMatrix(rel_transform * hand_matrix)

# Build robot scene
def buildBallToyScene():

    # Materials and ContactMaterials
    finger_material = agx.Material("GripperMaterial")
    box_material = agx.Material("BoxMaterial")

    mm = simulation().getMaterialManager()

    box_finger = mm.getOrCreateContactMaterial(box_material, finger_material)
    fm1 = agx.ScaleBoxFrictionModel(agx.FrictionModel.DIRECT)
    box_finger.setFrictionModel(fm1)
    box_finger.setFrictionCoefficient(0.9)

    box_box = mm.getOrCreateContactMaterial(box_material, box_material)
    fm2 = agx.ScaleBoxFrictionModel(agx.FrictionModel.DIRECT)
    box_box.setFrictionModel(fm2)
    box_box.setFrictionCoefficient(0.5)

    # Create the scene:
    # - a box as ground
    # - another box as a table/working area
    # - 3 boxes with different mass on the table
    # - a panda robot with a gripper tool
    #
    ground = agxCollide.Geometry(agxCollide.Box(3, 3, 0.1))
    ground.setPosition(agx.Vec3(0, 0, -0.1))
    ground.setName("ground")
    simulation().add(ground)
    ground_node = agxOSG.createVisual(ground, root())
    agxOSG.setDiffuseColor(ground_node, agxRender.Color.Black())

    # Table/working area
    #
    table_size = agx.Vec3(0.3, 0.3, 0.3)

    table = agxCollide.Geometry(agxCollide.Box(table_size))
    table.setPosition(agx.Vec3(0.5, 0, -0.2))
    table.setName("table")
    #table.setMaterial(box_material)
    simulation().add(table)
    
    agxOSG.createVisual(table, root())
    agxOSG.setTexture(table, root(), "textures/grid.png")

    drop_height = 0.1
    ball_radius = 0.03
    ball_mass = 0.1
    cyl_height=0.2
    cyl_mass=0.5
    cyl_radius=0.05
    cyl_inner_radius=0.045
    bottom_thickness=0.01
    cable_radius=0.005
    cable_resolution=5
    youngsModulus = 1e7

    stack1_pos = table.getPosition() + agx.Vec3(-0.5 * table_size[0], -0.5 * table_size[1], table_size[2] + cyl_height)
    stack2_pos = table.getPosition() + agx.Vec3(0.3 * table_size[0], 0.4 * table_size[1], table_size[2] + cyl_height/2 + drop_height + 2*ball_radius)

    #cylinder = agx.RigidBody()
    #cylinder.add(agxCollide.Geometry(agxCollide.Cylinder(cyl_radius,cyl_height)))
    #cylinder.setPosition(stack1_pos)
    #cylinder.setRotation(agx.EulerAngles(agx.Vec3(math.pi/2,0,0)))
    #cylinder.setName("cylinder1")
    #cylinder.getMassProperties().setMass(cyl_mass)
    #simulation().add(cylinder)
    #cyl_node = agxOSG.createVisual(cylinder, root())
    #agxOSG.setDiffuseColor(cyl_node, agxRender.Color.Yellow())
    #agxUtil.setBodyMaterial(cylinder, box_material)
   
    #now let's see the hollow cylinder
    hollow = agx.RigidBody()
    hollow.add(agxCollide.Geometry(agxCollide.HollowCylinder(cyl_inner_radius,cyl_height-bottom_thickness,cyl_radius-cyl_inner_radius)))
    bottom_of_container=agxCollide.Geometry(agxCollide.Cylinder(cyl_radius,bottom_thickness))
    bottom_of_container.setPosition(agx.Vec3(0,-(cyl_height-bottom_thickness)/2,0))
    hollow.add(bottom_of_container)
    hollow.setPosition(stack2_pos)
    hollow.setRotation(agx.EulerAngles(agx.Vec3(math.pi/2,0,0)))
    hollow.setName("hollow1")
    hollow.getMassProperties().setMass(cyl_mass)
    simulation().add(hollow)
    cyl_node = agxOSG.createVisual(hollow, root())
    agxOSG.setDiffuseColor(cyl_node, agxRender.Color.Red())
    agxUtil.setBodyMaterial(hollow, box_material)

    #add ball
    ball = agx.RigidBody()
    ball.add(agxCollide.Geometry(agxCollide.Sphere(ball_radius)))
    ball.setPosition(agx.Vec3(stack2_pos[0],stack2_pos[1],stack2_pos[2]-cyl_height/2-drop_height-ball_radius/2))
    ball.getMassProperties().setMass(ball_mass)
    simulation().add(ball)
    ball_node = agxOSG.createVisual(ball, root())
    agxOSG.setDiffuseColor(ball_node, agxRender.Color.Yellow())
    agxUtil.setBodyMaterial(ball, box_material)
    
    #add cable
    n_nodes=2
    cable= agxCable.Cable(cable_radius, cable_resolution)
    initTransform = agx.AffineMatrix4x4()
    initTransform.setTranslate(0,-(cyl_height+bottom_thickness)/2+cable_radius,0)
    initTransform.setRotate(agx.EulerAngles(agx.Vec3(math.pi/2,0,0)))
    cable.add(agxCable.BodyFixedNode(hollow,initTransform))
    cable.add(agxCable.FreeNode(agx.Vec3(stack2_pos[0],stack2_pos[1]+0.1,stack2_pos[2]-cyl_height/2-(drop_height/3))))
    cable.add(agxCable.FreeNode(agx.Vec3(stack2_pos[0]-0.2,stack2_pos[1],stack2_pos[2]-cyl_height/2-(2*drop_height/3))))
#for n in range(n_nodes):
#      cable.add(agxCable.FreeNode(agx.Vec3(stack2_pos[0],stack2_pos[1]+0.03,stack2_pos[2]-n*(drop_height/n_nodes))))
     
    ballTransform = agx.AffineMatrix4x4()
    ballTransform.setTranslate(0,0,cable_radius+ball_radius/2)

    properties = cable.getCableProperties()
    properties.setYoungsModulus(youngsModulus, agxCable.BEND)

    cable.add(agxCable.BodyFixedNode(ball,ballTransform))
    simulation().add(cable)
    cable_node = agxOSG.createVisual(cable, root())
    agxOSG.setDiffuseColor(cable_node, agxRender.Color.Green())
    

    ################################################################
    # The robot
    #
    panda = Panda(simulation(), use_tool=True, disable_self_collision=False)

    # sets maximum torque
    maximum_torque = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
    for tau, i in zip(maximum_torque, range(7)):
       #print("Setting tau_max ",tau," for motor ",i)
        panda.assembly.getConstraint1DOF("panda_joint"+str(i+1)).getMotor1D().setForceRange(-tau, tau)

    #disable collisions between base and table
    [simulation().getSpace().setEnableCollisions(geometry, ground, False) for geometry in panda.assembly.getRigidBody("panda_link0").getGeometries()]
    simulation().getSpace().setEnablePair("ground", "panda_link0", False)

    disable_collisions(panda,"panda_link0","panda_link1")
    disable_collisions(panda,"panda_link1","panda_link2")
    disable_collisions(panda,"panda_link2","panda_link3")
    disable_collisions(panda,"panda_link3","panda_link4")
    disable_collisions(panda,"panda_link4","panda_link5")
    disable_collisions(panda,"panda_link5","panda_link6")
    disable_collisions(panda,"panda_link6","panda_link7")
    disable_collisions(panda,"panda_link7","panda_link8")
    disable_collisions(panda,"panda_link6","panda_hand")
    disable_collisions(panda,"panda_link7","panda_hand")
    disable_collisions(panda,"panda_link8","panda_hand")
    disable_collisions(panda,"panda_hand","panda_leftfinger")
    disable_collisions(panda,"panda_hand","panda_rightfinger")
#to disable all self-collisions
#    simulation().getSpace().setEnablePair(panda.assembly.getName(), panda.assembly.getName(), False)
#    print("robot ", panda.assembly.getName())
#    print("ground ",ground.getName())
        
    # We add a relative transform to the end of the chain and will get a chain with
    # one additional link. This is so that we can make the position between the gripper fingers
    # the end of the chain. This simplifies which part of the robot we control.
    #
    chain = agxModel.SerialKinematicChain(simulation(), panda.base, panda.ee, panda.tooltip_offset)

    # Verify that a valid chain was found.
    if not chain.isValid():
        print("Error: a chain could not be found")
        application().stop()

    # Initial pose for the robot.
    set_robot_initial_pose(panda, chain, panda.q_ready)

    # Set material on the gripper fingers
    for name in ['panda_rightfinger', 'panda_leftfinger']:
        agxUtil.setBodyMaterial(panda.assembly.getRigidBody(name), finger_material)

    # Position camera
#    eye = agx.Vec3(1.93, -2.78, 0.93)
#    center = agx.Vec3(0.312, 0, 0.497)
#    up = agx.Vec3(-0.0251, 0.1424, 0.9895)
#    application().setCameraHome(eye, center, up)

    agxIO.writeFile("ball_toy_scene.agx",simulation())


# Build robot scene
def buildScene():

    # Materials and ContactMaterials
    finger_material = agx.Material("GripperMaterial")
    box_material = agx.Material("BoxMaterial")

    mm = simulation().getMaterialManager()

    box_finger = mm.getOrCreateContactMaterial(box_material, finger_material)
    fm1 = agx.ScaleBoxFrictionModel(agx.FrictionModel.DIRECT)
    box_finger.setFrictionModel(fm1)
    box_finger.setFrictionCoefficient(0.9)

    box_box = mm.getOrCreateContactMaterial(box_material, box_material)
    fm2 = agx.ScaleBoxFrictionModel(agx.FrictionModel.DIRECT)
    box_box.setFrictionModel(fm2)
    box_box.setFrictionCoefficient(0.5)

    # Create the scene:
    # - a box as ground
    # - another box as a table/working area
    # - 3 boxes with different mass on the table
    # - a panda robot with a gripper tool
    #
    ground = agxCollide.Geometry(agxCollide.Box(3, 3, 0.1))
    ground.setPosition(agx.Vec3(0, 0, -0.1))
    ground.setName("ground")
    simulation().add(ground)
    ground_node = agxOSG.createVisual(ground, root())
    agxOSG.setDiffuseColor(ground_node, agxRender.Color.Black())

    # Table/working area
    #
    table_size = agx.Vec3(0.3, 0.3, 0.3)

    table = agxCollide.Geometry(agxCollide.Box(table_size))
    table.setPosition(agx.Vec3(0.5, 0, -0.2))
    table.setName("table")
    #table.setMaterial(box_material)
    simulation().add(table)
    
    agxOSG.createVisual(table, root())
    agxOSG.setTexture(table, root(), "textures/grid.png")

    # The boxes on the table
    #
    box_size = agx.Vec3(0.04, 0.02, 0.03)

    box_stack1_pos = table.getPosition() + agx.Vec3(-0.5 * table_size[0], -0.5 * table_size[1], table_size[2] + box_size[2])
    box_stack2_pos = table.getPosition() + agx.Vec3(0.3 * table_size[0], 0.4 * table_size[1], table_size[2] + box_size[2])

    box_colors = [agxRender.Color.Yellow(), agxRender.Color.Orange(), agxRender.Color.Red()]
    box_names = ["yellow_box", "orange_box", "red_box"]
    box_positions = [box_stack1_pos, box_stack1_pos + agx.Vec3(0, 0, 0.05999), box_stack2_pos]

    for pos, color, mass, name in zip(box_positions, box_colors, [0.40, 0.50, 0.60], box_names):
        box = agx.RigidBody()
        box.add(agxCollide.Geometry(agxCollide.Box(0.04, 0.02, 0.03)))
        box.setPosition(pos)
        box.getMassProperties().setMass(mass)
        box.setName(name)

        simulation().add(box)
        box_node = agxOSG.createVisual(box, root())
        agxOSG.setDiffuseColor(box_node, color)

        agxUtil.setBodyMaterial(box, box_material)

    # The robot
    #
    panda = Panda(simulation(), use_tool=True, disable_self_collision=False)

    # sets maximum torque
    maximum_torque = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
    for tau, i in zip(maximum_torque, range(7)):
       #print("Setting tau_max ",tau," for motor ",i)
        panda.assembly.getConstraint1DOF("panda_joint"+str(i+1)).getMotor1D().setForceRange(-tau, tau)

    #disable collisions between base and table
    [simulation().getSpace().setEnableCollisions(geometry, ground, False) for geometry in panda.assembly.getRigidBody("panda_link0").getGeometries()]
    simulation().getSpace().setEnablePair("ground", "panda_link0", False)

    disable_collisions(panda,"panda_link0","panda_link1")
    disable_collisions(panda,"panda_link1","panda_link2")
    disable_collisions(panda,"panda_link2","panda_link3")
    disable_collisions(panda,"panda_link3","panda_link4")
    disable_collisions(panda,"panda_link4","panda_link5")
    disable_collisions(panda,"panda_link5","panda_link6")
    disable_collisions(panda,"panda_link6","panda_link7")
    disable_collisions(panda,"panda_link7","panda_link8")
    disable_collisions(panda,"panda_link6","panda_hand")
    disable_collisions(panda,"panda_link7","panda_hand")
    disable_collisions(panda,"panda_link8","panda_hand")
    disable_collisions(panda,"panda_hand","panda_leftfinger")
    disable_collisions(panda,"panda_hand","panda_rightfinger")
#to disable all self-collisions
#    simulation().getSpace().setEnablePair(panda.assembly.getName(), panda.assembly.getName(), False)
#    print("robot ", panda.assembly.getName())
#    print("ground ",ground.getName())
        
    # We add a relative transform to the end of the chain and will get a chain with
    # one additional link. This is so that we can make the position between the gripper fingers
    # the end of the chain. This simplifies which part of the robot we control.
    #
    chain = agxModel.SerialKinematicChain(simulation(), panda.base, panda.ee, panda.tooltip_offset)

    # Verify that a valid chain was found.
    if not chain.isValid():
        print("Error: a chain could not be found")
        application().stop()

    # Initial pose for the robot.
    set_robot_initial_pose(panda, chain, panda.q_ready)

    # Set material on the gripper fingers
    for name in ['panda_rightfinger', 'panda_leftfinger']:
        agxUtil.setBodyMaterial(panda.assembly.getRigidBody(name), finger_material)

    # Position camera
#    eye = agx.Vec3(1.93, -2.78, 0.93)
#    center = agx.Vec3(0.312, 0, 0.497)
#    up = agx.Vec3(-0.0251, 0.1424, 0.9895)
#    application().setCameraHome(eye, center, up)

    agxIO.writeFile("pick_scene.agx",simulation())

# Entry point when this script is started with python executable
init = init_app(
    name=__name__,
    scenes=[(buildScene, "1"),(buildBallToyScene, "2")],
    autoStepping=False,  # Default: False
    onInitialized=lambda app: print("App successfully initialized."),
    onShutdown=lambda app: print("App successfully shut down."),
)
