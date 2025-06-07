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

from taurob import Taurob

import math
import numpy as np

from collections import namedtuple

def disable_collisions(panda, linkA, linkB):
    simulation().getSpace().setEnablePair(linkA, linkB, False)
    for lA in panda.assembly.getRigidBody(linkA).getGeometries(): 
      for lB in panda.assembly.getRigidBody(linkB).getGeometries():
        simulation().getSpace().setEnableCollisions(lA, lB, False)


# Build robot scene
def buildScene():

    #add ground
    ground = agxCollide.Geometry(agxCollide.Box(30, 30, 0.1))
    ground.setPosition(agx.Vec3(0, 0, -0.1))
    ground.setName("ground")
    simulation().add(ground)
    ground_node = agxOSG.createVisual(ground, root())
    agxOSG.setDiffuseColor(ground_node, agxRender.Color.Black())

    # The robot
    #
    taurob = Taurob(sim=simulation(), disable_self_collision=True)
    #panda = Panda(simulation(), use_tool=True, disable_self_collision=False)

    # sets maximum torque
    #maximum_torque = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
    #for tau, i in zip(maximum_torque, range(7)):
       #print("Setting tau_max ",tau," for motor ",i)
    #    panda.assembly.getConstraint1DOF("panda_joint"+str(i+1)).getMotor1D().setForceRange(-tau, tau)

#    disable_collisions(panda,"panda_link0","panda_link1")
#to disable all self-collisions
#    simulation().getSpace().setEnablePair(panda.assembly.getName(), panda.assembly.getName(), False)
#    print("robot ", panda.assembly.getName())
#    print("ground ",ground.getName())
    
    #set contact materials
    trackMaterial = agx.Material('track')
    groundMaterial = agx.Material('ground')

    trackGroundContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(trackMaterial,
                                                                                              groundMaterial)  # type: agx.ContactMaterial
    trackGroundContactMaterial.setRestitution(0)
    trackGroundContactMaterial.setFrictionCoefficient(1.0, agx.ContactMaterial.PRIMARY_DIRECTION)
    trackGroundContactMaterial.setFrictionCoefficient(0.25, agx.ContactMaterial.SECONDARY_DIRECTION)
    trackGroundContactMaterial.setSurfaceViscosity(1.0E-6, agx.ContactMaterial.PRIMARY_DIRECTION)
    trackGroundContactMaterial.setSurfaceViscosity(6.0E-6, agx.ContactMaterial.SECONDARY_DIRECTION)

    trackGroundContactMaterial.setFrictionModel(agx.ConstantNormalForceOrientedBoxFrictionModel(0.5 * taurob.chassis.getMassProperties().getMass(),
                                                                                                taurob.chassis.getFrame(),
                                                                                                agx.Vec3.X_AXIS(),
                                                                                                agx.FrictionModel.DIRECT,
                                                                                                False))
    
    ground.setMaterial(groundMaterial)

    for track in taurob.tracks:
        track.setMaterial(trackMaterial)
    

    taurob.enable_motors(True)
    joint_names = taurob.get_joint_names()
    joint_positions = taurob.get_joint_velocities()
    for jp,jn in zip(joint_positions, joint_names):
      print ("joint",jp,"at",jn)

    joint_positions[0] = -0.5
    taurob.set_joint_velocities(joint_positions)

    agxIO.writeFile("taurob_scene.agx",simulation())

def onAppInitialized(app: agxOSG.ExampleApplication):
    cameraData = app.getCameraData()
    cameraData.eye = agx.Vec3(3.6617, -0.5055, 1.3403)
    cameraData.center = agx.Vec3(-0.3101, 0.0829, 0.1390)
    cameraData.up = agx.Vec3(-0.2714, 0.1179, 0.9552)
    cameraData.nearClippingPlane = 0.1
    cameraData.farClippingPlane = 5000
    app.applyCameraData(cameraData)

    app.getSceneDecorator().setEnableShadows(True)
    app.getSceneDecorator().setShadowMethod(agxOSG.SceneDecorator.SOFT_SHADOWMAP)

    def toVec3(v):
        return agx.Vec3(v[0], v[1], v[2])
    app.getSceneDecorator().setBackgroundColor(toVec3(agxRender.Color.SkyBlue()), toVec3(agxRender.Color.DodgerBlue()))


# Entry point when this script is started with python executable
init = init_app(
    name=__name__,
    scenes=[(buildScene, "1")],
    autoStepping=False,  # Default: False
    onInitialized=onAppInitialized,
    onShutdown=lambda app: print("App successfully shut down."),
)
