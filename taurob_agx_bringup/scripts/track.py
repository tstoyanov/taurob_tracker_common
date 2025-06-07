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
import agxVehicle

from agxPythonModules.utils.environment import simulation, init_app, application, root
from agxPythonModules.robots.panda import Panda  # noqa

from math import pi, radians, sin, cos, copysign

import math
import numpy as np

from collections import namedtuple

def disable_collisions(panda, linkA, linkB):
    simulation().getSpace().setEnablePair(linkA, linkB, False)
    for lA in panda.assembly.getRigidBody(linkA).getGeometries(): 
      for lB in panda.assembly.getRigidBody(linkB).getGeometries():
        simulation().getSpace().setEnableCollisions(lA, lB, False)

# Track properties collection containing named track properties.
# Referenced as: TrackModelDescription { 'properties': 'trackProperties1' }.
TrackPropertiesCollection = {
    'trackProperties1': {
        'hinge compliance translational': 1.0E-10,
        'hinge compliance rotational': 2.0E-9,
        'hinge damping': 0.0333,
        'enable hinge range': True,
        'hinge range range': agx.RangeReal(-2.0 * pi / 3.0, radians(20)),
        'enable on initialize merge nodes to wheels': False,
        'enable on initialize transform nodes to wheels': True,
        'transform nodes to wheels overlap': 1.0E-3,
        'nodes to wheels merge threshold': -0.1,
        'nodes to wheels split threshold': -0.15,
        'num nodes included in average direction': 5,
        'min stabilizing hinge normal force': 100.0,
        'stabilizing hinge friction parameter': 4.0E-6
    }
}

# Track internal merge properties containing named internal merge properties.
# Referenced as: TrackModelDescription { 'internalMergeProperties': 'mergeProperties1' }.
TrackInternalMergePropertiesCollection = {
    'mergeProperties1': {
        'enable merge': False,
        'num nodes per merge segment': 3,
        'contact reduction': agxVehicle.TrackInternalMergeProperties.AGGRESSIVE,
        'enable lock to reach merge condition': True,
        'lock to reach merge condition compliance': 1.0E-10,
        'lock to reach merge condition damping': 0.05,
        'max angle merge condition': 1.0E-5
    }
}
# Given e.g., fullName = "hinge compliance rotational" this function converts the
# string to functionName = "setHingeComplianceRotational" and verifies that method
# is present in type( instance ).
def invokePropertyFunction(instance, fullName, value):
    functionName = 'set' + fullName.title().replace(' ', '')
    if functionName not in type(instance).__dict__:
        print('Unable to find function "%s" in "%s". Ignoring "%s".' % (functionName, type(instance).__name__, fullName))
        return

    # This doesn't work in general with multiple arguments, so handling some
    # cases explicitly.
    if type(value) is agx.RangeReal:
        getattr(instance, functionName)(value.lower(), value.upper())
    else:
        getattr(instance, functionName)(value)

def buildWheel(wheel_radius, wheel_height, wheel_type, wheel_pos):
    drive_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
    drive_rb.setPosition(wheel_pos)
    agxOSG.setDiffuseColor(agxOSG.createVisual(drive_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Red())
    drive_wheel = agxVehicle.TrackWheel( wheel_type, wheel_radius, drive_rb)
    return drive_wheel

def buildScene():
    #track properties
    wheel_radius = 0.5
    wheel_height = 0.3
    track_nodes = 120
    track_width = 0.3
    track_thickness = 0.05
    node_offset = 0

    drive_wheel = buildWheel(0.5,0.3,agxVehicle.TrackWheel.SPROCKET,agx.Vec3(0,0,0))
#    roll1 = buildWheel(0.5,0.3,agxVehicle.TrackWheel.ROLLER,agx.Vec3(2,0,0))
#    roll2 = buildWheel(0.5,0.3,agxVehicle.TrackWheel.ROLLER,agx.Vec3(4,0,0))
#    roll3 = buildWheel(0.5,0.3,agxVehicle.TrackWheel.ROLLER,agx.Vec3(6,0,0))
    idler = buildWheel(0.5,0.3,agxVehicle.TrackWheel.IDLER,agx.Vec3(8,0,0))

    track = agxVehicle.Track(track_nodes,track_width,track_thickness,node_offset)
    track.add(drive_wheel)
#    track.add(roll1)
#    track.add(roll2)
#    track.add(roll3)
    track.add(idler)

    track.initialize()
    ##set track properties##
#   properties = TrackPropertiesCollection.get('trackProperties1', None)
#   for name, value in properties.items():
#       invokePropertyFunction(track.getProperties(), name, value)

#   internalMergeProperties = TrackInternalMergePropertiesCollection.get('mergeProperties1', None)
#   for name, value in internalMergeProperties.items():
#       invokePropertyFunction(track.getInternalMergeProperties(), name, value)
#   
    for node in track.nodes():
      agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.LightGreen())

    for wheel in track.getWheels():
      hinge = wheel.attachHinge("wheel_joint", None, 0.0)
    
    track.findReferenceWheel().getConstraint().asHinge().getMotor1D().setEnable(True)
    track.findReferenceWheel().getConstraint().asHinge().getMotor1D().setSpeed(1.0)
    simulation().add(track)

    

# Build robot scene
def buildScene2():

    #add ground
#   ground = agxCollide.Geometry(agxCollide.Box(30, 30, 0.1))
#   ground.setPosition(agx.Vec3(0, 0, -0.1))
#   ground.setName("ground")
#   simulation().add(ground)
#   ground_node = agxOSG.createVisual(ground, root())
#   agxOSG.setDiffuseColor(ground_node, agxRender.Color.Black())
#   
#   assembly = agxSDK.Assembly()
#   chassisExtents = agx.Vec3(1, 0.15, 0.1)
#   chassis = agx.RigidBody('chassis')
#   chassis.add(agxCollide.Geometry(agxCollide.Box(chassisExtents)))
#   chassis.setPosition(0,0,0.15)

#   agxOSG.setDiffuseColor(agxOSG.createVisual(chassis, root()),agxRender.Color.LightGreen())
#  
#   assembly.add(chassis) 
    
    #track properties
    wheel_radius = 0.0925
    wheel_height = 0.05
    track_nodes = 100
    track_width = 2*wheel_height
    track_thickness = 0.01
    node_offset = 0

    left_drive_wheel = buildWheel(wheel_radius,wheel_height,
        agxVehicle.TrackWheel.SPROCKET,agx.Vec3(0.37, 0.23375, 0.103))
    left_back_wheel = buildWheel(wheel_radius,wheel_height,
        agxVehicle.TrackWheel.IDLER,agx.Vec3(-0.441, 0.23375, 0.103))
    left_flipper_wheel = buildWheel(wheel_radius,wheel_height,
        agxVehicle.TrackWheel.ROLLER,agx.Vec3(0.051, 0.23375, 0.103))
    
    left_track = agxVehicle.Track(track_nodes,track_width,track_thickness,node_offset)
    left_track.add(left_drive_wheel)
    left_track.add(left_flipper_wheel)
    left_track.add(left_back_wheel)
    
    left_track.initialize()

    ##set track properties##
    properties = TrackPropertiesCollection.get('trackProperties1', None)
    for name, value in properties.items():
        invokePropertyFunction(left_track.getProperties(), name, value)

    internalMergeProperties = TrackInternalMergePropertiesCollection.get('mergeProperties1', None)
    for name, value in internalMergeProperties.items():
        invokePropertyFunction(left_track.getInternalMergeProperties(), name, value)
    
    for node in left_track.nodes():
      agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.Black())

    for wheel in left_track.getWheels():
      hinge = wheel.attachHinge("wheel_joint", None, 0.0)
    
    left_track.findReferenceWheel().getConstraint().asHinge().getMotor1D().setEnable(True)
    left_track.findReferenceWheel().getConstraint().asHinge().getMotor1D().setSpeed(1.0)
    simulation().add(left_track)

#   left_drive_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius+0.05, wheel_height)))
#   left_drive_rb.setPosition(agx.Vec3(0.37, 0.23375, 0.103))
#   #note: set position relative to?
#   agxOSG.setDiffuseColor(agxOSG.createVisual(left_drive_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Red())
#   left_sprocket = agxVehicle.TrackWheel( agxVehicle.TrackWheel.SPROCKET, wheel_radius+0.05, left_drive_rb)
#   left_hinge = left_sprocket.attachHinge("left_wheel_joint", None, 0.05)
#   left_hinge.getMotor1D().setEnable(True)  
#    left_hinge.getMotor1D().setCompliance(1e-6)  
#   left_hinge.getMotor1D().setSpeed(0.5)  
#   
#   left_flipper_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
#   left_flipper_rb.setPosition(agx.Vec3(0.051, 0.23375, 0.103))
#   agxOSG.setDiffuseColor(agxOSG.createVisual(left_flipper_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Salmon())
#   left_flipper_idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.IDLER, wheel_radius, left_flipper_rb)
#    left_flipper_idler.setEnableProperty(agxVehicle.TrackWheel.MOVE_NODES_TO_ROTATION_PLANE,True)
#   left_flipper_idler.setEnableProperty(agxVehicle.TrackWheel.SPLIT_SEGMENTS,True)
#   left_flipper_hinge = left_flipper_idler.attachHinge("left_flipper_joint", None, 0.05)
#    left_flipper_hinge.getMotor1D().setForceRange(agx.RangeReal(-np.inf,np.inf))
#    left_flipper_hinge.getMotor1D().setEnable(True)
#    left_flipper_hinge.getLock1D().setEnable(False)
 
#   left_back_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
#   left_back_rb.setPosition(agx.Vec3(-0.441, 0.23375, 0.103))
#   agxOSG.setDiffuseColor(agxOSG.createVisual(left_back_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Salmon())
#    left_back_idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.SPROCKET, wheel_radius, left_back_rb)
#    left_back_idler.setEnableProperty(agxVehicle.TrackWheel.MOVE_NODES_TO_ROTATION_PLANE,True)
#   left_back_idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.ROLLER, wheel_radius, left_back_rb)
#   left_back_idler.setEnableProperty(agxVehicle.TrackWheel.SPLIT_SEGMENTS,True)
#   left_back_hinge = left_back_idler.attachHinge("left_back_joint", chassis, 0.0)
#    left_back_hinge.getMotor1D().setEnable(True)
#    left_back_hinge.getMotor1D().setForceRange(agx.RangeReal(0,0))
#    left_back_hinge.getMotor1D().setSpeed(1.0)  
#    left_back_hinge.getLock1D().setEnable(False)

#   leftTrack = agxVehicle.Track(track_nodes,track_width,track_thickness,node_offset)
#   leftTrack.add(left_sprocket)
#   leftTrack.add(left_flipper_idler)
#    leftTrack.add(left_back_idler)
#   
#   leftTrack.initialize()
#   ##set track properties##
#   properties = TrackPropertiesCollection.get('trackProperties1', None)
#   for name, value in properties.items():
#       invokePropertyFunction(leftTrack.getProperties(), name, value)

#   internalMergeProperties = TrackInternalMergePropertiesCollection.get('mergeProperties1', None)
#   for name, value in internalMergeProperties.items():
#       invokePropertyFunction(leftTrack.getInternalMergeProperties(), name, value)
#   
#   for node in leftTrack.nodes():
#     agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.Black())
#   simulation().add(leftTrack)
#   
#   #set track and wheel materials
    trackMaterial = agx.Material('track')
    wheelMaterial = agx.Material('wheel')

    trackWheelContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(trackMaterial,
                                                                                             wheelMaterial)  # type: agx.ContactMaterial
    trackWheelContactMaterial.setRestitution(0)
    trackWheelContactMaterial.setFrictionCoefficient(10)
    trackWheelContactMaterial.setYoungsModulus(4.0E8)
    trackWheelContactMaterial.setDamping(0.05)
    trackWheelContactMaterial.setAdhesion(0.0, 8.0E-3)
    trackWheelContactMaterial.setUseContactAreaApproach(False)
    
    #set weheel materials
    for wheel in left_track.getWheels():
      agxUtil.setBodyMaterial(wheel.getRigidBody(), wheelMaterial)
    
    left_track.setMaterial(trackMaterial)
#   

#   #interaction between track and ground
#   groundMaterial = agx.Material('ground')
#   trackGroundContactMaterial = simulation().getMaterialManager().getOrCreateContactMaterial(trackMaterial,
#                                                                                             groundMaterial)
#   trackGroundContactMaterial.setRestitution(0)
#   trackGroundContactMaterial.setFrictionCoefficient(1.0, agx.ContactMaterial.PRIMARY_DIRECTION)
#   trackGroundContactMaterial.setFrictionCoefficient(0.25, agx.ContactMaterial.SECONDARY_DIRECTION)
#   trackGroundContactMaterial.setSurfaceViscosity(1.0E-6, agx.ContactMaterial.PRIMARY_DIRECTION)
#   trackGroundContactMaterial.setSurfaceViscosity(6.0E-6, agx.ContactMaterial.SECONDARY_DIRECTION)

#   assembly.add(leftTrack)
#   simulation().add(assembly)

def onAppInitialized(app: agxOSG.ExampleApplication):
    cameraData = app.getCameraData()
    cameraData.eye = agx.Vec3(3.6617, -0.5055, 1.3403)
    cameraData.center = agx.Vec3(-0.3101, 0.0829, 0.1390)
    cameraData.up = agx.Vec3(-0.2714, 0.1179, 0.9552)
#   cameraData.eye = agx.Vec3(9.0474, 11.7681, 3.8213)
#   cameraData.center = agx.Vec3(2.9829, -1.6921, -0.1307)
#   cameraData.up = agx.Vec3(-0.0320, -0.2683, 0.9628)

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
    scenes=[(buildScene2, "1"),(buildScene, "2")],
    autoStepping=False,  # Default: False
    onInitialized=onAppInitialized,
    onShutdown=lambda app: print("App successfully shut down."),
)
