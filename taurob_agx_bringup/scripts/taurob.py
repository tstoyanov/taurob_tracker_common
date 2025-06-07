import os
import numpy as np
from typing import Iterable, Dict

import agx
import agxSDK
import agxOSG
import agxPython
import agxModel
import agxVehicle
import agxRender
import agxCollide
import agxUtil

from math import pi, radians, sin, cos, copysign
from ament_index_python import get_package_share_directory
from agxPythonModules.utils.environment import root
from collections import OrderedDict

# Track properties collection containing named track properties.
# Referenced as: TrackModelDescription { 'properties': 'trackProperties1' }.
TrackPropertiesCollection = {
    'trackProperties1': {
        'hinge compliance translational': 1.0E-10,
        'hinge compliance rotational': 2.0E-9,
        'hinge damping': 0.0333,
        'enable hinge range': False,
        'hinge range range': agx.RangeReal(-2.0 * pi / 3.0, radians(20)),
        'enable on initialize merge nodes to wheels': False,
        'enable on initialize transform nodes to wheels': True,
        'transform nodes to wheels overlap': 1.0E-3,
        'nodes to wheels merge threshold': -0.1,
        'nodes to wheels split threshold': -0.15,
        'num nodes included in average direction': 5,
        'min stabilizing hinge normal force': 100.0,
        'stabilizing hinge friction parameter': 6.0E-6
    }
}

# Track internal merge properties containing named internal merge properties.
# Referenced as: TrackModelDescription { 'internalMergeProperties': 'mergeProperties1' }.
TrackInternalMergePropertiesCollection = {
    'mergeProperties1': {
        'enable merge': True,
        'num nodes per merge segment': 3,
        'contact reduction': agxVehicle.TrackInternalMergeProperties.MODERATE,
        'enable lock to reach merge condition': True,
        'lock to reach merge condition compliance': 1.0E-11,
        'lock to reach merge condition damping': 0.05,
        'max angle merge condition': 1.0E-5
    }
}


# Helper to buid a wheel
def buildWheel(wheel_radius, wheel_height, wheel_type, wheel_pos):
    drive_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, 
            wheel_height)))
    drive_rb.setPosition(wheel_pos)
    agxOSG.setDiffuseColor(agxOSG.createVisual(drive_rb.getGeometries()[0], 
          root(), 1.0), agxRender.Color.Red())
    drive_wheel = agxVehicle.TrackWheel( wheel_type, wheel_radius, drive_rb)
    return drive_wheel

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
def get(name: str, desc: dict, model: dict):
    return desc.get(name, model[name])

# Adapted from agxPythonModels.robots.panda
class Joint():
    def __init__(self,
                 name: str,
                 hinge: agx.Hinge):
        self.name = name
        self.hinge = hinge
        self.parent = hinge.getBodyAt(1)
        self.child = hinge.getBodyAt(0)
        self.motor = hinge.getMotor1D()
        self.motor.setForceRange(agx.RangeReal(-np.inf, np.inf))
        self.lock = hinge.getLock1D()

    def get_angle(self) -> float:
        return self.hinge.getAngle()

    def get_speed(self) -> float:
        return self.hinge.getCurrentSpeed()

class Taurob():
  def __init__(self, **kwargs): 
               
    sim = kwargs.get('sim', None )
    disable_self_collision = kwargs.get('disable_self_collision', True)

    assembly_ref = self.load_from_urdf()
    assembly = assembly_ref.get()
    self.assembly = assembly
   
    # get all constraints in the assembly
    all_c = list(self.assembly.getConstraints())
 
    if disable_self_collision:
      sim.getSpace().setEnablePair(assembly.getName(), assembly.getName(), False)

    agxOSG.createVisual(assembly, agxPython.getContext().environment.getSceneRoot())

    self.chassis = assembly.getRigidBody('chassis_link')
    agxOSG.setDiffuseColor(agxOSG.createVisual(self.chassis, root()), agxRender.Color.LightGreen())

    #set track and wheel materials
    trackMaterial = agx.Material('track')
    wheelMaterial = agx.Material('wheel')

    trackWheelContactMaterial = sim.getMaterialManager().getOrCreateContactMaterial(trackMaterial,
                                                                                    wheelMaterial)  # type: agx.ContactMaterial
    trackWheelContactMaterial.setRestitution(0)
    trackWheelContactMaterial.setFrictionCoefficient(10)
    trackWheelContactMaterial.setYoungsModulus(4.0E8)
    trackWheelContactMaterial.setDamping(0.05)
    trackWheelContactMaterial.setAdhesion(0.0, 8.0E-3)
    trackWheelContactMaterial.setUseContactAreaApproach(False)
    
    #track properties
    wheel_radius = 0.0925
    wheel_height = 0.05
    track_nodes = 140
    track_width = 0.1
    track_thickness = 0.015
    flipper_link = assembly.getRigidBody("flipper_link_1")
    node_offset = 0.0002 

#    pos = agx.Vec3(0.37, 0.23375, 0.103)
#    axis = agx.Vec3(0,1,0)
#    hinge_frame = agx.HingeFrame(pos,axis)

    #####left track#####
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

    left_hinge = left_drive_wheel.attachHinge("left_wheel_joint", flipper_link, 0.0)
    left_hinge.getMotor1D().setEnable(True)  
    left_hinge.getMotor1D().setSpeed(1.0)  
    left_flipper_hinge = left_flipper_wheel.attachHinge("left_flipper_wheel_joint", self.chassis, 0.0)
    left_back_hinge = left_back_wheel.attachHinge("left_back_wheel_joint", self.chassis, 0.0)
    
    #set weheel materials
    for wheel in left_track.getWheels():
      agxUtil.setBodyMaterial(wheel.getRigidBody(), wheelMaterial)
    
    left_track.setMaterial(trackMaterial)
    assembly.add(left_track)

    ##### Right track ####
    right_drive_wheel = buildWheel(wheel_radius,wheel_height,
        agxVehicle.TrackWheel.SPROCKET,agx.Vec3(0.37, -0.23375, 0.103))
    right_back_wheel = buildWheel(wheel_radius,wheel_height,
        agxVehicle.TrackWheel.IDLER,agx.Vec3(-0.441, -0.23375, 0.103))
    right_flipper_wheel = buildWheel(wheel_radius,wheel_height,
        agxVehicle.TrackWheel.ROLLER,agx.Vec3(0.051, -0.23375, 0.103))
    
    right_track = agxVehicle.Track(track_nodes,track_width,track_thickness,node_offset)
    right_track.add(right_drive_wheel)
    right_track.add(right_flipper_wheel)
    right_track.add(right_back_wheel)
    right_track.initialize()
    ##set track properties##
    properties = TrackPropertiesCollection.get('trackProperties1', None)
    for name, value in properties.items():
        invokePropertyFunction(right_track.getProperties(), name, value)

    internalMergeProperties = TrackInternalMergePropertiesCollection.get('mergeProperties1', None)
    for name, value in internalMergeProperties.items():
        invokePropertyFunction(right_track.getInternalMergeProperties(), name, value)
    
    for node in right_track.nodes():
      agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.Black())

    right_hinge = right_drive_wheel.attachHinge("right_wheel_joint", flipper_link, 0.0)
    right_hinge.getMotor1D().setEnable(True)  
    right_hinge.getMotor1D().setSpeed(1.0)  
    right_flipper_hinge = right_flipper_wheel.attachHinge("right_flipper_wheel_joint", self.chassis, 0.0)
    right_back_hinge = right_back_wheel.attachHinge("right_back_wheel_joint", self.chassis, 0.0)
    
    #set weheel materials
    for wheel in right_track.getWheels():
      agxUtil.setBodyMaterial(wheel.getRigidBody(), wheelMaterial)
    
    right_track.setMaterial(trackMaterial)
    assembly.add(right_track)

####################################################################
    left_track.setEnableCollisions(self.chassis, disable_self_collision)
    right_track.setEnableCollisions(self.chassis, disable_self_collision)
       
    sim.add(assembly)
    self.tracks = [left_track, right_track]
    self.joints = [Joint(c.getName(), c.asHinge()) for c in all_c if c.asHinge()]
    self.locks = [c.asLockJoint() for c in all_c if c.asLockJoint()]
    

  ##### for arm ####
  def enable_motors(self, enable: bool) -> None:
      [j.motor.setEnable(enable) for j in self.joints]

  def enable_locks(self, enable: bool) -> None:
    [j.lock.setEnable(enable) for j in self.joints]

  def get_joint_names(self) -> list:
      return [j.name for j in self.joints]

  def get_joint_positions(self) -> np.ndarray:
      return np.array([j.get_angle() for j in self.joints])

  def set_joint_positions(self, q: Iterable[float]) -> None:
      for j, qi in zip(self.joints, q):
          j.lock.setPosition(qi)

  def get_joint_velocities(self) -> np.ndarray:
      return np.array([j.get_speed() for j in self.joints])

  def set_joint_velocities(self, qd: Iterable[float]) -> None:
      for i, qdi in enumerate(qd):
          self.joints[i].motor.setSpeed(qdi)

  @staticmethod
  def load_from_urdf() -> agxSDK.AssemblyRef:
    urdf_file = os.path.join(
        get_package_share_directory("taurob_agx_bringup"),
        "urdf",
        "taurob_static.urdf")
    (package_path, tail) = os.path.split(get_package_share_directory("taurob_agx_bringup")) #getting rid of a dupliacate

    print("package path:")
    print(package_path)
    
    urdf_settings = agxModel.UrdfReaderSettings(fixToWorld_=False,
                                                  disableLinkedBodies_=False,
                                                  mergeKinematicLinks_=False)
    robot_ref = agxModel.UrdfReader.read(urdf_file, package_path, None, urdf_settings)

    return robot_ref

