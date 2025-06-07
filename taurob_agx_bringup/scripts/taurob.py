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

#adapted from tutorial_trackedVehicle

# Track wheel models descriptions including some more properties than
# the agxVehicle.TrackWheel. Possible to add more models as well,
# e.g., 'turn-back-roller' or any of different radius or height.
# Schema:
#     <str>: {
#         'type'    : agxVehicle.TrackWheel.Model,
#         'radius'  : <float>
#         'height'  : <float>
#         'position>: <agx.Vec3> Should be given in the TrackModelDescription.wheels.
#         'color'   : <agxRender.Color> [optional]
#     }
TrackWheelModelDescription = {
    'sprocket': {
        'type': agxVehicle.TrackWheel.SPROCKET,
        'radius': 0.13,
        'height': 0.0925,
        'position': agx.Vec3(),
        'color': agxRender.Color.Salmon()
    },
    'idler': {
        'type': agxVehicle.TrackWheel.IDLER,
        'radius': 0.13,
        'height': 0.0925,
        'position': agx.Vec3(),
        'color': agxRender.Color.Yellow()
    },
    'roller': {
        'type': agxVehicle.TrackWheel.ROLLER,
        'radius': 0.13,
        'height': 0.0925,
        'position': agx.Vec3(),
        'color': agxRender.Color.WhiteSmoke()
    }
}


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
        'stabilizing hinge friction parameter': 5.0E-3
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

TrackModelDescription = {
    'numberOfNodes': 40,
    'width': 0.1,
    'thickness': 0.01,
    'initialTensionDistance': 0.001,
    'color': agxRender.Color.AliceBlue(),
    'properties': 'trackProperties1',
    'internalMergeProperties': 'mergeProperties1',
    'wheels': [
        {
            'name': 'drive_wheel',
            'model': TrackWheelModelDescription['sprocket'],
            'position': agx.Vec3(0.0, 0.0, 0.0),
            'property': {
                agxVehicle.TrackWheel.MOVE_NODES_TO_ROTATION_PLANE: True
            }
        },
        {
            'name': 'front_idler',
            'model': TrackWheelModelDescription['idler'],
            'position': agx.Vec3(0.319, 0, 0.0),
            'property': {
                agxVehicle.TrackWheel.MOVE_NODES_TO_ROTATION_PLANE: True
            }
        },
        {
            'name': 'back_idler',
            'model': TrackWheelModelDescription['idler'],
            'position': agx.Vec3(-0.492, 0, 0),
            'property': {
                agxVehicle.TrackWheel.MOVE_NODES_TO_ROTATION_PLANE: True
            }
        }
    ],
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
def get(name: str, desc: dict, model: dict):
    return desc.get(name, model[name])

class TrackWheel(agxVehicle.TrackWheel):
    def __init__(self, **kwargs):
        try:
            self.name = kwargs.get('name', '')
            self.model = kwargs['model']
            self.radius = get('radius', kwargs, self.model)
            self.height = get('height', kwargs, self.model)
            modelType = get('type', kwargs, self.model)

            visualGeometries = []
            # Using simplified shape sphere for rollers for increased performance and
            # better stability. A cylinder is created but only used for visual.
            if modelType == agxVehicle.TrackWheel.ROLLER:
                rollerCylinder = agxCollide.Geometry(agxCollide.Cylinder(self.radius, self.height))
                rollerSphere = agxCollide.Geometry(agxCollide.Sphere(self.radius))
                rollerCylinder.setEnableCollisions(False)
                rb = agx.RigidBody()
                rb.add(rollerSphere)
                rb.add(rollerCylinder)
                visualGeometries.append(rollerCylinder)
            else:
                rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(self.radius, self.height)))
                visualGeometries.append(rb.getGeometries()[0])

            rb.setPosition(get('position', kwargs, self.model))

            for visualGeometry in visualGeometries:
                agxOSG.setDiffuseColor(agxOSG.createVisual(visualGeometry, root(), 1.0),
                                       get('color', kwargs, self.model))

            # Base agxVehicle.TrackWheel constructor given agxVehicle.TrackWheel.Model,
            # radius and rigid body.
            super().__init__(modelType, self.radius, rb)

            if 'property' in kwargs:
                for prop, value in kwargs['property'].items():
                    self.setEnableProperty(prop, value)

        except KeyError as keyError:
            print('\nMandatory key not found in wheel: %s\n' % keyError)

# Track class with native agxVehicle.Track as base. This object contains
# some more information such as vehicle, suspension, TrackWheel list.
class Track(agxVehicle.Track):
    def __init__(self, **kwargs):
        try:
            kwargs.update(kwargs['model'])
            super().__init__(kwargs['numberOfNodes'],
                             kwargs['width'],
                             kwargs['thickness'],
                             kwargs['initialTensionDistance'])

            self.vehicle = kwargs['vehicle']
            self.wheels = []  # type: [TrackWheel]

            # Create TrackWheel instances and store them in a separate list.
            for wheelDef in kwargs['wheels']:
                self.wheels.append(TrackWheel(**wheelDef))

            # Add the wheels to the track <-> agxVehicle.Track.add( self, wheel ).
            for wheel in self.wheels:
                self.add(wheel)

            # Assuming vehicle forward along +x with vehicle center (0, 0, 0) during
            # this modelling stage. Track position y gives if this track is the
            # left or the right one.
            trackPosition = kwargs.get('position', agx.Vec3())

            self._color = kwargs.get('color', agxRender.Color.Wheat())

        except KeyError as keyError:
            print('\nMandatory key not found in track: %s\n' % keyError)

        try:

            self.setPosition(trackPosition)

            for wheel in self.wheels:
                if wheel.getConstraint() is None:
                    wheel.attachHinge(wheel.name, self.chassis, 0.0)

        except KeyError as keyError:
            print('\nMandatory key not found in suspension: %s\n' % keyError)

    @property
    def chassis(self) -> agx.RigidBody:
        return self.vehicle.chassis if self.vehicle else None

    def createVisual(self):
        for node in self.nodes():
            agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), self._color)

    def setWheelMaterial(self, material: agx.Material):
        for wheel in self.wheels:
            agxUtil.setBodyMaterial(wheel.getRigidBody(), material)

    def getWheel(self, name: str) -> TrackWheel:
        try:
            return [wheel for wheel in self.wheels if wheel.name == name][0]
        except Exception as e:
            return None


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

#   trackDefs = [
#       {
#           'name': 'right',
#           'model': TrackModelDescription,
#           'position': agx.Vec3(-0.051, 0.35, 0)
#       },
#       {
#           'name': 'left',
#           'model': TrackModelDescription,
#           'position': agx.Vec3(-0.051, -0.35, 0)
#       }
#   ]
#   self._tracks = OrderedDict()
#   for trackDef in trackDefs:
#       trackDef['vehicle'] = self
#       self._setTrack(Track(**trackDef), trackDef['name'])

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
    track_nodes = 120
    track_width = 0.1
    track_thickness = 0.015

    #####left track#####
    left_drive_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
    #note: set position relative to?
    left_drive_rb.setPosition(agx.Vec3(0.37, 0.23375, 0.103))
    agxOSG.setDiffuseColor(agxOSG.createVisual(left_drive_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Red())
    left_sprocket = agxVehicle.TrackWheel( agxVehicle.TrackWheel.SPROCKET, wheel_radius, left_drive_rb)
    
    left_flipper = assembly.getRigidBody("flipper_link_1")
    left_flipper_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
    left_flipper_rb.setPosition(agx.Vec3(0.051, 0.23375, 0.103))
    agxOSG.setDiffuseColor(agxOSG.createVisual(left_flipper_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Salmon())
    left_flipper_idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.ROLLER, wheel_radius, left_flipper_rb)
#   left_flipper_hinge.getMotor1D().setEnable(True)  
#   left_flipper_hinge.getMotor1D().setForceRange(agx.RangeReal(-np.inf,np.inf))
#   left_flipper_hinge.getLock1D().setEnable(False)
 
    left_back_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
    left_back_rb.setPosition(agx.Vec3(-0.441, 0.23375, 0.103))
    agxOSG.setDiffuseColor(agxOSG.createVisual(left_back_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Salmon())
    left_back_idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.IDLER, wheel_radius, left_back_rb)
#   left_back_hinge.getMotor1D().setEnable(True)  
#   left_back_hinge.getMotor1D().setForceRange(agx.RangeReal(-np.inf,np.inf))
#   left_back_hinge.getLock1D().setEnable(False)

    leftTrack = agxVehicle.Track(track_nodes,track_width,track_thickness)
    leftTrack.add(left_sprocket)
    leftTrack.add(left_flipper_idler)
    leftTrack.add(left_back_idler)
    leftTrack.initialize()
    for node in leftTrack.nodes():
      agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.Black())

    left_hinge = left_sprocket.attachHinge("left_wheel_joint", self.chassis, 0.0)
    left_hinge.getMotor1D().setEnable(True)  
    left_hinge.getMotor1D().setSpeed(1.0)  
    left_flipper_hinge = left_flipper_idler.attachHinge("left_flipper_wheel_joint", left_flipper, 0.0)
    left_back_hinge = left_back_idler.attachHinge("left_back_wheel_joint", self.chassis, 0.0)

    ##set track properties##
#   properties = TrackPropertiesCollection.get('trackProperties1', None)
#   for name, value in properties.items():
#       invokePropertyFunction(leftTrack.getProperties(), name, value)

#   internalMergeProperties = TrackInternalMergePropertiesCollection.get('mergeProperties1', None)
#   for name, value in internalMergeProperties.items():
#       invokePropertyFunction(leftTrack.getInternalMergeProperties(), name, value)
    
    for node in leftTrack.nodes():
      agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.Black())
    
    #set weheel materials
    for wheel in [left_drive_rb, left_flipper_rb, left_back_rb] :
      agxUtil.setBodyMaterial(wheel, wheelMaterial)
    
    leftTrack.setMaterial(trackMaterial)
    assembly.add(leftTrack)

    ##### Right track ####
    right_drive_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
    #note: set position relative to?
    right_drive_rb.setPosition(agx.Vec3(0.37, -0.23375, 0.103))
    agxOSG.setDiffuseColor(agxOSG.createVisual(right_drive_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Red())
    right_sprocket = agxVehicle.TrackWheel( agxVehicle.TrackWheel.SPROCKET, wheel_radius, right_drive_rb)
    
    right_flipper = assembly.getRigidBody("flipper_link_1")
    right_flipper_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
    right_flipper_rb.setPosition(agx.Vec3(0.051, -0.23375, 0.103))
    agxOSG.setDiffuseColor(agxOSG.createVisual(right_flipper_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Salmon())
    right_flipper_idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.IDLER, wheel_radius, right_flipper_rb)
#   right_flipper_hinge.getMotor1D().setEnable(False)  
#   right_flipper_hinge.getLock1D().setEnable(False)
#   right_flipper_hinge.getMotor1D().setForceRange(agx.RangeReal(0,0))
 
    right_back_rb = agx.RigidBody(agxCollide.Geometry(agxCollide.Cylinder(wheel_radius, wheel_height)))
    right_back_rb.setPosition(agx.Vec3(-0.441, -0.23375, 0.103))
    agxOSG.setDiffuseColor(agxOSG.createVisual(right_back_rb.getGeometries()[0], root(), 1.0), agxRender.Color.Salmon())
    right_back_idler = agxVehicle.TrackWheel( agxVehicle.TrackWheel.IDLER, wheel_radius, right_back_rb)
#   right_back_hinge.getMotor1D().setEnable(False)  
#   right_back_hinge.getLock1D().setEnable(False)
#   right_back_hinge.getMotor1D().setForceRange(agx.RangeReal(0,0))

    rightTrack = agxVehicle.Track(track_nodes,track_width,track_thickness)
    rightTrack.add(right_sprocket)
    rightTrack.add(right_flipper_idler)
    rightTrack.add(right_back_idler)
    rightTrack.initialize()
    for node in rightTrack.nodes():
      agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.Black())

    right_hinge = right_sprocket.attachHinge("right_wheel_joint", self.chassis, 0.0)
    right_hinge.getMotor1D().setEnable(True)  
    right_hinge.getMotor1D().setSpeed(1.0)  
    right_flipper_hinge = right_flipper_idler.attachHinge("right_flipper_wheel_joint", right_flipper, 0.0)
    right_back_hinge = right_back_idler.attachHinge("right_back_wheel_joint", self.chassis, 0.0)

    ##set track properties##
#   properties = TrackPropertiesCollection.get('trackProperties1', None)
#   for name, value in properties.items():
#       invokePropertyFunction(rightTrack.getProperties(), name, value)

#   internalMergeProperties = TrackInternalMergePropertiesCollection.get('mergeProperties1', None)
#   for name, value in internalMergeProperties.items():
#       invokePropertyFunction(rightTrack.getInternalMergeProperties(), name, value)
#   
#   for node in rightTrack.nodes():
#     agxOSG.setDiffuseColor(agxOSG.createVisual(node.getRigidBody(), root()), agxRender.Color.Black())
    
    #set weheel materials
    for wheel in [right_drive_rb, right_flipper_rb, right_back_rb] :
      agxUtil.setBodyMaterial(wheel, wheelMaterial)
    assembly.add(rightTrack)

    sim.add(assembly)

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

