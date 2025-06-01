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

    chassisBaseHalfExtents = kwargs.get('size', agx.Vec3(4, 2, 1)) * 0.5
    chassisTopHalfExtents = agx.Vec3(0.8 * chassisBaseHalfExtents.x(),
                                     0.8 * chassisBaseHalfExtents.y(),
                                     1.0 * chassisBaseHalfExtents.z())

    self.chassis = assembly.getRigidBody('chassis')
    agxOSG.setDiffuseColor(agxOSG.createVisual(self.chassis, root()),
                           kwargs.get('color', agxRender.Color.LightGreen()))

    trackDefs = [
        {
            'name': 'right',
            'model': TrackModelDescription,
            'position': agx.Vec3(-0.051, 0.35, 0)
        },
        {
            'name': 'left',
            'model': TrackModelDescription,
            'position': agx.Vec3(-0.051, -0.35, 0)
        }
    ]
    self._tracks = OrderedDict()
    for trackDef in trackDefs:
        trackDef['vehicle'] = self
        self._setTrack(Track(**trackDef), trackDef['name'])

    sim.add(assembly)

    self.joints = [Joint(c.getName(), c.asHinge()) for c in all_c if c.asHinge()]
    self.locks = [c.asLockJoint() for c in all_c if c.asLockJoint()]
    

  ############  methods  ################
  @property
  def tracks(self) -> [Track]:
      return [track for _, track in self._tracks.items()]

  def getTrack(self, name: str) -> Track:
      try:
          return self._tracks[name]
      except:  # noqa
          return None

  def _setTrack(self, track: Track, name: str):
      if name in self._tracks:
          self._setEnableCollisions(self._tracks[name], True)
          self.remove(self._tracks[name])

      self._tracks[name] = track

      if track:
          self._setEnableCollisions(self._tracks[name], False)
          self.assembly.add(track)

  def _setEnableCollisions(self, track: Track, enable: bool):
      track.setEnableCollisions(self.chassis, enable)

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

