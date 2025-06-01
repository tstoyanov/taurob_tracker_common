import os
import numpy as np
from typing import Iterable, Dict

import agx
import agxSDK
import agxOSG
import agxPython
import agxModel

from ament_index_python import get_package_share_directory

#from launch.substitutions import PathJoinSubstitution
#from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

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
  def __init__(self,
               sim: agxSDK.Simulation,
               disable_self_collision: bool = True):

    assembly_ref = self.load_from_urdf()
    assembly = assembly_ref.get()
    self.assembly = assembly
   
    # get all constraints in the assembly
    all_c = list(self.assembly.getConstraints())
 
    if disable_self_collision:
      sim.getSpace().setEnablePair(assembly.getName(), assembly.getName(), False)

    agxOSG.createVisual(assembly, agxPython.getContext().environment.getSceneRoot())

    sim.add(assembly)

    self.joints = [Joint(c.getName(), c.asHinge()) for c in all_c if c.asHinge()]
    self.locks = [c.asLockJoint() for c in all_c if c.asLockJoint()]

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

