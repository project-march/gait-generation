import rospy
from march_shared_classes.gait.subgait import Subgait
from march_shared_classes.gait.limits import Limits
from modifiable_joint_trajectory import ModifiableJointTrajectory
from modifiable_setpoint import ModifiableSetpoint


class ModifiableSubgait(Subgait):
    joint_class = ModifiableJointTrajectory

    @classmethod
    def empty_subgait(cls, gait_generator, robot, gait_type='walk_like', duration=8):
        if robot is None:
            rospy.logerr("Cannot create gait without a loaded robot.")
            return None
        joint_list = []
        for i in range(0, len(robot.joints)):
            urdf_joint = robot.joints[i]
            if urdf_joint.type == "fixed":
                rospy.loginfo("Skipping fixed joint " + urdf_joint.name)
                continue

            if urdf_joint.limit is None:
                rospy.logwarn("Skipping joint " + urdf_joint.name + " because it has no limits.")
                continue

            default_setpoints = [
                ModifiableSetpoint(0, 0, 0),
                ModifiableSetpoint(3, 1.3, 0),
                ModifiableSetpoint(4, 1.3, 0),
                ModifiableSetpoint(duration, 0, 0)
            ]
            joint = ModifiableJointTrajectory(urdf_joint.name,
                                              Limits(urdf_joint.safety_controller.soft_lower_limit,
                                                     urdf_joint.safety_controller.soft_upper_limit,
                                                     urdf_joint.limit.velocity),
                                              default_setpoints,
                                              duration,
                                              gait_generator
                                              )
            joint_list.append(joint)
        return cls(joint_list, duration, gait_type)

    @classmethod
    def from_file(cls, gait_generator, robot, filename):
        subgait = super(ModifiableSubgait, cls).from_file(robot, filename, gait_generator)
        if subgait is None:
            return
        return subgait

    def has_multiple_setpoints_before_duration(self, duration):
        for joint in self.joints:
            count = 0
            for setpoint in joint.setpoints:
                if setpoint.time <= duration:
                    count += 1
            if count < 2:
                return False
        return True

    def has_setpoints_after_duration(self, duration):
        for joint in self.joints:
            for setpoint in joint.setpoints:
                if setpoint.time > duration:
                    return True
        return False

    def can_mirror(self, key_1, key_2):
        if not key_1 or not key_2:
            rospy.loginfo("Keys are invalid")
            return False

        # XNOR, only one key can and must exist in the subgait name
        if (key_1 in self.subgait_name) == (key_2 in self.subgait_name):
            rospy.loginfo("Multiple or no keys exist in subgait %s", self.subgait_name)
            return False

        # If a joint name has both keys, we wouldn't know how to replace them.
        for joint in self.joints:
            if key_1 in joint.name and key_2 in joint.name:
                rospy.loginfo("Both keys exist in joint %s", joint.name)
                return False
            if key_1 in joint.name:
                joint_1 = joint
                joint_2 = self.get_joint(joint.name.replace(key_1, key_2))
            elif key_2 in joint.name:
                joint_1 = self.get_joint(joint.name.replace(key_2, key_1))
                joint_2 = joint
            else:
                continue

            if joint_1 is None or joint_2 is None:
                rospy.logwarn("Joints %s and %s are not valid.", str(joint_1), str(joint_2))
                return False

            if joint_1.setpoints[0].position != joint_2.setpoints[-1].position \
                    or joint_1.setpoints[0].velocity != joint_2.setpoints[-1].velocity:
                rospy.loginfo("First setpoint of %s != last setpoint of %s", joint_1.name, joint_2.name)
                return False
            if joint_1.setpoints[-1].position != joint_2.setpoints[0].position \
                    or joint_1.setpoints[-1].velocity != joint_2.setpoints[0].velocity:
                rospy.loginfo("Last setpoint of %s != first setpoint of %s", joint_1.name, joint_2.name)
                return False

        return True

    def get_mirror(self, key_1, key_2):
        if not self.can_mirror(key_1, key_2):
            rospy.logwarn("Cannot mirror gait %s", self.gait_name)
            return False

        if key_1 in self.subgait_name:
            mirrored_subgait_name = self.subgait_name.replace(key_1, key_2)
        elif key_2 in self.subgait_name:
            mirrored_subgait_name = self.subgait_name.replace(key_2, key_1)
        else:
            rospy.logerr("This case should have been caught by can_mirror()")
            return False

        mirrored_joints = []
        for joint in self.joints:
            if key_1 in joint.name:
                mirrored_name = str(joint.name.replace(key_1, key_2))
            elif key_2 in joint.name:
                mirrored_name = str(joint.name.replace(key_2, key_1))
            else:
                continue

            mirrored_joint = ModifiableJointTrajectory(mirrored_name, joint.limits, joint.setpoints, joint.duration)
            mirrored_joints.append(mirrored_joint)

        return ModifiableSubgait(mirrored_joints, self.duration, self.gait_type, self.gait_name, mirrored_subgait_name,
                                 self.version, self.description)

    def set_gait_type(self, gait_type):
        self.gait_type = str(gait_type)

    def set_gait_name(self, gait_name):
        self.gait_name = gait_name

    def set_description(self, description):
        self.description = str(description)

    def set_version(self, version):
        self.version = version

    def set_subgait_name(self, subgait_name):
        self.subgait_name = subgait_name

    def set_duration(self, duration, rescale=False):
        for joint in self.joints:
            # Loop in reverse to avoid out of bounds errors while deleting.
            for setpoint in reversed(joint.setpoints):
                if rescale:
                    setpoint.time = duration * setpoint.time / self.duration
                else:
                    if setpoint.time > duration:
                        joint.setpoints.remove(setpoint)
            joint.interpolated_setpoints = joint.interpolate_setpoints()

            joint.duration = duration

        self.duration = duration
