import rospy

from model.Gait import Gait
from model.Joint import Joint
from model.Limits import Limits
from model.Setpoint import Setpoint


def empty_gait(robot, duration):
    if robot is None:
        rospy.logerr("Cannot create gait without a loaded robot.")
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
            Setpoint(0.2, 0, 0),
            Setpoint(3, 1.3, 0),
            Setpoint(4, 1.3, 0),
            Setpoint(duration, 0, 0)
        ]
        joint = Joint(urdf_joint.name,
                      Limits(urdf_joint.safety_controller.soft_lower_limit,
                             urdf_joint.safety_controller.soft_upper_limit,
                             urdf_joint.limit.velocity),
                      default_setpoints,
                      duration
                      )
        joint_list.append(joint)
    return Gait(joint_list, duration)


def from_matlab(robot, mat):
    if robot is None:
        rospy.logerr("Cannot create gait without a loaded robot.")
        return None

    duration = mat["stepTime"][0][0]

    # Map M3 names to ours
    expected_joints_map = {
        "hip": "_hip_fe",
        "knee": "_knee",
        "ankle": "_ankle"
    }

    joints = []
    for expected_joint in expected_joints_map:

        # Create actual joint names
        left_name = "left" + expected_joints_map[expected_joint]
        right_name = "right" + expected_joints_map[expected_joint]

        left_urdf_joint = get_joint_from_urdf(robot, left_name)
        right_urdf_joint = get_joint_from_urdf(robot, right_name)

        # Get limits from urdf
        left_limits = Limits(left_urdf_joint.limit.lower, left_urdf_joint.limit.upper, left_urdf_joint.limit.velocity)
        right_limits = Limits(right_urdf_joint.limit.lower, right_urdf_joint.limit.upper,
                              right_urdf_joint.limit.velocity)

        left_setpoints = []
        right_setpoints = []

        # Parse matlab file
        relative_time = mat[expected_joint][0][0][0][0]
        position = mat[expected_joint][0][0][1][0]
        velocity = mat[expected_joint][0][0][2][0]

        # Create setpoints
        for i in range(0, len(relative_time)):
            absolute_time = relative_time[i] / 100 * duration
            left_setpoints.append(Setpoint(absolute_time, position[i], velocity[i]))
            right_setpoints.append(Setpoint((absolute_time + duration / 2.0) % duration, position[i], velocity[i]))

        right_setpoints.sort(key=get_time)

        # Add a setpoint at the end to avoid interpolation issues
        last_left_setpoint = left_setpoints[-1]
        left_setpoints.append(Setpoint(duration, last_left_setpoint.position, last_left_setpoint.velocity))
        last_right_setpoint = right_setpoints[-1]
        right_setpoints.append(Setpoint(duration, last_right_setpoint.position, last_right_setpoint.velocity))

        joints.append(Joint(left_name, left_limits, left_setpoints, duration))
        joints.append(Joint(right_name, right_limits, right_setpoints, duration))

        default_setpoints = [
            Setpoint(0, 0, 0),
            Setpoint(duration, 0, 0)
        ]

        # Add hip_aa joints if present in the urdf
        left_hip_aa_urdf = get_joint_from_urdf(robot, "left_hip_aa")
        if left_hip_aa_urdf is not None:
            left_hip_aa = Joint("left_hip_aa",
                                Limits(left_hip_aa_urdf.safety_controller.soft_lower_limit,
                                       left_hip_aa_urdf.safety_controller.soft_upper_limit,
                                       left_hip_aa_urdf.limit.velocity),
                                default_setpoints,
                                duration
                                )
            joints.append(left_hip_aa)

        right_hip_aa_urdf = get_joint_from_urdf(robot, "right_hip_aa")
        if right_hip_aa_urdf is not None:
            right_hip_aa = Joint("right_hip_aa",
                                 Limits(right_hip_aa_urdf.safety_controller.soft_lower_limit,
                                        right_hip_aa_urdf.safety_controller.soft_upper_limit,
                                        right_hip_aa_urdf.limit.velocity),
                                 default_setpoints,
                                 duration
                                 )
            joints.append(right_hip_aa)
    return Gait(joints, duration, "Gait placeholder", "Subgait placeholder", "Version placeholder",
                "Description placeholder")


def get_time(setpoint):
    return setpoint.time


def from_msg(robot, march_gait, gait_name, subgait_name, version):
    if robot is None:
        rospy.logerr("Cannot create gait without a loaded robot.")
        return None

    user_defined_setpoints = march_gait.setpoints

    if not user_defined_setpoints:
        rospy.logwarn("Subgait is missing setpoints, assuming all trajectory points are setpoints")
    if len(march_gait.trajectory.points) < 2:
        rospy.logwarn("Cannot load gait as it has only %s setpoints instead of the minimal 2",
                      str(len(march_gait.trajectory.points)))
        return None
    joint_trajectory = march_gait.trajectory

    # Check if all joints in this gait exist in the robot, joints in the robot but not in the gait are allowed.
    for joint_name in joint_trajectory.joint_names:
        if not joint_exists(robot, joint_name):
            rospy.logerr("Joint " + joint_name + " not found in robot description")
            return None

    joint_list = []
    duration = rospy.Duration(march_gait.duration.secs, march_gait.duration.nsecs).to_sec()
    for joint_name in joint_trajectory.joint_names:
        setpoints = []
        if user_defined_setpoints:
            for actual_setpoint in user_defined_setpoints:
                if joint_name in actual_setpoint.joint_names:
                    setpoints.append(get_setpoint_at_duration(
                        joint_trajectory, joint_name, actual_setpoint.time_from_start))
        else:
            joint_index = joint_trajectory.joint_names.index(joint_name)

            for point in joint_trajectory.points:
                time = rospy.Duration(point.time_from_start.secs, point.time_from_start.nsecs).to_sec()
                setpoints.append(Setpoint(time, point.positions[joint_index], point.velocities[joint_index]))

        rospy.loginfo("Joint " + joint_name + " has setpoints " + str(setpoints))
        urdf_joint = get_joint_from_urdf(robot, joint_name)

        limits = Limits(urdf_joint.safety_controller.soft_lower_limit,
                        urdf_joint.safety_controller.soft_upper_limit,
                        urdf_joint.limit.velocity)
        joint = Joint(joint_name,
                      limits,
                      setpoints,
                      duration
                      )
        joint_list.append(joint)

    return Gait(joint_list, duration, gait_name, subgait_name, version, march_gait.description)


def get_setpoint_at_duration(joint_trajectory, joint_name, duration):
    for point in joint_trajectory.points:
        if point.time_from_start == duration:
            index = joint_trajectory.joint_names.index(joint_name)
            time = rospy.Duration(point.time_from_start.secs, point.time_from_start.nsecs).to_sec()

            return Setpoint(time, point.positions[index], point.velocities[index])
    return None


def joint_exists(robot, joint_name):
    return get_joint_from_urdf(robot, joint_name) is not None


def get_joint_from_urdf(robot, joint_name):
    for urdf_joint in robot.joints:
        if urdf_joint.name == joint_name:
            return urdf_joint
    return None
