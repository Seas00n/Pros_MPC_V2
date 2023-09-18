

#include <string>

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

using namespace ocs2;

namespace
{
    scalar_t targetDisplacementVelocity;
    scalar_t targetRotationVelocity;
    scalar_t comHeight;
    vector_t defaultJointState(12);
} // namespace

scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
{
    const scalar_t &dx = desiredBaseDisplacement(0);
    const scalar_t &dy = desiredBaseDisplacement(1);
    const scalar_t &dyaw = desiredBaseDisplacement(3);
    const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
    const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
    const scalar_t displacementTime = displacement / targetDisplacementVelocity;
    return std::max(rotationTime, displacementTime);
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t &commadLineTarget, const SystemObservation &observation)
{
    const vector_t currentPose = observation.state.segment<6>(6);
    const vector_t targetPose = [&]()
    {
        vector_t target(6);
        // base p_x, p_y are relative to current state
        target(0) = currentPose(0) + commadLineTarget(0);
        target(1) = currentPose(1) + commadLineTarget(1);
        // base z relative to the default height
        target(2) = comHeight + commadLineTarget(2);
        // theta_z relative to current
        target(3) = currentPose(3) + commadLineTarget(3) * M_PI / 180.0;
        // theta_y, theta_x
        target(4) = currentPose(4);
        target(5) = currentPose(5);
        return target;
    }();

    // target reaching duration
    const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);

    // desired time trajectory
    const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

    // desired state trajectory
    vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
    stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState;
    stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState;

    // desired input trajectory (just right dimensions, they are not used)
    const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char *argv[])
{
    const std::string robotName = "legged_robot";

    // Initialize ros node
    ::ros::init(argc, argv, robotName + "_target");
    ::ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string referenceFile;
    nodeHandle.getParam("/referenceFile", referenceFile);

    loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
    loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState);
    loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity);

    // goalPose: [deltaX, deltaY, deltaZ, deltaYaw]
    const scalar_array_t relativeBaseLimit{10.0, 10.0, 0.2, 360.0};
    TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeBaseLimit, &commandLineToTargetTrajectories);

    const std::string commandMsg = "Enter XYZ and Yaw (deg) displacements for the TORSO, separated by spaces";
    targetPoseCommand.publishKeyboardCommand(commandMsg);

    // Successful exit
    return 0;
}
