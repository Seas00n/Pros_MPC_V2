#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pros_interface2/ProsWalkerInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_pros_controllers2/visualization/ProsWalkerVisualizer.h"

using namespace ocs2;
using namespace pros_walker;

int main(int argc, char **argv)
{
    const std::string robotName = "legged_robot";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mrt");
    ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string taskFile, urdfFile, referenceFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/urdfFile", urdfFile);
    nodeHandle.getParam("/referenceFile", referenceFile);

    // Robot interface
    LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

    // MRT
    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(nodeHandle);

    // Visualization
    CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
    PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                         interface.modelSettings().contactNames3DoF);
    auto leggedRobotVisualizer = std::make_shared<LeggedRobotVisualizer>(
        interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle);

    // Dummy legged robot
    MRT_ROS_Dummy_Loop leggedRobotDummySimulator(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                                                 interface.mpcSettings().mpcDesiredFrequency_);
    leggedRobotDummySimulator.subscribeObservers({leggedRobotVisualizer});

    // Initial state
    SystemObservation initObservation;
    initObservation.state = interface.getInitialState();
    initObservation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
    initObservation.mode = ModeNumber::STANCE;

    // Initial command
    TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state}, {initObservation.input});

    // run dummy
    leggedRobotDummySimulator.run(initObservation, initTargetTrajectories);

    // Successful exit
    return 0;
}
