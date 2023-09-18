#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_pros_interface2/ProsWalkerInterface.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>

#include "ocs2_pros_controllers2/gait/GaitReceiver.h"

using namespace ocs2;
using namespace pros_walker;

int main(int argc, char **argv)
{
    const std::string robotName = "legged_robot";

    // Initialize ros node
    ::ros::init(argc, argv, robotName + "_mpc");
    ::ros::NodeHandle nodeHandle;
    // Get node parameters
    bool multiplot = false;
    std::string taskFile, urdfFile, referenceFile;
    nodeHandle.getParam("/multiplot", multiplot);
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/referenceFile", referenceFile);
    nodeHandle.getParam("/urdfFile", urdfFile);

    // Robot interface
    LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

    // Gait receiver
    auto gaitReceiverPtr =
        std::make_shared<GaitReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nodeHandle);
    
    // MPC
    GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(), interface.getOptimalControlProblem(),
                           interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

    // observer for zero velocity constraints (only add this for debugging as it slows down the solver)
    if (multiplot)
    {
        auto createStateInputBoundsObserver = [&](const std::string &termName)
        {
            const ocs2::scalar_array_t observingTimePoints{0.0};
            const std::vector<std::string> topicNames{"metrics/" + termName + "/0MsLookAhead"};
            auto callback = ocs2::ros::createConstraintCallback(nodeHandle, {0.0}, topicNames,
                                                                ocs2::ros::CallbackInterpolationStrategy::linear_interpolation);
            return ocs2::SolverObserver::ConstraintTermObserver(ocs2::SolverObserver::Type::Intermediate, termName, std::move(callback));
        };
        for (size_t i = 0; i < interface.getCentroidalModelInfo().numThreeDofContacts; i++)
        {
            const std::string &footName = interface.modelSettings().contactNames3DoF[i];
            mpc.getSolverPtr()->addSolverObserver(createStateInputBoundsObserver(footName + "_zeroVelocity"));
        }
    }

    // Launch MPC ROS node
    MPC_ROS_Interface mpcNode(mpc, robotName);
    mpcNode.launchNodes(nodeHandle);

    // Successful exit
    return 0;
}
