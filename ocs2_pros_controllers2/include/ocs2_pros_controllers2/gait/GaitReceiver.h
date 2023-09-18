#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ocs2_pros_interface2/gait/ProsGaitSchedule.h>
#include <ocs2_pros_interface2/gait/ProsModeSequenceTemplate.h>
#include <ocs2_pros_interface2/gait/ProsMotionPhaseDefinition.h>

namespace ocs2
{
    namespace pros_walker
    {

        class GaitReceiver : public SolverSynchronizedModule
        {
        public:
            GaitReceiver(::ros::NodeHandle nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string &robotName);

            void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                              const ReferenceManagerInterface &referenceManager) override;

            void postSolverRun(const PrimalSolution &primalSolution) override{};

        private:
            void mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr &msg);

            std::shared_ptr<GaitSchedule> gaitSchedulePtr_;

            ::ros::Subscriber mpcModeSequenceSubscriber_;

            std::mutex receivedGaitMutex_;
            std::atomic_bool gaitUpdated_;
            ModeSequenceTemplate receivedGait_;
        };

    } // namespace legged_robot
} // namespace ocs2
