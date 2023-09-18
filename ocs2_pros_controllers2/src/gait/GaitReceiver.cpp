#include "ocs2_pros_controllers2/gait/GaitReceiver.h"

#include "ocs2_pros_controllers2/gait/ModeSequenceTemplateRos.h"

namespace ocs2
{
    namespace pros_walker
    {

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        GaitReceiver::GaitReceiver(::ros::NodeHandle nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string &robotName)
            : gaitSchedulePtr_(std::move(gaitSchedulePtr)), receivedGait_({0.0, 1.0}, {ModeNumber::STANCE}), gaitUpdated_(false)
        {
            mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &GaitReceiver::mpcModeSequenceCallback, this,
                                                              ::ros::TransportHints().udp());
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void GaitReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                                        const ReferenceManagerInterface &referenceManager)
        {
            if (gaitUpdated_)
            {
                std::lock_guard<std::mutex> lock(receivedGaitMutex_);
                std::cerr << "[GaitReceiver]: Setting new gait after time " << finalTime << "\n";
                std::cerr << receivedGait_;
                const auto timeHorizon = finalTime - initTime;
                gaitSchedulePtr_->insertModeSequenceTemplate(receivedGait_, finalTime, timeHorizon);
                gaitUpdated_ = false;
            }
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void GaitReceiver::mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr &msg)
        {
            std::lock_guard<std::mutex> lock(receivedGaitMutex_);
            receivedGait_ = readModeSequenceTemplateMsg(*msg);
            gaitUpdated_ = true;
        }

    } // namespace legged_robot
} // namespace ocs2
