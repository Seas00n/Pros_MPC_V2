#include "ocs2_pros_controllers2/gait/GaitKeyboardPublisher.h"

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>

#include "ocs2_pros_controllers2/gait/ModeSequenceTemplateRos.h"

namespace ocs2
{
    namespace pros_walker
    {

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        GaitKeyboardPublisher::GaitKeyboardPublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile, const std::string &robotName,
                                                     bool verbose)
        {
            ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
            loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

            modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

            gaitMap_.clear();
            for (const auto &gaitName : gaitList_)
            {
                gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
            }
            ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void GaitKeyboardPublisher::getKeyboardCommand()
        {
            const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
            std::cout << commadMsg << ": ";

            auto shouldTerminate = []()
            { return !ros::ok() || !ros::master::check(); };
            const auto commandLine = stringToWords(getCommandLineString(shouldTerminate));

            if (commandLine.empty())
            {
                return;
            }

            if (commandLine.size() > 1)
            {
                std::cout << "WARNING: The command should be a single word." << std::endl;
                return;
            }

            // lower case transform
            auto gaitCommand = commandLine.front();
            std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(), ::tolower);

            if (gaitCommand == "list")
            {
                printGaitList(gaitList_);
                return;
            }

            try
            {
                ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
                modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
            }
            catch (const std::out_of_range &e)
            {
                std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
                printGaitList(gaitList_);
            }
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void GaitKeyboardPublisher::printGaitList(const std::vector<std::string> &gaitList) const
        {
            std::cout << "List of available gaits:\n";
            size_t itr = 0;
            for (const auto &s : gaitList)
            {
                std::cout << "[" << itr++ << "]: " << s << "\n";
            }
            std::cout << std::endl;
        }

    } // namespace legged_robot
} // end of namespace ocs2
