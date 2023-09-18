#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <ocs2_pros_interface2/gait/ProsModeSequenceTemplate.h>

namespace ocs2
{
    namespace pros_walker
    {

        /** This class implements ModeSequence communication using ROS. */
        class GaitKeyboardPublisher
        {
        public:
            GaitKeyboardPublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile, const std::string &robotName, bool verbose = false);

            /** Prints the command line interface and responds to user input. Function returns after one user input. */
            void getKeyboardCommand();

        private:
            /** Prints the list of available gaits. */
            void printGaitList(const std::vector<std::string> &gaitList) const;

            std::vector<std::string> gaitList_;
            std::map<std::string, ModeSequenceTemplate> gaitMap_;

            ros::Publisher modeSequenceTemplatePublisher_;
        };

    } // namespace legged_robot
} // end of namespace ocs2
