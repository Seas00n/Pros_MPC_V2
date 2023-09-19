#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2
{
    namespace pros_walker
    {

        struct ModelSettings
        {
            scalar_t positionErrorGain = 0.0;

            scalar_t phaseTransitionStanceTime = 0.4;

            bool verboseCppAd = true;
            bool recompileLibrariesCppAd = false;
            std::string modelFolderCppAd = "/tmp/pros_walker";

            // This is only used to get names for the knees and to check urdf for extra joints that need to be fixed.
            std::vector<std::string> jointNames{"l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
                                                "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax"};
            std::vector<std::string> contactNames6DoF{};
            std::vector<std::string> swingRefName{"fp0_l", "fp0_r"};
            scalar_t frictionConeNum = 8;
            std::vector<std::string> contactNames3DoF{"fp1_l", "fp2_l", "fp3_l", "fp4_l", "fp1_r", "fp2_r", "fp3_r", "fp4_r"};
        };

        ModelSettings loadModelSettings(const std::string &filename, const std::string &fieldName = "model_settings", bool verbose = "true");

    } // namespace legged_robot
} // namespace ocs2