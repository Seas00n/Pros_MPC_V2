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
            std::vector<std::string> jointNames{"r_leg_lax", "r_leg_uay", "r_leg_kny", "r_leg_lhy", "r_leg_mhx",
                                                "l_leg_lax", "l_leg_uay", "l_leg_kny", "l_leg_lhy", "l_leg_mhx"};
            std::vector<std::string> contactNames6DoF{};
            // std::vector<std::string> contactNames3DoF{"fp1_r", "fp2_r", "fp3_r", "fp4_r", "fp1_l", "fp2_l", "fp3_l", "fp4_l"};
            std::vector<std::string> contactNames3DoF{"fp1_r", "fp1_l"};
        };

        ModelSettings loadModelSettings(const std::string &filename, const std::string &fieldName = "model_settings", bool verbose = "true");

    } // namespace legged_robot
} // namespace ocs2