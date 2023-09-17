#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "ocs2_pros_interface2/common/ProsWalkerModelSettings.h"
#include "ocs2_pros_interface2/constraint/ProsEndEffectorLinearConstraint.h"
#include "ocs2_pros_interface2/foot_planner/ProsSwingTrajectoryPlanner.h"

namespace ocs2
{
    namespace pros_walker
    {

        /** Callback for caching and reference update */
        class LeggedRobotPreComputation : public PreComputation
        {
        public:
            LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                      const SwingTrajectoryPlanner &swingTrajectoryPlanner, ModelSettings settings);
            ~LeggedRobotPreComputation() override = default;

            LeggedRobotPreComputation *clone() const override;

            void request(RequestSet request, scalar_t t, const vector_t &x, const vector_t &u) override;

            const std::vector<EndEffectorLinearConstraint::Config> &getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

            PinocchioInterface &getPinocchioInterface() { return pinocchioInterface_; }
            const PinocchioInterface &getPinocchioInterface() const { return pinocchioInterface_; }

        private:
            LeggedRobotPreComputation(const LeggedRobotPreComputation &other) = default;

            PinocchioInterface pinocchioInterface_;
            CentroidalModelInfo info_;
            const SwingTrajectoryPlanner *swingTrajectoryPlannerPtr_;
            const ModelSettings settings_;

            std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
        };

    } // namespace legged_robot
} // namespace ocs2