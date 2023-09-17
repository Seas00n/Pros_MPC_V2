
#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_pros_interface2/reference_manager/ProsSwitchedModelReferenceManager.h"

namespace ocs2
{
    namespace pros_walker
    {

        class LeggedRobotInitializer final : public Initializer
        {
        public:
            /*
             * Constructor
             * @param [in] info : The centroidal model information.
             * @param [in] referenceManager : Switched system reference manager.
             * @param [in] extendNormalizedMomentum: If true, it extrapolates the normalized momenta; otherwise sets them to zero.
             */
            LeggedRobotInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager &referenceManager,
                                   bool extendNormalizedMomentum = false);

            ~LeggedRobotInitializer() override = default;
            LeggedRobotInitializer *clone() const override;

            void compute(scalar_t time, const vector_t &state, scalar_t nextTime, vector_t &input, vector_t &nextState) override;

        private:
            LeggedRobotInitializer(const LeggedRobotInitializer &other) = default;

            const CentroidalModelInfo info_;
            const SwitchedModelReferenceManager *referenceManagerPtr_;
            const bool extendNormalizedMomentum_;
        };

    } // namespace legged_robot
} // namespace ocs2
