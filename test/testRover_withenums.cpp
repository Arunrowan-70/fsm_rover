#include "Rover.h"
#include "roverfsmwithEnums.h"
#include <gtest/gtest.h>

namespace with_enums {
    
    class FSMTest : public ::testing::Test {
    protected:
        FSM fsm;
        
        void SetUp() override {
            // Initial setup for each test case if needed
            fsm.process();  // Process FSM to start the rover
        }
    };

    TEST_F(FSMTest, InitialStateIsIdle) {
        // Check the initial state is Idle
        ASSERT_EQ(fsm.getState(), States::Idle);
    }

    TEST_F(FSMTest, TransitionToPrepare) {
        // Trigger transition to Prepare state
        fsm.process(pre_conditions_met{true});

        // Ensure the FSM transitioned to the Prepare state
        ASSERT_EQ(fsm.getState(), States::Prepare);
    }

    TEST_F(FSMTest, TransitionToPlanTrajectory) {
        // Trigger the PlanTrajectory state transition from Prepare
        fsm.process(pre_conditions_met{true});
        fsm.process(path_estimation{true});
        
        // Ensure the FSM transitioned to PlanTrajectory
        ASSERT_EQ(fsm.getState(), States::PlanTrajectory);
    }

    TEST_F(FSMTest, TransitionToManeuvering) {
        // Transition to PlanTrajectory first
        fsm.process(pre_conditions_met{true});
        fsm.process(path_estimation{true});

        // Then transition to Maneuvering
        fsm.process(path_estimation{true});

        ASSERT_EQ(fsm.getState(), States::Moving);
    }

    TEST_F(FSMTest, TransitionToSleep) {
        // Trigger the sleep transition from any state
        fsm.process(push_to_sleep{});

        // Ensure the FSM transitioned to Sleep
        ASSERT_EQ(fsm.getState(), States::Sleep);
    }

    TEST_F(FSMTest, TransitionToFailedAfterTimeout) {
        // Transition to Prepare state first
        fsm.process(pre_conditions_met{true});

        // Then simulate Timeout and check for Failed state
        fsm.process(Timeout{});
        fsm.process(Timeout{});
        fsm.process(Timeout{});
        
        ASSERT_EQ(fsm.getState(), States::Failed);
    }

    TEST_F(FSMTest, TransitionToLowPowerMode) {
        // Simulate entering low power mode
        fsm.process(lowpower{});

        // Ensure the state transitions to Low Power Mode
        ASSERT_EQ(fsm.getState(), States::Low_Power_Mode);
    }

    TEST_F(FSMTest, TransitionToSuccessOnDestinationReached) {
        // Simulate moving and reaching destination
        fsm.process(path_estimation{true});
        fsm.process(Destination_reached{});

        ASSERT_EQ(fsm.getState(), States::Success);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
