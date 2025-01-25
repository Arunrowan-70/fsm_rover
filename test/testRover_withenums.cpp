#include "Rover.h"
#include "roverfsmwithEnums.h"
#include <gtest/gtest.h>

using with_enums::FSM;

TEST(roverfsmwithEnums, TestInitialState) {
	FSM fsm;

	EXPECT_EQ(States::Idle, fsm.getState());

	EXPECT_EQ(LEDController::status::OFF, fsm.getled().getStatus());
	EXPECT_EQ("Hi I am Rufus", fsm.getMsg().getFirstRow());
	EXPECT_EQ("", fsm.getMsg().getSecondRow());
	EXPECT_EQ("", fsm.getMsg().getThirdRow());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::NO_SENSORS, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::NO_SENSORS, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::NO_SENSORS, fsm.getsmStatus().getSensorStatus("radar"));
}

TEST(roverfsmwithEnums, TestPrepareState) {
	FSM fsm;
	EXPECT_EQ(States::Idle, fsm.getState());
	fsm.process();

	EXPECT_EQ(States::Prepare, fsm.getState());

	EXPECT_TRUE(fsm.checkpre_conditions().pre_condtions);
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ("PREPARATION", fsm.getMsg().getFirstRow());
	EXPECT_EQ("SENSORS_BEING_ACTIVATED", fsm.getMsg().getSecondRow());
	EXPECT_EQ("READY_TO_PLAN_TRAJECTORY", fsm.getMsg().getThirdRow());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));
}

TEST(roverfsmwithEnums, Test_Pla_TrajState) {
	FSM fsm;
	fsm.process();
	EXPECT_EQ(States::Prepare, fsm.getState());

	fsm.process(pre_conditions_met{});

	EXPECT_EQ(States::PlanTrajectory, fsm.getState());
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));
	EXPECT_TRUE(fsm.check_trajplanner().traj_planned);
	EXPECT_EQ("SENSOR_CHECK", fsm.getMsg().getFirstRow());
	EXPECT_EQ("PLANNING_TRAJECTORY", fsm.getMsg().getSecondRow());
	EXPECT_EQ("READY_TO_MANEUVER", fsm.getMsg().getThirdRow());
}

TEST(roverfsmwithEnums, Test_RoverManeuvering) {
	FSM fsm;
	fsm.process().process(pre_conditions_met{}).process(path_estimation{});

	EXPECT_TRUE(fsm.check_trajplanner().traj_planned);
	EXPECT_EQ(States::Moving, fsm.getState());
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ("YO", fsm.getMsg().getFirstRow());
	EXPECT_EQ("I AM", fsm.getMsg().getSecondRow());
	EXPECT_EQ("MANEUVERING", fsm.getMsg().getThirdRow());
	EXPECT_EQ(Wheels::wheelstatus::MOVING, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));
}

TEST(roverfsmwithEnums, Test_PauseState) {
	FSM fsm;
	fsm.process().process(pre_conditions_met{}).process(path_estimation{}).process(obstacle{});

	EXPECT_EQ(States::Pause, fsm.getState());
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ("PAUSE", fsm.getMsg().getFirstRow());
	EXPECT_EQ("", fsm.getMsg().getSecondRow());
	EXPECT_EQ("", fsm.getMsg().getThirdRow());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));

	fsm.process(obstacle{});
	EXPECT_EQ(States::Prepare, fsm.getState());
	EXPECT_EQ("RE_PREPARATION", fsm.getMsg().getFirstRow());
	EXPECT_EQ("SENSORS_ALREADY_ACTIAVTED", fsm.getMsg().getSecondRow());
	EXPECT_EQ("REPLAN_TRAJECTORY", fsm.getMsg().getThirdRow());
}


TEST(roverfsmwithEnums, Test_Success_State) {
	FSM fsm;
	fsm.process().process(pre_conditions_met{}).process(path_estimation{});

	EXPECT_TRUE(fsm.check_trajplanner().traj_planned);
	EXPECT_EQ(States::Moving, fsm.getState());
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ("YO", fsm.getMsg().getFirstRow());
	EXPECT_EQ("I AM", fsm.getMsg().getSecondRow());
	EXPECT_EQ("MANEUVERING", fsm.getMsg().getThirdRow());
	EXPECT_EQ(Wheels::wheelstatus::MOVING, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));

	fsm.process(Destination_reached{});
	EXPECT_EQ(States::Success, fsm.getState());
	EXPECT_EQ("REACHED_THE_DESTINATION", fsm.getMsg().getFirstRow());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
}

TEST(roverfsmwithEnums, Test_Failed_State) {
	FSM fsm;
	fsm.process().process(pre_conditions_met{}).process(path_estimation{}).process(obstacle{});

	EXPECT_EQ(States::Pause, fsm.getState());
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ("PAUSE", fsm.getMsg().getFirstRow());
	EXPECT_EQ("", fsm.getMsg().getSecondRow());
	EXPECT_EQ("", fsm.getMsg().getThirdRow());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));


	fsm.process(Timeout{}).process(Timeout{}).process(Timeout{});
	EXPECT_EQ(States::Failed, fsm.getState());
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("radar"));
}

TEST(roverfsmwithEnums, Test_LowPowerMode_State) {
	FSM fsm;
	fsm.process();
	EXPECT_EQ(States::Prepare, fsm.getState());

	fsm.process(pre_conditions_met{});

	EXPECT_EQ(States::PlanTrajectory, fsm.getState());
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));
	EXPECT_TRUE(fsm.check_trajplanner().traj_planned);
	EXPECT_EQ("SENSOR_CHECK", fsm.getMsg().getFirstRow());
	EXPECT_EQ("PLANNING_TRAJECTORY", fsm.getMsg().getSecondRow());
	EXPECT_EQ("READY_TO_MANEUVER", fsm.getMsg().getThirdRow());

	fsm.process(lowpower{});
	EXPECT_EQ(States::Low_Power_Mode, fsm.getState());
	EXPECT_EQ(LEDController::status::LOW_POWER, fsm.getled().getStatus());
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("radar"));
	EXPECT_EQ("LOW_POWER", fsm.getMsg().getFirstRow());
}

TEST(roverfsmwithEnums, Test_Sleep_State) {
	FSM fsm;
	EXPECT_EQ(States::Idle, fsm.getState());
	fsm.process();

	EXPECT_EQ(States::Prepare, fsm.getState());

	EXPECT_TRUE(fsm.checkpre_conditions().pre_condtions);
	EXPECT_EQ(LEDController::status::ON, fsm.getled().getStatus());
	EXPECT_EQ("PREPARATION", fsm.getMsg().getFirstRow());
	EXPECT_EQ("SENSORS_BEING_ACTIVATED", fsm.getMsg().getSecondRow());
	EXPECT_EQ("READY_TO_PLAN_TRAJECTORY", fsm.getMsg().getThirdRow());
	EXPECT_EQ(Wheels::wheelstatus::IDLE, fsm.getwstatus().get_wheelStatus());
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::ACTIVE, fsm.getsmStatus().getSensorStatus("radar"));

	fsm.process(push_to_sleep{});
	EXPECT_EQ(States::Sleep, fsm.getState());
	EXPECT_EQ(LEDController::status::OFF, fsm.getled().getStatus());
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("camera"));
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("ultrasonic"));
	EXPECT_EQ(SensorManager::SensorStatus::IDLE, fsm.getsmStatus().getSensorStatus("radar"));
	EXPECT_EQ("SLEEP_MODE", fsm.getMsg().getFirstRow());
	EXPECT_EQ("ALL_SYSTEMS_OFF", fsm.getMsg().getSecondRow());
	EXPECT_EQ("ZZZZZZZ", fsm.getMsg().getThirdRow());
}
