    #include "Rover.h"
    #include <array>
    namespace with_enums{
        class FSM{
            public: 
                FSM & process();
                FSM & process(pre_conditions_met event);
                FSM & process(path_estimation event);
                FSM & process(obstacle event);
                FSM & process(Timeout event);
                FSM & process(Destination_reached event);
                FSM & process(lowpower event);
                FSM & process(push_to_sleep event);
                void reset();

            States getState() const{
                return _state;
            }

            const LEDController & getled() const{
                return _led;
            }
            const Display_Terminal & getMsg() const{
                return _dt;
            }

            const Wheels& getwstatus() const {
                return _wstatus;
            }

            const SensorManager& getsmStatus() const {
                return _sm;
            }

            const pre_conditions_met& checkpre_conditions() const{
                return _pre_conditions;
            }

            const path_estimation& check_trajplanner() const {
                return _traj_planner;
            }
            
            private:
            void start_rover();

            void transitionToPrepare();
            void transitionToPlanTrajectory();
            void transitionToManeuvering();
            void transitionToPause();
            void transitionToFailed();
            void transitionToSuccess();
            void transitionToLowPowerMode();
            void transitionToSleep();
            

                States _state { States::Idle};

                LEDController _led;
                Display_Terminal _dt{"Hi I am Rufus"};
                Wheels _wstatus;
                Diagnostics _diag;
                SensorManager _sm;

                int _retryCounts{0};
                pre_conditions_met _pre_conditions;
                path_estimation _traj_planner;

                bool rover_stopped{false};
            
        };
        inline void FSM::reset() {
            _state = States::Idle;  
            _retryCounts = 0;       
            rover_stopped = false;
            _pre_conditions.pre_condtions = false;  
            _traj_planner.traj_planned = false;     
            _led.turnoff();         
            _dt.setRows("Reset to Idle");
            _sm.deactivateSensor("camera");  
            _sm.deactivateSensor("ultrasonic");
            _sm.deactivateSensor("radar");
        }

        inline void FSM::start_rover(){
            _led.turnon();
            _dt.setRows("Hi I am Rufus");
            transitionToPrepare();
        }


        inline FSM & FSM::process(){
            switch (_state)
            {
            case States::Idle:
                start_rover();
                break;
            
            default:
                break;
            }
            return *this;
        }

        inline FSM & FSM::process(pre_conditions_met event){
            switch (_state)
            {
            case States::Prepare:          
                if(_pre_conditions.pre_condtions)
                    transitionToPlanTrajectory();
                break;
            
            default:
                break;
            }
            return *this;
        }

        inline FSM & FSM::process(path_estimation event){
            switch (_state)
            {
            case States::PlanTrajectory:          
                if(_traj_planner.traj_planned)
                    transitionToManeuvering();
                break;
            
            default:
                break;
            }
            return *this;
        }

        inline FSM & FSM::process(obstacle event){
            switch (_state)
            {
            case States::Moving:
                transitionToPause();
                break;

            case States::Pause:
                if(rover_stopped)
                    transitionToPrepare();
                    break;
            
            default:
                break;
            }
            return *this;
        }

        inline FSM & FSM::process(Timeout event){
            switch (_state)
            {
            case States::Pause:
                _retryCounts++;
                if(_retryCounts > 2)
                {
                    transitionToFailed();
                }
                
                break;
            
            default:
                break;
            }
            return *this;
        }

        inline FSM & FSM::process(Destination_reached event){
            switch (_state)
            {
            case States::Moving:
                transitionToSuccess();
                break;
            
            default:
                break;
            }
            return *this;
        }

        inline FSM &FSM::process(push_to_sleep event) {
            transitionToSleep();
            return *this;
        }

        inline FSM &FSM::process(lowpower event) {
            transitionToLowPowerMode();
            return *this;
        }

        inline void FSM::transitionToPrepare(){
            _state=States::Prepare;
            if(rover_stopped){
                _dt.setRows("RE_PREPARATION","SENSORS_ALREADY_ACTIAVTED", "REPLAN_TRAJECTORY");
                rover_stopped=false;
                return;
            }
            _sm.addSensor("camera");
            _sm.addSensor("ultrasonic");
            _sm.addSensor("radar");
            _sm.activateSensor("camera");
            _sm.activateSensor("ultrasonic");
            _sm.activateSensor("radar");
            _dt.setRows("PREPARATION","SENSORS_BEING_ACTIVATED", "READY_TO_PLAN_TRAJECTORY");
            _pre_conditions.pre_condtions=true;
        }

        inline void FSM::transitionToPlanTrajectory(){
            _state=States::PlanTrajectory;
            if(_sm.getSensorStatus("camera") == SensorManager::SensorStatus::ACTIVE
            && _sm.getSensorStatus("ultrasonic") == SensorManager::SensorStatus::ACTIVE
            && _sm.getSensorStatus("radar") == SensorManager::SensorStatus::ACTIVE){
                _traj_planner.traj_planned=true;
            }
            _dt.setRows("SENSOR_CHECK","PLANNING_TRAJECTORY", "READY_TO_MANEUVER");
        }

        inline void FSM::transitionToManeuvering(){
            _state=States::Moving;
            _wstatus.setmoving();
            _dt.setRows("YO","I AM", "MANEUVERING");
        }

        inline void FSM::transitionToPause(){
            _state=States::Pause;
            _wstatus.setNotmoving();
            _dt.setRows("PAUSE");
            rover_stopped=true;
        }

        inline void FSM::transitionToFailed(){
            _state=States::Failed;
            _dt.setRows("FAILED");
            _sm.deactivateSensor("camera");
            _sm.deactivateSensor("ultrasonic");
            _sm.deactivateSensor("radar");
        }

        inline void FSM::transitionToSuccess(){
            _state=States::Success;
            _wstatus.setNotmoving();
            _dt.setRows("REACHED_THE_DESTINATION");
        }

        inline void FSM::transitionToLowPowerMode() {
            _state = States::Low_Power_Mode;
            _led.setLowPower();
            _sm.deactivateSensor("camera");
            _sm.deactivateSensor("ultrasonic");
            _sm.deactivateSensor("radar");
            _dt.setRows("LOW_POWER");
        }

        inline void FSM::transitionToSleep() {
            _state = States::Sleep;
            _led.turnoff();
            _sm.deactivateSensor("camera");
            _sm.deactivateSensor("ultrasonic");
            _sm.deactivateSensor("radar");
            _dt.setRows("SLEEP_MODE", "ALL_SYSTEMS_OFF","ZZZZZZZ");
        }


    };

    