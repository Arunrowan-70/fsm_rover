#pragma once

#include <iostream>
#include <functional>
#include <sstream>
#include <string>
#include <array>
#include<vector>
#include<unordered_map>

class LEDController{
    public:
        enum class status{
            OFF,
            ON,
            ERROR,
            WARNING,
            LOW_POWER,
            SUCCESS
        };
        void turnon(){
            _status=status::ON;
        }

        void turnoff(){
            _status=status::OFF;
        }

        void setError(){
            _status=status::ERROR;
        }

        void setWarning(){
            _status=status::WARNING;
        }      

        void setLowPower(){
            _status=status::LOW_POWER;
        }  

        void setSuccess(){
            _status=status::SUCCESS;
        }    


        status getStatus() const{
            return _status;
        }
    
    private:
        status _status{status::OFF};
};


class Display_Terminal{
    public:
        Display_Terminal(std::string fr, std::string sr="", std::string tr="")
        :_firstRow(std::move(fr)), _secondRow(std::move(sr)), _thirdRow(std::move(tr))
        {}

        const std::string& getFirstRow() const{
            return _firstRow;
        }
        const std::string& getSecondRow() const{
            return _secondRow;
        }
        const std::string& getThirdRow() const{
            return _thirdRow;
        }

        void setRows(std::string fr, std::string sr="", std::string tr=""){
            _firstRow=std::move(fr);
            _secondRow=std::move(sr);
            _thirdRow=std::move(tr);
        }

        std::string getRows()const{
            std::stringstream stm;
            stm<<_firstRow<<","<<_secondRow<<","<<_thirdRow;
            return stm.str();
        }

    private:
        std::string _firstRow;
        std::string _secondRow;
        std::string _thirdRow;

};

class Wheels{
    public:
        enum class wheelstatus{
           IDLE,
           MOVING,
           ERROR 
        };

    void setmoving(){
        _wstatus = wheelstatus::MOVING;
    }
    void setNotmoving(){
        _wstatus = wheelstatus::IDLE;
    }
    wheelstatus get_wheelStatus() const{
        return _wstatus;
    }
    Wheels(): _wstatus(wheelstatus::IDLE), _speed(0)

    {}
    private:
        wheelstatus _wstatus{wheelstatus::IDLE};
        int _speed;
};


class Diagnostics{
    public:
        void logmsg(const std::string& message){
            logs.push_back(message);
        }

        std::vector<std::string> getlog() const {
            return logs;
        }

    private:
        std::vector<std::string> logs;
};

class SensorManager{
    public:
    enum class SensorStatus {
        ACTIVE,
        IDLE,
        ERROR,
        NO_SENSORS
    };

    void addSensor(const std::string& sensorName);

    void activateSensor(const std::string& sensorName);

    void deactivateSensor(const std::string& sensorName);

    SensorStatus getSensorStatus(const std::string& sensorName) const;

    private:
        std::unordered_map<std::string, SensorStatus> sensors;
    
};

//EVENTS
struct turnon_rover{
    bool turned_on;
};
struct pre_conditions_met{
    bool pre_condtions;
};

struct path_estimation{
    bool traj_planned;
};

struct obstacle
{
    bool obstacle_detected;
};
struct lowpower{};
struct push_to_sleep{};
struct Timeout{};
struct Destination_reached{};
//STATES

enum class States{
    Idle,
    Prepare,
    PlanTrajectory,
    Moving,
    Pause,
    Failed,
    Success,
    Low_Power_Mode,
    Sleep
};
