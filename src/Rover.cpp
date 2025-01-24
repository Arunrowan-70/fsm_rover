#include "Rover.h"

void SensorManager::addSensor(const std::string& sensorName) {
    sensors[sensorName] = SensorStatus::IDLE;
}

void SensorManager::activateSensor(const std::string& sensorName) {
    if (sensors.find(sensorName) != sensors.end()) {
        sensors[sensorName] = SensorStatus::ACTIVE;
        std::cout << "Sensor " << sensorName << " activated.\n";
    } else {
        std::cout << "Sensor " << sensorName << " not found.\n";
    }
}

void SensorManager::deactivateSensor(const std::string& sensorName) {
    if (sensors.find(sensorName) != sensors.end()) {
        sensors[sensorName] = SensorStatus::IDLE;
        std::cout << "Sensor " << sensorName << " deactivated.\n";
    }
}

SensorManager::SensorStatus SensorManager::getSensorStatus(const std::string& sensorName) const {
    if (sensors.find(sensorName) != sensors.end()) {
        return sensors.at(sensorName);
    }
    return SensorStatus::ERROR;
}

