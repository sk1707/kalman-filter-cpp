#pragma once
#include "ekf.h"
#include <vector>
#include <string>

enum class ObjectType { VEHICLE, PEDESTRIAN };

struct TrackedObject {
    int id;
    ObjectType type;
    EKF filter;
    bool initialized;

    TrackedObject(int id, ObjectType type)
        : id(id), type(type), filter(4, 2), initialized(false) {}
};

class MultiObjectTracker {
public:
    MultiObjectTracker();
    void addObject(int id, ObjectType type, const Eigen::VectorXd& initial_state);
    void predict(double dt);
    void update(int id, const Eigen::VectorXd& measurement, const std::string& sensor_type);
    void printStates() const;

private:
    std::vector<TrackedObject> objects_;
    Eigen::MatrixXd vehicleQ();
    Eigen::MatrixXd pedestrianQ();
    Eigen::MatrixXd lidarR();
    Eigen::MatrixXd radarR();
};
