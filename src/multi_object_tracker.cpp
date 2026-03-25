#include "tracker.h"
#include <iostream>

MultiObjectTracker::MultiObjectTracker() {}

Eigen::MatrixXd MultiObjectTracker::vehicleQ() {
    // Vehicles: higher acceleration noise
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
    Q(0,0) = 0.5; Q(1,1) = 0.5;
    Q(2,2) = 2.0; Q(3,3) = 2.0;
    return Q;
}

Eigen::MatrixXd MultiObjectTracker::pedestrianQ() {
    // Pedestrians: lower, more unpredictable motion
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
    Q(0,0) = 0.2; Q(1,1) = 0.2;
    Q(2,2) = 0.5; Q(3,3) = 0.5;
    return Q;
}

Eigen::MatrixXd MultiObjectTracker::lidarR() {
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
    R(0,0) = 0.05; R(1,1) = 0.05;
    return R;
}

Eigen::MatrixXd MultiObjectTracker::radarR() {
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    R(0,0) = 0.3; R(1,1) = 0.03; R(2,2) = 0.3;
    return R;
}

void MultiObjectTracker::addObject(int id, ObjectType type,
                                   const Eigen::VectorXd& initial_state) {
    TrackedObject obj(id, type);
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4) * 10.0;
    Eigen::MatrixXd Q = (type == ObjectType::VEHICLE) ? vehicleQ() : pedestrianQ();
    Eigen::MatrixXd R = lidarR();
    obj.filter.init(initial_state, P, Q, R);
    obj.initialized = true;
    objects_.push_back(obj);
}

void MultiObjectTracker::predict(double dt) {
    for (auto& obj : objects_)
        if (obj.initialized) obj.filter.predict(dt);
}

void MultiObjectTracker::update(int id, const Eigen::VectorXd& measurement,
                                const std::string& sensor_type) {
    for (auto& obj : objects_) {
        if (obj.id == id && obj.initialized) {
            obj.filter.update(measurement, sensor_type);
            break;
        }
    }
}

void MultiObjectTracker::printStates() const {
    for (const auto& obj : objects_) {
        if (obj.initialized) {
            std::string type_str = (obj.type == ObjectType::VEHICLE) ? "VEHICLE" : "PEDESTRIAN";
            std::cout << "Object ID: " << obj.id << " (" << type_str << ")\n";
            std::cout << "State: " << obj.filter.getState().transpose() << "\n";
        }
    }
}
