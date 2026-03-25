#include <iostream>
#include "tracker.h"

int main() {
    MultiObjectTracker tracker;

    // Vehicle starting at (10, 5) moving at (2, 0.5) m/s
    Eigen::VectorXd vehicle_state(4);
    vehicle_state << 10, 5, 2, 0.5;
    tracker.addObject(1, ObjectType::VEHICLE, vehicle_state);

    // Pedestrian starting at (2, 8) moving at (0.3, -0.2) m/s
    Eigen::VectorXd ped_state(4);
    ped_state << 2, 8, 0.3, -0.2;
    tracker.addObject(2, ObjectType::PEDESTRIAN, ped_state);

    double dt = 0.1;
    // Simulated lidar measurements for 5 timesteps
    double vehicle_meas[][2] = {{10.2,5.1},{10.4,5.2},{10.7,5.3},{10.9,5.3},{11.1,5.4}};
    double ped_meas[][2]     = {{2.1,7.9},{2.2,7.8},{2.3,7.7},{2.4,7.6},{2.5,7.5}};

    std::cout << "=== Urban Intersection Multi-Object Tracker ===\n\n";
    for (int i = 0; i < 5; i++) {
        tracker.predict(dt);

        Eigen::VectorXd z_vehicle(2);
        z_vehicle << vehicle_meas[i][0], vehicle_meas[i][1];
        tracker.update(1, z_vehicle, "lidar");

        Eigen::VectorXd z_ped(2);
        z_ped << ped_meas[i][0], ped_meas[i][1];
        tracker.update(2, z_ped, "lidar");

        std::cout << "--- Timestep " << i+1 << " ---\n";
        tracker.printStates();
        std::cout << "\n";
    }
    return 0;
}
