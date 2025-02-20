#include "kalmanfilter.h"


// Constants
constexpr double G  = 9.81;      // gravitational acceleration 
constexpr double DT = 0.01;      // time step 

// Process noise covariance (Q)
constexpr double Q_ALT = 0.01;   // process noise for altitude
constexpr double Q_VEL = 0.1;    // process noise for velocity

// Measurement noise covariance (R) for barometer
constexpr double R_BARO = 0.5;   // barometer altitude noise (m^2)

int main()
{
    // 1) Initialize the sensors
    if (!initICM20948()) {
        std::cerr << "ERROR: Failed to initialize ICM-20948!\n";
        return -1;
    }
    if (!initBMP581()) {
        std::cerr << "ERROR: Failed to initialize BMP581!\n";
        return -1;
    }

    // 2) Kalman Filter variables

    // State vector x = [altitude, velocity]
    Eigen::Vector2d x;
    x << 0.0, 0.0;  // initial guess of altitude, velocity

    // State covariance P
    Eigen::Matrix2d P;
    P << 10.0, 0.0,
         0.0, 10.0; // large initial uncertainty

    // F matrix
    Eigen::Matrix2d F;
    F << 1.0, DT,
         0.0, 1.0;

    // B vector for (a_z - g)
    Eigen::Vector2d B;

    // Q matrix (process noise)
    Eigen::Matrix2d Q;
    Q << Q_ALT,   0.0,
         0.0,     Q_VEL;

    // Measurement matrix H
    Eigen::RowVector2d H; // 1x2
    H << 1.0, 0.0;

    // R (measurement noise for altitude)
    double R = R_BARO;

    // Identity matrix
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

    // 3) Main loop
    while (true) {
        // ---------------------------------------------------------------------
        // A) Read sensors
        // ---------------------------------------------------------------------
        double accelZ = readICM20948AccelZ(); // m/s^2 (raw reading includes gravity)
        double baroAltitude = readBMP581Altitude(); // altitude in meters

        // net acceleration in vertical axis, subtract gravity if sensor has
        double netAccel = accelZ - G;

        // B) Prediction Step
        B << 0.0,
             DT;

        x = F * x + B * netAccel;
        P = F * P * F.transpose() + Q;

        // C) Measurement Update (Barometer altitude)
        double z = baroAltitude;       // barometer measurement
        double y = z - H * x;          // innovation or residual

        double S = (H * P * H.transpose())(0, 0) + R; // scalar
        Eigen::Vector2d K = P * H.transpose() * (1.0 / S);

        x = x + K * y;
        P = (I - K * H) * P;

        // D) Use or log results
        double estAltitude = x(0);
        double estVelocity = x(1);

        std::cout << "[KF] Altitude: " << estAltitude << " m, "
                  << "Velocity: " << estVelocity << " m/s\n";

        // Wait next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(DT * 1000)
        ));
    }

    return 0;
}