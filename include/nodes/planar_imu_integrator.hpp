#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>

namespace algorithms {

    class PlanarImuIntegrator {
    public:

        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        void update(float gyro_z, double dt);

        void setCalibration(std::vector<float> gyro);

        [[nodiscard]] float getYaw() const;

        void reset();

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}
