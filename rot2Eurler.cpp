#include <mecl/core/RotationMatrix.h>
#include <iostream>

mecl::core::Matrix<float32_t,3,1> getRPY(mecl::core::Matrix<float32_t,3,3> data) const {
    float r, p, y;
    if (data[2][0] < 1.0) {
        if (data[2][0] > -1.0) {
            p = asin(-data[2][0]);
            r = atan2(data[2][1]/cos(p), data[2][2]/cos(p));
            y = atan2(data[1][0]/cos(p), data[0][0]/cos(p));
        } else {
            p = M_PI/2.0;
            r = -atan2(-data[0][1], -data[0][2]);
            y = 0.0;
        }
    } else {
        p = -M_PI/2.0;
        r = atan2(-data[0][1], -data[0][2]);
        y = 0.0;
    }
	mecl::core::Matrix<float32_t, 3, 1> RPY;
	RPY(0) = r;
	RPY(1) = p;
	RPY(2) = y;
    return Vec3f(r, p, y);
}


int main() {
    mecl::core::Matrix<float32_t,3,3> rotMat;
    // Set the values of the rotation matrix
    rotMat << 0.0, -1.0, 0.0,
              1.0, 0.0, 0.0,
              0.0, 0.0, 1.0;
    // Get the Euler angles in the order of roll, pitch, yaw
    mecl::core::Matrix<float32_t,3,1> rpy = getRPY(rotMat);
    std::cout << "Roll: " << rpy(0) << ", Pitch: " << rpy(1) << ", Yaw: " << rpy(2) << std::endl;
    return 0;
}
