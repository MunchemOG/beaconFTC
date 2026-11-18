#include "geometry.h"

namespace beacon {

SE3 mountToTransform(double x, double y, double z, double roll, double pitch, double yaw) {
    // R_forward turns optical axes into robot axes for a camera looking along robot +x with the
    // image upright: optical z (forward) -> robot +x, optical x (right) -> robot -y,
    // optical y (down) -> robot -z.
    Mat3 rForward;
    rForward << 0, 0, 1,
               -1, 0, 0,
                0, -1, 0;
    Mat3 r = (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
              Eigen::AngleAxisd(roll, Vec3::UnitX()))
                 .toRotationMatrix() *
             rForward;
    SE3 t = SE3::Identity();
    t.linear() = r;
    t.translation() = Vec3(x, y, z);
    return t;
}

}  // namespace beacon
