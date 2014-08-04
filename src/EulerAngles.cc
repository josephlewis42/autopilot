
#include "EulerAngles.h"


#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>



EulerAngles::EulerAngles(double rollRad, double pitchRad, double yawRad)
:_rollRad(rollRad),
 _pitchRad(pitchRad),
 _yawRad(yawRad)
{
}



boost::numeric::ublas::matrix<double> EulerAngles::toRotation() const
{
    boost::numeric::ublas::matrix<double> rot(3,3);
    rot.clear();

    double roll = _rollRad, pitch = _pitchRad, yaw = _yawRad;
    rot(0, 0) = cos(yaw) * cos(pitch);
    rot(0, 1) = sin(yaw) * cos(pitch);
    rot(0, 2) = -sin(pitch);
    rot(1, 0) = -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);
    rot(1, 1) = cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll);
    rot(1, 2) = cos(pitch) * sin(roll);
    rot(2, 0) = sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);
    rot(2, 1) = -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);
    rot(2, 2) = cos(pitch) * cos(roll);

    return trans(rot);
}


/// Adapted from http://ai.stanford.edu/~acoates/quaternion.h
EulerAngles EulerAngles::fromQuaternion(double w, double x, double y, double z)
{
    double roll;
    double pitch;
    double yaw;

    const static double PI_OVER_2 = M_PI * 0.5;
    const static double EPSILON = 1e-10;
    double sqw, sqx, sqy, sqz;

    // quick conversion to Euler angles to give tilt to user
    sqw = w * w;
    sqx = x * x;
    sqy = y * y;
    sqz = z * z;

    pitch = asin(2.0 * (w*y - x*z));
    if (PI_OVER_2 - fabs(pitch) > EPSILON) {
        yaw = atan2(2.0 * (x*y + w*z),
                         sqx - sqy - sqz + sqw);
        roll = atan2(2.0 * (w*x + y*z),
                         sqw - sqx - sqy + sqz);
    } else {
        // compute heading from local 'down' vector
        yaw = atan2(2*y*z - 2*x*w,
                         2*x*z + 2*y*w);
        roll = 0.0;

        // If facing down, reverse yaw
        if (pitch < 0)
        {
              yaw = M_PI - yaw;
        }
    }

    EulerAngles e(roll, pitch, yaw);
    return e;
}
