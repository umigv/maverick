#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <vectornav/Implementation/MeasurementDatatypes.hpp>
#include <vectornav/TemplateLibrary/Matrix.hpp>

inline geometry_msgs::msg::Vector3 to_vector3(const VN::Vec3f& vector) {
    geometry_msgs::msg::Vector3 out;
    out.x = vector[0];
    out.y = vector[1];
    out.z = vector[2];
    return out;
}

inline tf2::Quaternion to_tf2_quaternion(const VN::Quat& quaternion) {
    return tf2::Quaternion(quaternion.vector[0], quaternion.vector[1], quaternion.vector[2], quaternion.scalar);
}

inline std::array<double, 9> diag_covariance(double vector) {
    // clang-format off
    return {vector, 0, 0, 
            0, vector, 0, 
            0, 0, vector};
    // clang-format on
}

inline std::array<double, 9> diag_covariance(double x, double y, double z) {
    // clang-format off
    return {x, 0, 0, 
            0, y, 0, 
            0, 0, z};
    // clang-format on
}

inline std::array<double, 9> to_covariance(const std::vector<double>& vector) {
    std::array<double, 9> out;
    std::copy_n(vector.begin(), 9, out.begin());
    return out;
}

inline std::array<double, 36> block_diag_covariance(const std::array<double, 9>& top,
                                                    const std::array<double, 9>& bottom) {
    // clang-format off
    return {
        top[0], top[1], top[2], 0, 0, 0, 
        top[3], top[4], top[5], 0, 0, 0, 
        top[6], top[7], top[8], 0, 0, 0, 
        0, 0, 0, bottom[0], bottom[1], bottom[2],
        0, 0, 0, bottom[3], bottom[4], bottom[5],
        0, 0, 0, bottom[6], bottom[7], bottom[8]
    };
    // clang-format on
}

inline geometry_msgs::msg::Vector3 frd_to_flu(const geometry_msgs::msg::Vector3& vector) {
    geometry_msgs::msg::Vector3 out;
    out.x = vector.x;
    out.y = -vector.y;
    out.z = -vector.z;
    return out;
}

inline std::array<double, 9> frd_to_flu(const std::array<double, 9>& covariance) {
    // clang-format off
    return {covariance[0], -covariance[1], -covariance[2], 
            -covariance[3], covariance[4], covariance[5], 
            -covariance[6], covariance[7], covariance[8]};
    // clang-format on
}

inline tf2::Quaternion ned_to_enu(const tf2::Quaternion& quaternion) {
    static const tf2::Quaternion q_ned_to_enu(std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0);
    static const tf2::Quaternion q_frd_to_flu(1.0, 0.0, 0.0, 0.0);
    return (q_ned_to_enu * quaternion * q_frd_to_flu).normalize();
}

class Rotation {
  public:
    Rotation() = default;

    explicit Rotation(const tf2::Quaternion& quaternion)
        : quaternion_(quaternion.normalized()), rotation_(quaternion_) {}

    geometry_msgs::msg::Vector3 rotate(const geometry_msgs::msg::Vector3& vector) const {
        const tf2::Vector3 rotated = rotation_ * tf2::Vector3(vector.x, vector.y, vector.z);
        geometry_msgs::msg::Vector3 out;
        out.x = rotated.x();
        out.y = rotated.y();
        out.z = rotated.z();
        return out;
    }

    tf2::Quaternion rotate(const tf2::Quaternion& quaternion) const {
        return (quaternion * quaternion_.inverse()).normalize();
    }

    std::array<double, 9> rotate(const std::array<double, 9>& covariance) const {
        // clang-format off
        const tf2::Matrix3x3 mat(covariance[0], covariance[1], covariance[2], 
                                 covariance[3], covariance[4], covariance[5], 
                                 covariance[6], covariance[7], covariance[8]);
        // clang-format on
        const tf2::Matrix3x3 result = rotation_ * mat * rotation_.transpose();
        std::array<double, 9> out;
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                out[i * 3 + j] = result[i][j];
            }
        }
        return out;
    }

  private:
    tf2::Quaternion quaternion_{0.0, 0.0, 0.0, 1.0};
    tf2::Matrix3x3 rotation_{tf2::Matrix3x3::getIdentity()};
};
