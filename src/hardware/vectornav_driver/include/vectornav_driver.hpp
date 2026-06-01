#pragma once

#include <GeographicLib/LocalCartesian.hpp>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "transforms.hpp"
#include "vectornav/Interface/Sensor.hpp"

#if __linux__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

static bool optimize_serial_communication(const std::string& port) {
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        return false;
    }
    struct serial_struct serial;
    ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &serial);
    ::close(fd);
    return true;
}
#else
static bool optimize_serial_communication(const std::string&) { return false; }
#endif

static std::optional<VN::Sensor::BaudRate> to_baud_rate(int baud) {
    switch (baud) {
        case 9600:
            return VN::Sensor::BaudRate::Baud9600;
        case 19200:
            return VN::Sensor::BaudRate::Baud19200;
        case 38400:
            return VN::Sensor::BaudRate::Baud38400;
        case 57600:
            return VN::Sensor::BaudRate::Baud57600;
        case 115200:
            return VN::Sensor::BaudRate::Baud115200;
        case 128000:
            return VN::Sensor::BaudRate::Baud128000;
        case 230400:
            return VN::Sensor::BaudRate::Baud230400;
        case 460800:
            return VN::Sensor::BaudRate::Baud460800;
        case 921600:
            return VN::Sensor::BaudRate::Baud921600;
        default:
            return std::nullopt;
    }
}

class VectornavDriver : public rclcpp::Node {
    enum class SensorType { VN100, VN200, VN300 };

    static constexpr uint16_t MODE_NOT_TRACKING = 0;
    static constexpr uint16_t MODE_ALIGNING = 1;
    static constexpr uint16_t MODE_TRACKING = 2;
    static constexpr uint16_t MODE_GNSS_LOST = 3;

  public:
    VectornavDriver() : Node("vectornav_driver") {
        declare_parameter<std::string>("port", "/dev/vn300");
        declare_parameter<int>("baud_rate", static_cast<int>(VN::Sensor::BaudRate::Baud115200));
        declare_parameter<int>("publish_rate", 100);
        declare_parameter<std::vector<double>>("linear_accel_covariance", std::vector<double>(9, 0.0));
        declare_parameter<std::vector<double>>("angular_vel_covariance", std::vector<double>(9, 0.0));
        declare_parameter<std::string>("imu_frame_id", "vectornav");
        declare_parameter<std::string>("ins_frame_id", "vectornav");
        declare_parameter<std::string>("gnss_a_frame_id", "");
        declare_parameter<std::string>("gnss_b_frame_id", "");
        declare_parameter<std::vector<double>>("datum", std::vector<double>{});
        declare_parameter<std::string>("map_frame_id", "map");

        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
        fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("vectornav/fix", 10);
        velocity_publisher_ =
            create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("vectornav/velocity", 10);
        ins_status_publisher_ = create_publisher<std_msgs::msg::UInt16>("vectornav/ins_status", 10);
        gnss_status_publisher_ = create_publisher<std_msgs::msg::UInt16>("vectornav/gnss_status", 10);
        gnss2_status_publisher_ = create_publisher<std_msgs::msg::UInt16>("vectornav/gnss2_status", 10);
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("vectornav/odom", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    }

    ~VectornavDriver() {
        vs_.disconnect();
        RCLCPP_INFO(get_logger(), "Device disconnected.");
    }

    VectornavDriver(const VectornavDriver&) = delete;
    VectornavDriver& operator=(const VectornavDriver&) = delete;
    VectornavDriver(VectornavDriver&&) = delete;
    VectornavDriver& operator=(VectornavDriver&&) = delete;

    bool init() {
        const auto baud_rate = to_baud_rate(get_parameter("baud_rate").as_int());
        if (!baud_rate) {
            RCLCPP_ERROR(get_logger(),
                         "Invalid baud_rate. Valid values: 9600 19200 38400 57600 "
                         "115200 128000 230400 460800 921600");
            return false;
        }

        if (!connect(get_parameter("port").as_string(), *baud_rate)) {
            return false;
        }

        if (!validate_params()) {
            return false;
        }

        if (!configure_sensor_output()) {
            return false;
        }

        if (!configure_sensor_transforms()) {
            return false;
        }

        imu_frame_id_ = get_parameter("imu_frame_id").as_string();
        ins_frame_id_ = get_parameter("ins_frame_id").as_string();
        linear_accel_covariance_ = to_covariance(get_parameter("linear_accel_covariance").as_double_array());
        angular_vel_covariance_ = to_covariance(get_parameter("angular_vel_covariance").as_double_array());

        if (const auto datum = get_parameter("datum").as_double_array(); !datum.empty()) {
            datum_projection_.emplace(datum[0], datum[1], datum[2]);
            map_frame_id_ = get_parameter("map_frame_id").as_string();
            RCLCPP_INFO(get_logger(), "Odometry datum: lat=%f lon=%f alt=%f", datum[0], datum[1], datum[2]);
        }

        return true;
    }

    void publish_messages() {
        if (const auto cd = vs_.getNextMeasurement()) {
            const rclcpp::Time time = now();

            publish_imu(cd.get(), time);

            if (sensor_type_ != SensorType::VN100) {
                publish_fix(cd.get(), time);
                publish_velocity(cd.get(), time);
                publish_ins_status(cd.get());
                publish_gnss_status(cd.get());
                publish_gnss2_status(cd.get());
                publish_odom(cd.get(), time);
            }
        }
    }

  private:
    bool connect(const std::string& port, const VN::Sensor::BaudRate requested_baud_rate) {
        RCLCPP_INFO(get_logger(), "Starting Connection...");

        if (!optimize_serial_communication(port)) {
            RCLCPP_WARN(get_logger(), "Failed to optimize serial communication for %s", port.c_str());
        }

        if (const VN::Error error = vs_.autoConnect(port); error != VN::Error::None) {
            RCLCPP_ERROR(get_logger(), "Unable to connect to device %s", port.c_str());
            RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
            return false;
        }

        RCLCPP_INFO(get_logger(), "AutoConnect Complete..");

        if (requested_baud_rate != *vs_.connectedBaudRate()) {
            if (const auto error = vs_.changeBaudRate(requested_baud_rate); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Failure to change device to requested baud rate.");
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }
        }

        const uint32_t connected_baud_rate = static_cast<uint32_t>(*vs_.connectedBaudRate());

        VN::Registers::System::Model model_register;
        vs_.readRegister(&model_register);

        VN::Registers::System::FwVer firmware_register;
        vs_.readRegister(&firmware_register);

        VN::Registers::System::HwVer hardware_register;
        vs_.readRegister(&hardware_register);

        VN::Registers::System::Serial serial_register;
        vs_.readRegister(&serial_register);

        VN::Registers::System::UserTag user_tag_register;
        vs_.readRegister(&user_tag_register);

        RCLCPP_INFO(get_logger(), "Connected to %s @ %d baud", port.c_str(), connected_baud_rate);
        RCLCPP_INFO(get_logger(), "Model: %s", model_register.model.c_str());
        RCLCPP_INFO(get_logger(), "Firmware Version: %s", firmware_register.fwVer.c_str());
        RCLCPP_INFO(get_logger(), "Hardware Version : %d", hardware_register.hwVer);
        RCLCPP_INFO(get_logger(), "Serial Number : %d", serial_register.serialNum);
        RCLCPP_INFO(get_logger(), "User Tag : \"%s\"", user_tag_register.tag->c_str());

        switch (model_register.model[3]) {
            case '1':
                sensor_type_ = SensorType::VN100;
                break;
            case '2':
                sensor_type_ = SensorType::VN200;
                break;
            case '3':
                sensor_type_ = SensorType::VN300;
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unsupported sensor model: %s", model_register.model.c_str());
                return false;
        }

        return true;
    }

    bool validate_params() {
        const auto publish_rate = get_parameter("publish_rate").as_int();
        const auto linear_accel_covariance = get_parameter("linear_accel_covariance").as_double_array();
        const auto angular_vel_covariance = get_parameter("angular_vel_covariance").as_double_array();
        const auto imu_frame_id = get_parameter("imu_frame_id").as_string();
        const auto ins_frame_id = get_parameter("ins_frame_id").as_string();
        const auto gnss_a_frame_id = get_parameter("gnss_a_frame_id").as_string();
        const auto gnss_b_frame_id = get_parameter("gnss_b_frame_id").as_string();
        const auto datum = get_parameter("datum").as_double_array();
        const auto map_frame_id = get_parameter("map_frame_id").as_string();

        const bool has_ins = sensor_type_ == SensorType::VN200 || sensor_type_ == SensorType::VN300;
        const bool has_dual_gnss = sensor_type_ == SensorType::VN300;
        const uint16_t sensor_sample_rate = (sensor_type_ == SensorType::VN300) ? 400 : 800;

        if (publish_rate <= 0) {
            RCLCPP_ERROR(get_logger(), "publish_rate must be a positive integer");
            return false;
        }

        if (sensor_sample_rate % publish_rate != 0) {
            RCLCPP_ERROR(get_logger(),
                         "publish_rate (%ld) is not a valid divisor of sensor sample "
                         "rate (%d Hz)",
                         publish_rate, sensor_sample_rate);
            return false;
        }

        if (linear_accel_covariance.size() != 9) {
            RCLCPP_ERROR(get_logger(), "linear_accel_covariance must have 9 elements");
            return false;
        }

        if (angular_vel_covariance.size() != 9) {
            RCLCPP_ERROR(get_logger(), "angular_vel_covariance must have 9 elements");
            return false;
        }

        if (imu_frame_id.empty()) {
            RCLCPP_ERROR(get_logger(), "imu_frame_id must be set");
            return false;
        }

        if (!ins_frame_id.empty() && !has_ins) {
            RCLCPP_WARN(get_logger(), "ins_frame_id is not applicable to VN100 and will be ignored");
        } else if (ins_frame_id.empty() && has_ins) {
            RCLCPP_ERROR(get_logger(), "ins_frame_id must be set for VN200 and VN300");
            return false;
        }

        if (!gnss_a_frame_id.empty() && !has_ins) {
            RCLCPP_WARN(get_logger(), "gnss_a_frame_id is not applicable to VN100 and will be ignored");
        } else if (gnss_a_frame_id.empty() && has_ins) {
            RCLCPP_ERROR(get_logger(), "gnss_a_frame_id must be set for VN200 and VN300");
            return false;
        }

        if (!gnss_b_frame_id.empty() && !has_dual_gnss) {
            RCLCPP_WARN(get_logger(), "gnss_b_frame_id is not applicable to VN100/200 and will be ignored");
        } else if (gnss_b_frame_id.empty() && has_dual_gnss) {
            RCLCPP_ERROR(get_logger(), "gnss_b_frame_id must be set for VN300");
            return false;
        }

        if (datum.size() != 0 && datum.size() != 3) {
            RCLCPP_ERROR(get_logger(), "datum must have 3 elements: [lat, lon, alt]");
            return false;
        }

        if (!datum.empty() && map_frame_id.empty()) {
            RCLCPP_ERROR(get_logger(), "map_frame_id must be set when datum is provided");
            return false;
        }

        if (!datum.empty() && !has_ins) {
            RCLCPP_WARN(get_logger(), "datum is not applicable to VN100 and will be ignored");
        } else if (datum.empty() && has_ins) {
            RCLCPP_INFO(get_logger(), "No datum provided, odometry will not be published");
        }

        return true;
    }

    bool configure_sensor_output() {
        std::vector<VN::ConfigurationRegister*> config_registers;

        using namespace VN::Registers::System;

        AsyncOutputType async_data_output_type;
        async_data_output_type.ador = AsyncOutputType::Ador::OFF;
        config_registers.push_back(&async_data_output_type);

        const uint16_t sensor_sample_rate = (sensor_type_ == SensorType::VN300) ? 400 : 800;
        const uint16_t publish_rate = static_cast<uint16_t>(get_parameter("publish_rate").as_int());

        BinaryOutput1 binary_output_1_register;
        binary_output_1_register.asyncMode = 1;
        binary_output_1_register.rateDivisor = sensor_sample_rate / publish_rate;
        binary_output_1_register.imu.angularRate = 1;
        binary_output_1_register.imu.accel = 1;
        binary_output_1_register.attitude.quaternion = 1;
        binary_output_1_register.attitude.yprU = 1;
        binary_output_1_register.attitude.linBodyAcc = 1;
        binary_output_1_register.gnss.gnss1Fix = 1;
        binary_output_1_register.gnss.gnss1SatInfo = 1;
        binary_output_1_register.gnss.gnss1Status = 1;
        binary_output_1_register.gnss2.gnss2Status = 1;
        binary_output_1_register.ins.insStatus = 1;
        binary_output_1_register.ins.posLla = 1;
        binary_output_1_register.ins.velBody = 1;
        binary_output_1_register.ins.posU = 1;
        binary_output_1_register.ins.velU = 1;
        config_registers.push_back(&binary_output_1_register);

        BinaryOutput2 binary_output_2_register;
        binary_output_2_register.asyncMode = 0;
        binary_output_2_register.rateDivisor = 0;
        config_registers.push_back(&binary_output_2_register);

        BinaryOutput3 binary_output_3_register;
        binary_output_3_register.asyncMode = 0;
        binary_output_3_register.rateDivisor = 0;
        config_registers.push_back(&binary_output_3_register);

        for (auto& reg : config_registers) {
            if (const VN::Error error = vs_.writeRegister(reg); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Unable to configure Register %d -> %s", reg->id(),
                             VN::errorCodeToString(error).data());
                return false;
            }
        }

        return true;
    }

    bool configure_sensor_transforms() {
        const auto imu_frame_id = get_parameter("imu_frame_id").as_string();
        const auto ins_frame_id = get_parameter("ins_frame_id").as_string();
        const auto gnss_a_frame_id = get_parameter("gnss_a_frame_id").as_string();
        const auto gnss_b_frame_id = get_parameter("gnss_b_frame_id").as_string();

        const bool has_ins = sensor_type_ == SensorType::VN200 || sensor_type_ == SensorType::VN300;
        const bool need_imu_ins = has_ins && (imu_frame_id != ins_frame_id);
        const bool has_dual_gnss = sensor_type_ == SensorType::VN300;

        if (!need_imu_ins && !has_ins && !has_dual_gnss) {
            return true;
        }

        RCLCPP_INFO(get_logger(), "Waiting for TF transforms...");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            const bool imu_ins_ready =
                !need_imu_ins || tf_buffer_->canTransform(imu_frame_id, ins_frame_id, tf2::TimePointZero);
            const bool gnss_a_ready =
                !has_ins || tf_buffer_->canTransform(imu_frame_id, gnss_a_frame_id, tf2::TimePointZero);
            const bool gnss_b_ready =
                !has_dual_gnss || tf_buffer_->canTransform(imu_frame_id, gnss_b_frame_id, tf2::TimePointZero);

            if (imu_ins_ready && gnss_a_ready && gnss_b_ready) {
                break;
            }

            if (std::chrono::steady_clock::now() > deadline) {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for TF transforms");
                return false;
            }

            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        auto differs = [](float a, float b) { return std::abs(a - b) > 1e-3f; };

        // TODO: we temporarily disable this since InsRefOffset is a static register so we need a reset anyways.
        // to avoid this potentially breaking during comp we just don't run it and make ins = imu
        // if (need_imu_ins) {
        //     const auto tf = tf_buffer_->lookupTransform(imu_frame_id, ins_frame_id, tf2::TimePointZero);
        //     tf2::Quaternion q;
        //     tf2::fromMsg(tf.transform.rotation, q);
        //     imu_to_ins_ = Rotation(q.inverse());

        //     // FLU -> FRD; uncertainty = max(2.5% of max offset, 1cm) (VectorNav recommendation)
        //     const float x = static_cast<float>(tf.transform.translation.x);
        //     const float y = static_cast<float>(-tf.transform.translation.y);
        //     const float z = static_cast<float>(-tf.transform.translation.z);
        //     const float u = std::max(0.01f, 0.025f * std::max({std::abs(x), std::abs(y), std::abs(z)}));

        //     VN::Registers::INS::InsRefOffset current;
        //     // clang-format off
        //     const bool unchanged = vs_.readRegister(&current) == VN::Error::None &&
        //                            !differs(current.refOffsetX.value_or(0.f), x) &&
        //                            !differs(current.refOffsetY.value_or(0.f), y) &&
        //                            !differs(current.refOffsetZ.value_or(0.f), z);
        //     // clang-format on
        //     if (!unchanged) {
        //         VN::Registers::INS::InsRefOffset updated;
        //         updated.refOffsetX = x;
        //         updated.refOffsetY = y;
        //         updated.refOffsetZ = z;
        //         updated.refUncertX = u;
        //         updated.refUncertY = u;
        //         updated.refUncertZ = u;
        //         if (vs_.writeRegister(&updated) != VN::Error::None) {
        //             RCLCPP_ERROR(get_logger(), "Failed to write InsRefOffset");
        //             return false;
        //         }
        //     }
        //     RCLCPP_INFO(get_logger(), "InsRefOffset [write FRD]: %.3f %.3f %.3f (u=%.3f)%s", x, y, z, u,
        //                 unchanged ? " [unchanged]" : "");
        // }

        if (has_ins) {
            const auto tf = tf_buffer_->lookupTransform(imu_frame_id, gnss_a_frame_id, tf2::TimePointZero);
            const float x = static_cast<float>(tf.transform.translation.x);
            const float y = static_cast<float>(-tf.transform.translation.y);
            const float z = static_cast<float>(-tf.transform.translation.z);

            VN::Registers::GNSS::GnssAOffset current;
            // clang-format off
            const bool unchanged = vs_.readRegister(&current) == VN::Error::None &&
                                    !differs(current.positionX.value_or(0.f), x) &&
                                    !differs(current.positionY.value_or(0.f), y) &&
                                    !differs(current.positionZ.value_or(0.f), z);
            // clang-format on
            if (!unchanged) {
                VN::Registers::GNSS::GnssAOffset updated;
                updated.positionX = x;
                updated.positionY = y;
                updated.positionZ = z;
                if (vs_.writeRegister(&updated) != VN::Error::None) {
                    RCLCPP_ERROR(get_logger(), "Failed to write GnssAOffset");
                    return false;
                }
            }
            RCLCPP_INFO(get_logger(), "GnssAOffset [write FRD]: %.3f %.3f %.3f%s", x, y, z,
                        unchanged ? " [unchanged]" : "");
        }

        if (has_dual_gnss) {
            const auto tf_a = tf_buffer_->lookupTransform(imu_frame_id, gnss_a_frame_id, tf2::TimePointZero);
            const auto tf_b = tf_buffer_->lookupTransform(imu_frame_id, gnss_b_frame_id, tf2::TimePointZero);
            // Baseline A->B in body frame (FLU -> FRD)
            const float x = static_cast<float>(tf_b.transform.translation.x - tf_a.transform.translation.x);
            const float y = static_cast<float>(-(tf_b.transform.translation.y - tf_a.transform.translation.y));
            const float z = static_cast<float>(-(tf_b.transform.translation.z - tf_a.transform.translation.z));
            const float u = std::max(0.01f, 0.025f * std::max({std::abs(x), std::abs(y), std::abs(z)}));

            VN::Registers::GNSSCompass::GnssCompassBaseline current;
            // clang-format off
            const bool unchanged = vs_.readRegister(&current) == VN::Error::None &&
                                    !differs(current.positionX.value_or(0.f), x) &&
                                    !differs(current.positionY.value_or(0.f), y) &&
                                    !differs(current.positionZ.value_or(0.f), z);
            // clang-format on
            if (!unchanged) {
                VN::Registers::GNSSCompass::GnssCompassBaseline updated;
                updated.positionX = x;
                updated.positionY = y;
                updated.positionZ = z;
                updated.uncertaintyX = u;
                updated.uncertaintyY = u;
                updated.uncertaintyZ = u;
                if (vs_.writeRegister(&updated) != VN::Error::None) {
                    RCLCPP_ERROR(get_logger(), "Failed to write GnssCompassBaseline");
                    return false;
                }
            }
            RCLCPP_INFO(get_logger(), "GnssCompassBaseline [write FRD]: %.3f %.3f %.3f (u=%.3f)%s", x, y, z, u,
                        unchanged ? " [unchanged]" : "");
        }

        // Readback final register values for validation (FRD -> FLU for display)
        if (need_imu_ins) {
            VN::Registers::INS::InsRefOffset reg;
            if (vs_.readRegister(&reg) == VN::Error::None) {
                RCLCPP_INFO(get_logger(), "InsRefOffset [readback FLU]: %.3f %.3f %.3f (u=%.3f %.3f %.3f)",
                            reg.refOffsetX.value_or(0.f), -reg.refOffsetY.value_or(0.f), -reg.refOffsetZ.value_or(0.f),
                            reg.refUncertX.value_or(0.f), reg.refUncertY.value_or(0.f), reg.refUncertZ.value_or(0.f));
            }
        }
        if (has_ins) {
            VN::Registers::GNSS::GnssAOffset reg;
            if (vs_.readRegister(&reg) == VN::Error::None) {
                RCLCPP_INFO(get_logger(), "GnssAOffset [readback FLU]: %.3f %.3f %.3f", reg.positionX.value_or(0.f),
                            -reg.positionY.value_or(0.f), -reg.positionZ.value_or(0.f));
            }
        }
        if (has_dual_gnss) {
            VN::Registers::GNSSCompass::GnssCompassBaseline reg;
            if (vs_.readRegister(&reg) == VN::Error::None) {
                RCLCPP_INFO(get_logger(), "GnssCompassBaseline [readback FLU]: %.3f %.3f %.3f (u=%.3f %.3f %.3f)",
                            reg.positionX.value_or(0.f), -reg.positionY.value_or(0.f), -reg.positionZ.value_or(0.f),
                            reg.uncertaintyX.value_or(0.f), reg.uncertaintyY.value_or(0.f),
                            reg.uncertaintyZ.value_or(0.f));
            }
        }

        return true;
    }

    void publish_imu(const VN::CompositeData* cd, const rclcpp::Time& time) {
        if (imu_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& quat = cd->attitude.quaternion;
        const auto& angular_vel = cd->imu.angularRate;
        const auto& accel = cd->attitude.linBodyAcc;
        const auto& ypr_u = cd->attitude.yprU;
        const auto& ins_status = cd->ins.insStatus;

        const bool is_vn100 = sensor_type_ == SensorType::VN100;
        const bool ins_valid = ins_status && ins_status->imuErr == 0 && ins_status->mode != MODE_ALIGNING;
        if (!quat || !angular_vel || !accel || !ypr_u || (!is_vn100 && !ins_valid)) {
            return;
        }

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = time;
        msg.header.frame_id = imu_frame_id_;

        msg.orientation = tf2::toMsg(ned_to_enu(to_tf2_quaternion(*quat)));
        msg.angular_velocity = frd_to_flu(to_vector3(*angular_vel));
        msg.linear_acceleration = frd_to_flu(to_vector3(*accel));

        // ypr -> roll, pitch, yaw
        msg.orientation_covariance =
            frd_to_flu(diag_covariance(std::pow((*ypr_u)[2] * M_PI / 180.0, 2), std::pow((*ypr_u)[1] * M_PI / 180.0, 2),
                                       std::pow((*ypr_u)[0] * M_PI / 180.0, 2)));
        msg.angular_velocity_covariance = angular_vel_covariance_;
        msg.linear_acceleration_covariance = linear_accel_covariance_;

        imu_publisher_->publish(msg);
    }

    void publish_fix(const VN::CompositeData* cd, const rclcpp::Time& time) {
        if (fix_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& lla = cd->ins.posLla;
        const auto& fix = cd->gnss.gnss1Fix;
        const auto& sat_info = cd->gnss.gnss1SatInfo;
        const auto& pos_u = cd->ins.posU;
        const auto& ins_status = cd->ins.insStatus;

        if (!lla || !fix || !sat_info || !pos_u || !ins_status) {
            return;
        }

        if (ins_status->gnssErr != 0 || ins_status->gnssFix != 1 || ins_status->mode == MODE_NOT_TRACKING ||
            ins_status->mode == MODE_GNSS_LOST) {
            return;
        }

        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = time;
        msg.header.frame_id = ins_frame_id_;

        msg.latitude = lla->lat;
        msg.longitude = lla->lon;
        msg.altitude = lla->alt;

        using Fix = VN::Registers::GNSS::GnssSolLla::Gnss1Fix;
        switch (static_cast<Fix>(*fix)) {
            case Fix::Fix2D:
            case Fix::Fix3D:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                break;
            case Fix::SBAS:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                break;
            case Fix::RtkFloat:
            case Fix::RtkFix:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                break;
            default:
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                break;
        }

        for (std::size_t i = 0; i < sat_info->numSats; ++i) {
            switch (sat_info->sys[i]) {
                case 0:  // GPS
                    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
                    break;
                case 2:  // Galileo
                    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
                    break;
                case 3:  // BeiDou
                    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
                    break;
                case 6:  // GLONASS
                    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
                    break;
                default:
                    break;
            }
        }

        // posU is in world frame, no rotation needed
        msg.position_covariance = diag_covariance(std::pow(*pos_u, 2));
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        fix_publisher_->publish(msg);
    }

    void publish_velocity(const VN::CompositeData* cd, const rclcpp::Time& time) {
        if (velocity_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& vel = cd->ins.velBody;
        const auto& angular_vel = cd->imu.angularRate;
        const auto& vel_u = cd->ins.velU;
        const auto& ins_status = cd->ins.insStatus;

        if (!vel || !angular_vel || !vel_u || !ins_status) {
            return;
        }

        if (ins_status->imuErr != 0 || ins_status->mode == MODE_NOT_TRACKING || ins_status->mode == MODE_GNSS_LOST) {
            return;
        }

        geometry_msgs::msg::TwistWithCovarianceStamped msg;
        msg.header.stamp = time;
        msg.header.frame_id = ins_frame_id_;

        msg.twist.twist.linear = imu_to_ins_.rotate(frd_to_flu(to_vector3(*vel)));
        msg.twist.twist.angular = imu_to_ins_.rotate(frd_to_flu(to_vector3(*angular_vel)));

        msg.twist.covariance =
            block_diag_covariance(imu_to_ins_.rotate(frd_to_flu(diag_covariance(std::pow(*vel_u, 2)))),
                                  imu_to_ins_.rotate(angular_vel_covariance_));

        velocity_publisher_->publish(msg);
    }

    void publish_odom(const VN::CompositeData* cd, const rclcpp::Time& time) {
        if (!datum_projection_ || odom_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& lla = cd->ins.posLla;
        const auto& quat = cd->attitude.quaternion;
        const auto& vel = cd->ins.velBody;
        const auto& angular_vel = cd->imu.angularRate;
        const auto& pos_u = cd->ins.posU;
        const auto& vel_u = cd->ins.velU;
        const auto& ypr_u = cd->attitude.yprU;
        const auto& ins_status = cd->ins.insStatus;

        if (!lla || !quat || !vel || !angular_vel || !pos_u || !vel_u || !ypr_u || !ins_status) {
            return;
        }

        if (ins_status->imuErr != 0 || ins_status->gnssErr != 0 || ins_status->gnssFix != 1 ||
            ins_status->mode != MODE_TRACKING) {
            return;
        }

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = time;
        msg.header.frame_id = map_frame_id_;
        msg.child_frame_id = ins_frame_id_;

        double east, north, up;
        datum_projection_->Forward(lla->lat, lla->lon, lla->alt, east, north, up);
        msg.pose.pose.position.x = east;
        msg.pose.pose.position.y = north;
        msg.pose.pose.position.z = up;

        msg.pose.pose.orientation = tf2::toMsg(imu_to_ins_.rotate(ned_to_enu(to_tf2_quaternion(*quat))));
        msg.twist.twist.linear = imu_to_ins_.rotate(frd_to_flu(to_vector3(*vel)));
        msg.twist.twist.angular = imu_to_ins_.rotate(frd_to_flu(to_vector3(*angular_vel)));

        // posU is in world frame, no rotation needed
        // ypr -> roll, pitch, yaw
        msg.pose.covariance =
            block_diag_covariance(diag_covariance(std::pow(*pos_u, 2)),
                                  imu_to_ins_.rotate(frd_to_flu(diag_covariance(
                                      std::pow((*ypr_u)[2] * M_PI / 180.0, 2), std::pow((*ypr_u)[1] * M_PI / 180.0, 2),
                                      std::pow((*ypr_u)[0] * M_PI / 180.0, 2)))));

        msg.twist.covariance =
            block_diag_covariance(imu_to_ins_.rotate(frd_to_flu(diag_covariance(std::pow(*vel_u, 2)))),
                                  imu_to_ins_.rotate(angular_vel_covariance_));

        odom_publisher_->publish(msg);
    }

    void publish_ins_status(const VN::CompositeData* cd) {
        if (ins_status_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& ins_status = cd->ins.insStatus;

        if (!ins_status) {
            return;
        }

        std_msgs::msg::UInt16 msg;
        msg.data = static_cast<uint16_t>(*ins_status);
        ins_status_publisher_->publish(msg);
    }

    void publish_gnss_status(const VN::CompositeData* cd) {
        if (gnss_status_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& gnss_status = cd->gnss.gnss1Status;

        if (!gnss_status) {
            return;
        }

        std_msgs::msg::UInt16 msg;
        msg.data = static_cast<uint16_t>(*gnss_status);
        gnss_status_publisher_->publish(msg);
    }

    void publish_gnss2_status(const VN::CompositeData* cd) {
        if (gnss2_status_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& gnss2_status = cd->gnss2.gnss2Status;

        if (!gnss2_status) {
            return;
        }

        std_msgs::msg::UInt16 msg;
        msg.data = static_cast<uint16_t>(*gnss2_status);
        gnss2_status_publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr ins_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr gnss_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr gnss2_status_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    VN::Sensor vs_;
    SensorType sensor_type_ = SensorType::VN100;

    std::string imu_frame_id_;
    std::string ins_frame_id_;
    std::string map_frame_id_;
    std::array<double, 9> linear_accel_covariance_;
    std::array<double, 9> angular_vel_covariance_;
    std::optional<GeographicLib::LocalCartesian> datum_projection_;
    Rotation imu_to_ins_;
};
