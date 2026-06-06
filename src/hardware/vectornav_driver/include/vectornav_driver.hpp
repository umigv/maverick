#pragma once

#include <GeographicLib/LocalCartesian.hpp>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
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

inline bool optimize_serial_communication(const std::string& port) {
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
inline bool optimize_serial_communication(const std::string&) { return false; }
#endif

inline std::optional<VN::Sensor::BaudRate> to_baud_rate(int baud) {
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

inline bool within_register_precision(float a, float b) { return std::abs(a - b) <= 1e-3f; }

class VectornavDriver : public rclcpp::Node {
    static constexpr uint16_t SENSOR_SAMPLE_RATE = 400;
    static constexpr uint16_t MODE_NOT_TRACKING = 0;
    static constexpr uint16_t MODE_ALIGNING = 1;
    static constexpr uint16_t MODE_TRACKING = 2;
    static constexpr uint16_t MODE_GNSS_LOST = 3;

  public:
    VectornavDriver() : Node("vectornav_driver") {
        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
        fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("vectornav/fix", 10);
        velocity_publisher_ =
            create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("vectornav/velocity", 10);
        ins_status_publisher_ = create_publisher<std_msgs::msg::UInt16>("vectornav/ins_status", 10);
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("vectornav/odom", 10);
        gnss_status_publisher_ = create_publisher<std_msgs::msg::UInt16>("vectornav/gnss_status", 10);
        gnss2_status_publisher_ = create_publisher<std_msgs::msg::UInt16>("vectornav/gnss2_status", 10);
        yaw_uncertainty_publisher_ = create_publisher<std_msgs::msg::Float32>("vectornav/yaw_uncertainty", 10);
        gnss_compass_signal_health_publisher_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("vectornav/gnss_compass_signal_health", 10);
        gnss_compass_startup_status_publisher_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("vectornav/gnss_compass_startup_status", 10);

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
        if (!load_params()) {
            return false;
        }

        if (!connect()) {
            return false;
        }

        if (!configure_sensor_transforms()) {
            return false;
        }

        if (!configure_sensor_output()) {
            return false;
        }

        measurement_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        status_register_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        measurement_timer_ = create_wall_timer(
            std::chrono::milliseconds(0), std::bind(&VectornavDriver::publish_measurements, this), measurement_group_);
        status_register_timer_ =
            create_wall_timer(std::chrono::duration<double>(status_register_poll_period_s_),
                              std::bind(&VectornavDriver::publish_status_register, this), status_register_group_);

        return true;
    }

  private:
    bool load_params() {
        declare_parameter<std::string>("port", "/dev/vn300");
        declare_parameter<int>("baud_rate", static_cast<int>(VN::Sensor::BaudRate::Baud115200));
        declare_parameter<double>("measurement_publish_period_s", 0.01);
        declare_parameter<double>("status_register_poll_period_s", 1.0);
        declare_parameter<std::string>("imu_frame_id", "vectornav");
        declare_parameter<std::string>("ins_frame_id", "vectornav");
        declare_parameter<std::string>("gnss_a_frame_id", "");
        declare_parameter<std::string>("gnss_b_frame_id", "");
        declare_parameter<std::vector<double>>("linear_accel_covariance", std::vector<double>(9, 0.0));
        declare_parameter<std::vector<double>>("angular_vel_covariance", std::vector<double>(9, 0.0));
        declare_parameter<std::vector<double>>("datum", std::vector<double>{});
        declare_parameter<std::string>("map_frame_id", "map");
        declare_parameter<bool>("require_attitude", true);

        port_ = get_parameter("port").as_string();

        if (const auto baud_rate = to_baud_rate(get_parameter("baud_rate").as_int()); !baud_rate) {
            RCLCPP_ERROR(get_logger(),
                         "Invalid baud_rate. Valid values: 9600 19200 38400 57600 "
                         "115200 128000 230400 460800 921600");
            return false;
        } else {
            baud_rate_ = *baud_rate;
        }

        if (const auto period = get_parameter("measurement_publish_period_s").as_double(); period <= 0.0) {
            RCLCPP_ERROR(get_logger(), "measurement_publish_period_s must be positive");
            return false;
        } else {
            const auto divisor = static_cast<int>(std::round(SENSOR_SAMPLE_RATE * period));
            if (divisor < 1 || SENSOR_SAMPLE_RATE % divisor != 0) {
                RCLCPP_ERROR(get_logger(),
                             "publish_period_s (%.4f) does not correspond to an integer divisor of the "
                             "%d Hz sensor sample rate",
                             period, SENSOR_SAMPLE_RATE);
                return false;
            }
            rate_divisor_ = static_cast<uint16_t>(divisor);
        }

        if (status_register_poll_period_s_ = get_parameter("status_register_poll_period_s").as_double();
            status_register_poll_period_s_ <= 0.0) {
            RCLCPP_ERROR(get_logger(), "status_register_poll_period_s must be positive");
            return false;
        }

        if (imu_frame_id_ = get_parameter("imu_frame_id").as_string(); imu_frame_id_.empty()) {
            RCLCPP_ERROR(get_logger(), "imu_frame_id must be set");
            return false;
        }

        if (ins_frame_id_ = get_parameter("ins_frame_id").as_string(); ins_frame_id_.empty()) {
            RCLCPP_ERROR(get_logger(), "ins_frame_id must be set");
            return false;
        }

        if (gnss_a_frame_id_ = get_parameter("gnss_a_frame_id").as_string(); gnss_a_frame_id_.empty()) {
            RCLCPP_ERROR(get_logger(), "gnss_a_frame_id must be set");
            return false;
        }

        if (gnss_b_frame_id_ = get_parameter("gnss_b_frame_id").as_string(); gnss_b_frame_id_.empty()) {
            RCLCPP_ERROR(get_logger(), "gnss_b_frame_id must be set");
            return false;
        }

        if (const auto cov = get_parameter("linear_accel_covariance").as_double_array(); cov.size() != 9) {
            RCLCPP_ERROR(get_logger(), "linear_accel_covariance must have 9 elements");
            return false;
        } else {
            linear_accel_covariance_ = to_covariance(cov);
        }

        if (const auto cov = get_parameter("angular_vel_covariance").as_double_array(); cov.size() != 9) {
            RCLCPP_ERROR(get_logger(), "angular_vel_covariance must have 9 elements");
            return false;
        } else {
            angular_vel_covariance_ = to_covariance(cov);
        }

        if (const auto datum = get_parameter("datum").as_double_array(); !datum.empty()) {
            if (datum.size() != 3) {
                RCLCPP_ERROR(get_logger(), "datum must have 3 elements: [lat, lon, alt]");
                return false;
            }

            datum_projection_.emplace(datum[0], datum[1], datum[2]);
            RCLCPP_INFO(get_logger(), "Odometry datum: lat=%f lon=%f alt=%f", datum[0], datum[1], datum[2]);

            map_frame_id_ = get_parameter("map_frame_id").as_string();
            if (map_frame_id_.empty()) {
                RCLCPP_ERROR(get_logger(), "map_frame_id must be set when datum is provided");
                return false;
            }
        } else {
            RCLCPP_INFO(get_logger(), "No datum provided, odometry will not be published");
        }

        require_attitude_ = get_parameter("require_attitude").as_bool();

        return true;
    }

    bool connect() {
        RCLCPP_INFO(get_logger(), "Starting Connection...");

        if (!optimize_serial_communication(port_)) {
            RCLCPP_WARN(get_logger(), "Failed to optimize serial communication for %s", port_.c_str());
        }

        if (const VN::Error error = vs_.autoConnect(port_); error != VN::Error::None) {
            RCLCPP_ERROR(get_logger(), "Unable to connect to device %s", port_.c_str());
            RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
            return false;
        }

        if (baud_rate_ != *vs_.connectedBaudRate()) {
            if (const auto error = vs_.changeBaudRate(baud_rate_); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Failed to change device to requested baud rate");
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }
        }

        const uint32_t connected_baud_rate = static_cast<uint32_t>(*vs_.connectedBaudRate());
        RCLCPP_INFO(get_logger(), "Connected to %s @ %d baud", port_.c_str(), connected_baud_rate);

        VN::Registers::System::Model model_register;
        if (const auto error = vs_.readRegister(&model_register); error != VN::Error::None) {
            RCLCPP_ERROR(get_logger(), "Failed to read model register");
            RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
            return false;
        } else if (model_register.model.find("300") == VN::String<24>::npos) {
            RCLCPP_ERROR(get_logger(), "Unsupported sensor model: %s (only VN300 is supported)",
                         model_register.model.c_str());
            return false;
        } else {
            RCLCPP_INFO(get_logger(), "Model: %s", model_register.model.c_str());
        }

        VN::Registers::System::FwVer firmware_register;
        if (const auto error = vs_.readRegister(&firmware_register); error != VN::Error::None) {
            RCLCPP_WARN(get_logger(), "Failed to read firmware version register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        } else {
            RCLCPP_INFO(get_logger(), "Firmware Version: %s", firmware_register.fwVer.c_str());
        }

        VN::Registers::System::HwVer hardware_register;
        if (const auto error = vs_.readRegister(&hardware_register); error != VN::Error::None) {
            RCLCPP_WARN(get_logger(), "Failed to read hardware version register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        } else {
            RCLCPP_INFO(get_logger(), "Hardware Version: %d", hardware_register.hwVer);
        }

        VN::Registers::System::Serial serial_register;
        if (const auto error = vs_.readRegister(&serial_register); error != VN::Error::None) {
            RCLCPP_WARN(get_logger(), "Failed to read serial number register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        } else {
            RCLCPP_INFO(get_logger(), "Serial Number: %d", serial_register.serialNum);
        }

        VN::Registers::System::UserTag user_tag_register;
        if (const auto error = vs_.readRegister(&user_tag_register); error != VN::Error::None) {
            RCLCPP_WARN(get_logger(), "Failed to read user tag register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        } else {
            RCLCPP_INFO(get_logger(), "User Tag: %s", user_tag_register.tag.value_or("").c_str());
        }

        return true;
    }

    bool configure_sensor_transforms() {
        RCLCPP_INFO(get_logger(), "Waiting for TF transforms...");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            const bool ins_ready = tf_buffer_->canTransform(imu_frame_id_, ins_frame_id_, tf2::TimePointZero);
            const bool gnss_a_ready = tf_buffer_->canTransform(imu_frame_id_, gnss_a_frame_id_, tf2::TimePointZero);
            const bool gnss_b_ready = tf_buffer_->canTransform(imu_frame_id_, gnss_b_frame_id_, tf2::TimePointZero);

            if (ins_ready && gnss_a_ready && gnss_b_ready) {
                break;
            }

            if (std::chrono::steady_clock::now() > deadline) {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for TF transforms");

                if (!ins_ready) {
                    RCLCPP_ERROR(get_logger(), "Missing transform: %s -> %s", imu_frame_id_.c_str(),
                                 ins_frame_id_.c_str());
                }

                if (!gnss_a_ready) {
                    RCLCPP_ERROR(get_logger(), "Missing transform: %s -> %s", imu_frame_id_.c_str(),
                                 gnss_a_frame_id_.c_str());
                }

                if (!gnss_b_ready) {
                    RCLCPP_ERROR(get_logger(), "Missing transform: %s -> %s", imu_frame_id_.c_str(),
                                 gnss_b_frame_id_.c_str());
                }

                return false;
            }

            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        try {
            const auto ins_tf = tf_buffer_->lookupTransform(imu_frame_id_, ins_frame_id_, tf2::TimePointZero);
            const auto gnss_a_tf = tf_buffer_->lookupTransform(imu_frame_id_, gnss_a_frame_id_, tf2::TimePointZero);
            const auto gnss_b_tf = tf_buffer_->lookupTransform(imu_frame_id_, gnss_b_frame_id_, tf2::TimePointZero);

            return configure_ins_ref_offset(ins_tf) && configure_gnss_a_offset(ins_tf, gnss_a_tf) &&
                   configure_gnss_compass_baseline(gnss_a_tf, gnss_b_tf);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(get_logger(), "TF transform error: %s", ex.what());
            return false;
        }
    }

    bool configure_ins_ref_offset(const geometry_msgs::msg::TransformStamped& ins_tf) {
        tf2::Quaternion q;
        tf2::fromMsg(ins_tf.transform.rotation, q);
        imu_to_ins_ = Rotation(q.inverse());

        if (imu_frame_id_ == ins_frame_id_) {
            return true;
        }

        // FLU -> FRD; uncertainty = max(2.5% of max offset, 1cm) (VectorNav recommendation)
        const float x = static_cast<float>(ins_tf.transform.translation.x);
        const float y = static_cast<float>(-ins_tf.transform.translation.y);
        const float z = static_cast<float>(-ins_tf.transform.translation.z);
        const float u = std::max(0.01f, 0.025f * std::max({std::abs(x), std::abs(y), std::abs(z)}));

        VN::Registers::INS::InsRefOffset current;
        // clang-format off
        const bool unchanged = vs_.readRegister(&current) == VN::Error::None &&
                               within_register_precision(current.refOffsetX.value_or(0.f), x) &&
                               within_register_precision(current.refOffsetY.value_or(0.f), y) &&
                               within_register_precision(current.refOffsetZ.value_or(0.f), z);
        // clang-format on
        RCLCPP_INFO(get_logger(), "InsRefOffset [write FRD]: %.3f %.3f %.3f (u=%.3f)%s", x, y, z, u,
                    unchanged ? " [unchanged]" : "");

        if (!unchanged) {
            VN::Registers::INS::InsRefOffset updated;
            updated.refOffsetX = x;
            updated.refOffsetY = y;
            updated.refOffsetZ = z;
            updated.refUncertX = u;
            updated.refUncertY = u;
            updated.refUncertZ = u;
            if (const auto error = vs_.writeRegister(&updated); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Failed to write InsRefOffset");
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }

            if (const auto error = vs_.writeSettings(); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Failed to save settings to NVRAM");
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }

            RCLCPP_INFO(get_logger(), "Resetting sensor for InsRefOffset to take effect...");
            if (const auto error = vs_.reset(); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Failed to reset sensor");
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }
        }

        VN::Registers::INS::InsRefOffset reg;
        if (const auto error = vs_.readRegister(&reg); error == VN::Error::None) {
            RCLCPP_INFO(get_logger(), "InsRefOffset [readback FLU]: %.3f %.3f %.3f (u=%.3f %.3f %.3f)",
                        reg.refOffsetX.value_or(0.f), -reg.refOffsetY.value_or(0.f), -reg.refOffsetZ.value_or(0.f),
                        reg.refUncertX.value_or(0.f), reg.refUncertY.value_or(0.f), reg.refUncertZ.value_or(0.f));
        } else {
            RCLCPP_WARN(get_logger(), "Failed to read back InsRefOffset register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        }

        return true;
    }

    bool configure_gnss_a_offset(const geometry_msgs::msg::TransformStamped& ins_tf,
                                 const geometry_msgs::msg::TransformStamped& gnss_a_tf) {
        // ins -> gnss_a in body frame (FLU -> FRD)
        const float x = static_cast<float>(gnss_a_tf.transform.translation.x - ins_tf.transform.translation.x);
        const float y = static_cast<float>(-(gnss_a_tf.transform.translation.y - ins_tf.transform.translation.y));
        const float z = static_cast<float>(-(gnss_a_tf.transform.translation.z - ins_tf.transform.translation.z));

        VN::Registers::GNSS::GnssAOffset current;
        // clang-format off
        const bool unchanged = vs_.readRegister(&current) == VN::Error::None &&
                               within_register_precision(current.positionX.value_or(0.f), x) &&
                               within_register_precision(current.positionY.value_or(0.f), y) &&
                               within_register_precision(current.positionZ.value_or(0.f), z);
        // clang-format on
        if (!unchanged) {
            VN::Registers::GNSS::GnssAOffset updated;
            updated.positionX = x;
            updated.positionY = y;
            updated.positionZ = z;
            if (const auto error = vs_.writeRegister(&updated); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Failed to write GnssAOffset");
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }
        }
        RCLCPP_INFO(get_logger(), "GnssAOffset [write FRD]: %.3f %.3f %.3f%s", x, y, z,
                    unchanged ? " [unchanged]" : "");

        VN::Registers::GNSS::GnssAOffset reg;
        if (const auto error = vs_.readRegister(&reg); error == VN::Error::None) {
            RCLCPP_INFO(get_logger(), "GnssAOffset [readback FLU]: %.3f %.3f %.3f", reg.positionX.value_or(0.f),
                        -reg.positionY.value_or(0.f), -reg.positionZ.value_or(0.f));
        } else {
            RCLCPP_WARN(get_logger(), "Failed to read back GnssAOffset register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        }

        return true;
    }

    bool configure_gnss_compass_baseline(const geometry_msgs::msg::TransformStamped& gnss_a_tf,
                                         const geometry_msgs::msg::TransformStamped& gnss_b_tf) {
        // gnss_a -> gnss_b in body frame (FLU -> FRD)
        const float x = static_cast<float>(gnss_b_tf.transform.translation.x - gnss_a_tf.transform.translation.x);
        const float y = static_cast<float>(-(gnss_b_tf.transform.translation.y - gnss_a_tf.transform.translation.y));
        const float z = static_cast<float>(-(gnss_b_tf.transform.translation.z - gnss_a_tf.transform.translation.z));
        const float u = std::max(0.01f, 0.025f * std::max({std::abs(x), std::abs(y), std::abs(z)}));

        VN::Registers::GNSSCompass::GnssCompassBaseline current;
        // clang-format off
        const bool unchanged = vs_.readRegister(&current) == VN::Error::None &&
                               within_register_precision(current.positionX.value_or(0.f), x) &&
                               within_register_precision(current.positionY.value_or(0.f), y) &&
                               within_register_precision(current.positionZ.value_or(0.f), z);
        // clang-format on
        if (!unchanged) {
            VN::Registers::GNSSCompass::GnssCompassBaseline updated;
            updated.positionX = x;
            updated.positionY = y;
            updated.positionZ = z;
            updated.uncertaintyX = u;
            updated.uncertaintyY = u;
            updated.uncertaintyZ = u;
            if (const auto error = vs_.writeRegister(&updated); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Failed to write GnssCompassBaseline");
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }
        }
        RCLCPP_INFO(get_logger(), "GnssCompassBaseline [write FRD]: %.3f %.3f %.3f (u=%.3f)%s", x, y, z, u,
                    unchanged ? " [unchanged]" : "");

        VN::Registers::GNSSCompass::GnssCompassBaseline reg;
        if (const auto error = vs_.readRegister(&reg); error == VN::Error::None) {
            RCLCPP_INFO(get_logger(), "GnssCompassBaseline [readback FLU]: %.3f %.3f %.3f (u=%.3f %.3f %.3f)",
                        reg.positionX.value_or(0.f), -reg.positionY.value_or(0.f), -reg.positionZ.value_or(0.f),
                        reg.uncertaintyX.value_or(0.f), reg.uncertaintyY.value_or(0.f), reg.uncertaintyZ.value_or(0.f));
        } else {
            RCLCPP_WARN(get_logger(), "Failed to read back GnssCompassBaseline register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        }

        return true;
    }

    bool configure_sensor_output() {
        std::vector<VN::ConfigurationRegister*> config_registers;

        VN::Registers::System::AsyncOutputType async_data_output_type;
        async_data_output_type.ador = VN::Registers::System::AsyncOutputType::Ador::OFF;
        config_registers.push_back(&async_data_output_type);

        VN::Registers::System::BinaryOutput1 binary_output_1_register;
        binary_output_1_register.asyncMode = 1;
        binary_output_1_register.rateDivisor = rate_divisor_;
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

        VN::Registers::System::BinaryOutput2 binary_output_2_register;
        binary_output_2_register.asyncMode = 0;
        binary_output_2_register.rateDivisor = 0;
        config_registers.push_back(&binary_output_2_register);

        VN::Registers::System::BinaryOutput3 binary_output_3_register;
        binary_output_3_register.asyncMode = 0;
        binary_output_3_register.rateDivisor = 0;
        config_registers.push_back(&binary_output_3_register);

        for (auto& reg : config_registers) {
            if (const VN::Error error = vs_.writeRegister(reg); error != VN::Error::None) {
                RCLCPP_ERROR(get_logger(), "Unable to configure Register %d", reg->id());
                RCLCPP_ERROR(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
                return false;
            }
        }

        return true;
    }

    void publish_measurements() {
        if (const auto cd = vs_.getNextMeasurement()) {
            const rclcpp::Time time = now();

            publish_imu(cd.get(), time);
            publish_fix(cd.get(), time);
            publish_velocity(cd.get(), time);
            publish_uint16(ins_status_publisher_, cd->ins.insStatus);
            publish_uint16(gnss_status_publisher_, cd->gnss.gnss1Status);
            publish_uint16(gnss2_status_publisher_, cd->gnss2.gnss2Status);
            publish_odom(cd.get(), time);
            publish_yaw_uncertainty(cd.get());
        }
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

        const bool imu_valid = ins_status && ins_status->imuErr == 0 && angular_vel;
        const bool attitude_valid = quat && accel && ypr_u && ins_status && ins_status->mode != MODE_ALIGNING;

        if (!imu_valid || (!attitude_valid && require_attitude_)) {
            return;
        }

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = time;
        msg.header.frame_id = imu_frame_id_;

        msg.angular_velocity = frd_to_flu(to_vector3(*angular_vel));
        msg.angular_velocity_covariance = angular_vel_covariance_;

        if (attitude_valid) {
            msg.orientation = tf2::toMsg(ned_to_enu(to_tf2_quaternion(*quat)));
            msg.linear_acceleration = frd_to_flu(to_vector3(*accel));
            // ypr -> roll, pitch, yaw
            msg.orientation_covariance = frd_to_flu(diag_covariance(std::pow((*ypr_u)[2] * M_PI / 180.0, 2),
                                                                    std::pow((*ypr_u)[1] * M_PI / 180.0, 2),
                                                                    std::pow((*ypr_u)[0] * M_PI / 180.0, 2)));
            msg.linear_acceleration_covariance = linear_accel_covariance_;
        } else {
            // ROS convention: covariance[0] = -1 means field is unknown
            msg.orientation_covariance[0] = -1;
            msg.linear_acceleration_covariance[0] = -1;
        }

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

    template <typename T>
    void publish_uint16(rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr& publisher, const std::optional<T>& value) {
        if (publisher->get_subscription_count() == 0 || !value) {
            return;
        }

        std_msgs::msg::UInt16 msg;
        msg.data = static_cast<uint16_t>(*value);
        publisher->publish(msg);
    }

    void publish_yaw_uncertainty(const VN::CompositeData* cd) {
        if (yaw_uncertainty_publisher_->get_subscription_count() == 0) {
            return;
        }

        const auto& ypr_u = cd->attitude.yprU;
        if (!ypr_u) {
            return;
        }

        std_msgs::msg::Float32 msg;
        msg.data = (*ypr_u)[0];
        yaw_uncertainty_publisher_->publish(msg);
    }

    void publish_status_register() {
        VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus signal_health;
        if (const auto error = vs_.readRegister(&signal_health); error == VN::Error::None) {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {signal_health.numSatsPvtA,   signal_health.numSatsRtkA,  signal_health.highestCn0A,
                        signal_health.numSatsPvtB,   signal_health.numSatsRtkB,  signal_health.highestCn0B,
                        signal_health.numComSatsPvt, signal_health.numComSatsRtk};
            gnss_compass_signal_health_publisher_->publish(msg);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to read GnssCompassSignalHealthStatus register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        }

        VN::Registers::GNSSCompass::GnssCompassStartupStatus startup_status;
        if (const auto error = vs_.readRegister(&startup_status); error == VN::Error::None) {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {static_cast<float>(startup_status.percentComplete), startup_status.currentHeading};
            gnss_compass_startup_status_publisher_->publish(msg);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to read GnssCompassStartupStatus register");
            RCLCPP_WARN(get_logger(), "Error: %s", VN::errorCodeToString(error).data());
        }
    }

    VN::Sensor vs_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr ins_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr gnss_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr gnss2_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_uncertainty_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gnss_compass_signal_health_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gnss_compass_startup_status_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string port_;
    VN::Sensor::BaudRate baud_rate_;
    uint16_t rate_divisor_;
    double status_register_poll_period_s_;
    std::string imu_frame_id_;
    std::string ins_frame_id_;
    std::string gnss_a_frame_id_;
    std::string gnss_b_frame_id_;
    std::array<double, 9> linear_accel_covariance_;
    std::array<double, 9> angular_vel_covariance_;
    std::optional<GeographicLib::LocalCartesian> datum_projection_;
    std::string map_frame_id_;
    bool require_attitude_;
    Rotation imu_to_ins_;

    rclcpp::CallbackGroup::SharedPtr measurement_group_;
    rclcpp::CallbackGroup::SharedPtr status_register_group_;
    rclcpp::TimerBase::SharedPtr measurement_timer_;
    rclcpp::TimerBase::SharedPtr status_register_timer_;
};
