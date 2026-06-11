// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

// Author: Nikola Banovic
// Contributor: Hajar Bartakh

#ifndef beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_
#define beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ads/AdsLib.h"
#include "ads/AdsDevice.h"
namespace beckhoff_ads_hardware_interface
{

  enum class PLCType
  {
    // Using enum instead of raw strings for faster iterations in read/write
    UNKNOWN,
    BOOL,
    LREAL,
    REAL,
    UDINT,
    DINT,
    INT,
    UINT,
    SINT,
    USINT,
    BYTE,
    STRING,
  };

  // Describes each PLC item (array or single variable) for polling via sum commands.
  struct ADSDataLayout
  {
    // Configured from yaml
    std::string plc_name_symbolic; // e.g., "MAIN.Joint_Pos_State". Used to get the handle.
    PLCType plc_type;
    uint32_t ads_handle = 0; // Raw PLC handle value kept alive by ads_symbol_handles_.

    size_t num_elements;          // 6 for LREAL[6], 1 for single LREAL/BOOL etc.
    size_t plc_element_byte_size; // byte size of ONE element on PLC (e.g., 8 for LREAL, 1 for BOOL).

    // If we have an identically named command and state interface, in case there are no new commands to be sent to the robot, we want to use the value read in the state interface for the next request.
    // for mapping the ros2 state interfaces names to the corresponding command interfaces names <command_interface_name, state_interface_name>
    std::map<std::string, std::string> state_command_interfaces_map_;

    // for unpacking sum read response  [Err1_ULONG,...,ErrN_ULONG | Data1_bytes,...,DataN_bytes]
    size_t offset_in_read_response_error; // Byte offset where this item's ULONG error code starts.
    size_t offset_in_read_response_data;  // Byte offset where this item's data starts.

    // for packing sum write response [ADS_ITEM_REQ_HEADER_1,...,ADS_ITEM_REQ_HEADER_N | Data1_bytes,...,DataN_bytes]
    size_t offset_in_write_request_data; // Byte offset where this item's data starts.

    // for decoding ADS notifications
    size_t notification_decode_offset = 0;
    size_t notification_decode_count = 0;

    // For interfaces targeting the same PLC symbol, store all their names with their corresponding index inside a map. This will be useful when calling thr ROS2 set_state and set_command functions.
    std::map<size_t, std::string> ros2_interfaces_;
  };

  // Packed header for sum read/write request item headers
  typedef struct
  {
    uint32_t indexGroup;   // ADSIGRP_SYM_VALBYHND
    uint32_t indexOffset;  // The ADS Handle
    uint32_t NumBytesData; // total num of bytes in this data section
  } ADS_ITEM_REQ_HEADER;

  struct ReadInstruction
  {
    size_t read_buffer_offset_error_code;
    size_t read_buffer_offset_data;
    PLCType plc_type;
    std::string state_interface_name;
    size_t notification_value_index = 0;
  };

  struct WriteInstruction
  {
    size_t write_buffer_offset_data;
    PLCType plc_type;
    std::string command_interface_name;
    std::string fallback_state_interface_name; // The state interface name corresponding to the current command interface name
  };

  struct NotificationDecodeInstruction
  {
    size_t notification_data_offset;
    PLCType plc_type;
    size_t notification_value_index;
  };

  class BeckhoffADSHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    ~BeckhoffADSHardwareInterface() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;

    std::vector<hardware_interface::InterfaceDescription>
    export_unlisted_state_interface_descriptions() override;

    std::vector<hardware_interface::InterfaceDescription>
    export_unlisted_command_interface_descriptions() override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    rclcpp::Logger getLogger() { return rclcpp::get_logger("BeckhoffADSHardwareInterface"); }
    rclcpp::Clock logging_throttle_clock_{RCL_STEADY_TIME};

    // ========= PLC ==============================

    // PLC Type and Size Helpers
    PLCType strToPlcType(const std::string &type_str);
    size_t plcTypeByteSize(PLCType type_enum);

    // ADS Communication objects
    std::unique_ptr<AdsDevice> ads_device_; // Manages the route/connection to the PLC
    std::vector<AdsHandle> ads_symbol_handles_; // Owns PLC symbol handles while sum read/write buffers use raw handle values.
    bool configure_ads_device();
    bool refresh_ads_handles();

    // Metadata (populated in on interface export)
    // Describes each variable on the PLC
    std::vector<ADSDataLayout> ads_item_layouts_read_;
    std::vector<ADSDataLayout> ads_item_layouts_write_;
    void ads_read_layout_configure();
    void ads_write_layout_configure();
    bool build_sum_read_buffers();
    bool build_sum_write_buffers();
    bool build_notification_read_buffers();
    bool configure_ads_notifications();
    void clear_ads_notifications();
    void handle_ads_notification(size_t layout_index, const uint8_t *data, size_t data_size);
    static void ads_notification_callback(
        const AmsAddr *addr,
        const AdsNotificationHeader *notification,
        uint32_t user_handle);

    // ADS Sum Command Buffers
    // SENT: List of ADS_ITEM_REQ_HEADER structs
    // RECEIVED: List of [Err1_ULONG,...,ErrN_ULONG | Data1_bytes,...,DataN_bytes]
    std::vector<uint8_t> ads_buffer_sum_read_request_;
    std::vector<uint8_t> ads_buffer_sum_read_response_;
    size_t num_items_read_ = 0;

    // SENT: List of [ADS_ITEM_REQ_HEADER_1,...,ADS_ITEM_REQ_HEADER_N | Data1_bytes,...,DataN_bytes]
    // RECEIVED: List of ErrCode_ULONGs
    std::vector<uint8_t> ads_buffer_sum_write_request_;
    std::vector<uint8_t> ads_buffer_sum_write_response_;
    size_t num_items_write_ = 0;

    bool use_ads_notifications_for_read_ = true;
    bool use_ads_async_write_ = true;
    uint32_t ads_notification_cycle_time_100ns_ = 10000; // 1 ms
    uint32_t ads_notification_max_delay_100ns_ = 0;
    std::vector<AdsHandle> ads_notification_handles_;
    std::vector<uint32_t> ads_notification_user_handles_;
    std::vector<NotificationDecodeInstruction> ads_notification_decode_instructions_;
    std::unique_ptr<std::atomic<double>[]> ads_notification_state_values_;
    size_t ads_notification_state_value_count_ = 0;
    std::atomic<bool> ads_notification_sample_received_{false};

    std::vector<ReadInstruction> ads_read_instructions_;
    std::vector<WriteInstruction> ads_write_instructions_;
  };

} // namespace beckhoff_ads_hardware_interface

#endif // beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_
