// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.
//
// Author: Nikola Banovic
// Contributor: Hajar Bartakh

#include <limits>
#include <vector>
#include <cstdint>
#include <algorithm> // std::transform
#include <atomic>
#include <cctype>
#include <cstring>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <utility>

#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace beckhoff_ads_hardware_interface
{
    namespace
    {
        constexpr const char *ARRAY_EXPAND_PARAM = "array_expand";
        constexpr const char *ARRAY_ROWS_PARAM = "array_rows";
        constexpr const char *ARRAY_COLS_PARAM = "array_cols";
        constexpr const char *ARRAY_NAME_PREFIX_PARAM = "array_name_prefix";
        constexpr const char *N_ELEMENTS_PARAM = "n_elements";
        constexpr const char *INDEX_PARAM = "index";
        constexpr const char *ADS_READ_MODE_PARAM = "ads_read_mode";
        constexpr const char *ADS_WRITE_MODE_PARAM = "ads_write_mode";
        constexpr const char *ADS_NOTIFICATION_CYCLE_TIME_US_PARAM = "ads_notification_cycle_time_us";
        constexpr const char *ADS_NOTIFICATION_MAX_DELAY_US_PARAM = "ads_notification_max_delay_us";

        struct NotificationTarget
        {
            BeckhoffADSHardwareInterface *owner = nullptr;
            size_t layout_index = 0;
        };

        std::mutex g_notification_targets_mutex;
        std::map<uint32_t, NotificationTarget> g_notification_targets;
        std::atomic<uint32_t> g_next_notification_user_handle{1};

        bool parse_true_string(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            return value == "true" || value == "1" || value == "yes" || value == "on";
        }

        template <typename ParamsT>
        bool parse_uint32_param(
            const ParamsT &params,
            const char *param_name,
            uint32_t default_value,
            uint32_t &parsed_value,
            const rclcpp::Logger &logger)
        {
            const auto param_it = params.find(param_name);
            if (param_it == params.end())
            {
                parsed_value = default_value;
                return true;
            }

            try
            {
                const auto value = std::stoul(param_it->second);
                if (value > std::numeric_limits<uint32_t>::max())
                {
                    RCLCPP_ERROR(
                        logger,
                        "Hardware parameter '%s'='%s' is outside uint32 range.",
                        param_name,
                        param_it->second.c_str());
                    return false;
                }
                parsed_value = static_cast<uint32_t>(value);
                return true;
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(
                    logger,
                    "Invalid hardware parameter '%s'='%s': %s.",
                    param_name,
                    param_it->second.c_str(),
                    ex.what());
                return false;
            }
        }

        bool notification_time_us_to_100ns(
            const uint32_t time_us,
            uint32_t &time_100ns,
            const char *param_name,
            const rclcpp::Logger &logger)
        {
            if (time_us > std::numeric_limits<uint32_t>::max() / 10)
            {
                RCLCPP_ERROR(
                    logger,
                    "Hardware parameter '%s'=%u us is too large for ADS notification time units.",
                    param_name,
                    time_us);
                return false;
            }

            time_100ns = time_us * 10;
            return true;
        }

        uint32_t register_notification_target(
            BeckhoffADSHardwareInterface *owner,
            const size_t layout_index)
        {
            for (;;)
            {
                const uint32_t user_handle = g_next_notification_user_handle.fetch_add(1);
                if (user_handle == 0)
                {
                    continue;
                }

                std::lock_guard<std::mutex> lock(g_notification_targets_mutex);
                const auto inserted = g_notification_targets.emplace(
                    user_handle,
                    NotificationTarget{owner, layout_index});
                if (inserted.second)
                {
                    return user_handle;
                }
            }
        }

        bool lookup_notification_target(
            const uint32_t user_handle,
            NotificationTarget &target)
        {
            std::lock_guard<std::mutex> lock(g_notification_targets_mutex);
            const auto target_it = g_notification_targets.find(user_handle);
            if (target_it == g_notification_targets.end())
            {
                return false;
            }
            target = target_it->second;
            return true;
        }

        void unregister_notification_target(const uint32_t user_handle)
        {
            std::lock_guard<std::mutex> lock(g_notification_targets_mutex);
            g_notification_targets.erase(user_handle);
        }

        long queue_ads_read_write_no_response(
            const AdsDevice &ads_device,
            const uint32_t index_group,
            const uint32_t index_offset,
            const size_t read_length,
            const size_t write_length,
            const void *write_data)
        {
            if (read_length > std::numeric_limits<uint32_t>::max() ||
                write_length > std::numeric_limits<uint32_t>::max())
            {
                return ADSERR_DEVICE_INVALIDSIZE;
            }

            return AdsAsyncReadWriteReqEx(
                ads_device.GetLocalPort(),
                &ads_device.m_Addr,
                index_group,
                index_offset,
                static_cast<uint32_t>(read_length),
                static_cast<uint32_t>(write_length),
                write_data);
        }

        bool has_true_param(
            const hardware_interface::InterfaceInfo &interface_info,
            const std::string &param_name)
        {
            const auto param_it = interface_info.parameters.find(param_name);
            if (param_it == interface_info.parameters.end())
            {
                return false;
            }

            return parse_true_string(param_it->second);
        }

        bool parse_size_param(
            const hardware_interface::InterfaceInfo &interface_info,
            const std::string &param_name,
            size_t &parsed_value,
            const rclcpp::Logger &logger)
        {
            const auto param_it = interface_info.parameters.find(param_name);
            if (param_it == interface_info.parameters.end())
            {
                RCLCPP_ERROR(
                    logger,
                    "Array-expanded interface '%s' is missing required parameter '%s'.",
                    interface_info.name.c_str(),
                    param_name.c_str());
                return false;
            }

            try
            {
                parsed_value = std::stoul(param_it->second);
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(
                    logger,
                    "Array-expanded interface '%s' has invalid parameter '%s'='%s': %s.",
                    interface_info.name.c_str(),
                    param_name.c_str(),
                    param_it->second.c_str(),
                    ex.what());
                return false;
            }

            return true;
        }

        bool parse_optional_size_param(
            const hardware_interface::InterfaceInfo &interface_info,
            const std::string &param_name,
            size_t &parsed_value,
            const rclcpp::Logger &logger)
        {
            if (interface_info.parameters.find(param_name) == interface_info.parameters.end())
            {
                return true;
            }

            return parse_size_param(interface_info, param_name, parsed_value, logger);
        }

        void remove_array_expansion_params(hardware_interface::InterfaceInfo &interface_info)
        {
            interface_info.parameters.erase(ARRAY_EXPAND_PARAM);
            interface_info.parameters.erase(ARRAY_ROWS_PARAM);
            interface_info.parameters.erase(ARRAY_COLS_PARAM);
            interface_info.parameters.erase(ARRAY_NAME_PREFIX_PARAM);
        }

        bool expand_interface_arrays(
            std::vector<hardware_interface::InterfaceInfo> &interfaces,
            const std::string &component_name,
            const std::string &interface_kind,
            const rclcpp::Logger &logger)
        {
            std::vector<hardware_interface::InterfaceInfo> expanded_interfaces;
            expanded_interfaces.reserve(interfaces.size());

            for (const auto &interface_info : interfaces)
            {
                if (!has_true_param(interface_info, ARRAY_EXPAND_PARAM))
                {
                    expanded_interfaces.push_back(interface_info);
                    continue;
                }

                size_t plc_num_elements = 0;
                size_t base_plc_index = 0;
                if (!parse_size_param(interface_info, N_ELEMENTS_PARAM, plc_num_elements, logger) ||
                    !parse_optional_size_param(interface_info, INDEX_PARAM, base_plc_index, logger))
                {
                    return false;
                }

                if (plc_num_elements == 0)
                {
                    RCLCPP_ERROR(
                        logger,
                        "Array-expanded %s interface '%s/%s' has n_elements=0.",
                        interface_kind.c_str(),
                        component_name.c_str(),
                        interface_info.name.c_str());
                    return false;
                }

                size_t array_rows = 0;
                size_t array_cols = 0;
                const bool has_rows =
                    interface_info.parameters.find(ARRAY_ROWS_PARAM) != interface_info.parameters.end();
                const bool has_cols =
                    interface_info.parameters.find(ARRAY_COLS_PARAM) != interface_info.parameters.end();
                if (has_rows != has_cols)
                {
                    RCLCPP_ERROR(
                        logger,
                        "Array-expanded %s interface '%s/%s' must provide both '%s' and '%s', or neither.",
                        interface_kind.c_str(),
                        component_name.c_str(),
                        interface_info.name.c_str(),
                        ARRAY_ROWS_PARAM,
                        ARRAY_COLS_PARAM);
                    return false;
                }

                if (has_rows)
                {
                    if (!parse_size_param(interface_info, ARRAY_ROWS_PARAM, array_rows, logger) ||
                        !parse_size_param(interface_info, ARRAY_COLS_PARAM, array_cols, logger))
                    {
                        return false;
                    }
                    if (array_rows == 0 || array_cols == 0)
                    {
                        RCLCPP_ERROR(
                            logger,
                            "Array-expanded %s interface '%s/%s' has invalid matrix dimensions %zux%zu.",
                            interface_kind.c_str(),
                            component_name.c_str(),
                            interface_info.name.c_str(),
                            array_rows,
                            array_cols);
                        return false;
                    }
                }

                const size_t generated_count = has_rows ? array_rows * array_cols : plc_num_elements;
                if (base_plc_index + generated_count > plc_num_elements)
                {
                    RCLCPP_ERROR(
                        logger,
                        "Array-expanded %s interface '%s/%s' maps %zu elements starting at PLC index %zu, "
                        "but n_elements is only %zu.",
                        interface_kind.c_str(),
                        component_name.c_str(),
                        interface_info.name.c_str(),
                        generated_count,
                        base_plc_index,
                        plc_num_elements);
                    return false;
                }

                std::string generated_name_prefix = interface_info.name;
                const auto prefix_it = interface_info.parameters.find(ARRAY_NAME_PREFIX_PARAM);
                if (prefix_it != interface_info.parameters.end() && !prefix_it->second.empty())
                {
                    generated_name_prefix = prefix_it->second;
                }

                expanded_interfaces.reserve(expanded_interfaces.size() + generated_count);
                if (has_rows)
                {
                    for (size_t row_index = 0; row_index < array_rows; ++row_index)
                    {
                        for (size_t col_index = 0; col_index < array_cols; ++col_index)
                        {
                            hardware_interface::InterfaceInfo generated_interface = interface_info;
                            generated_interface.name =
                                generated_name_prefix + "_" + std::to_string(row_index) + "_" +
                                std::to_string(col_index);
                            const size_t plc_index = base_plc_index + row_index * array_cols + col_index;
                            generated_interface.parameters[INDEX_PARAM] = std::to_string(plc_index);
                            remove_array_expansion_params(generated_interface);
                            expanded_interfaces.push_back(generated_interface);
                        }
                    }
                }
                else
                {
                    for (size_t element_index = 0; element_index < generated_count; ++element_index)
                    {
                        hardware_interface::InterfaceInfo generated_interface = interface_info;
                        generated_interface.name =
                            generated_name_prefix + "_" + std::to_string(element_index);
                        generated_interface.parameters[INDEX_PARAM] =
                            std::to_string(base_plc_index + element_index);
                        remove_array_expansion_params(generated_interface);
                        expanded_interfaces.push_back(generated_interface);
                    }
                }

                RCLCPP_INFO(
                    logger,
                    "Expanded %s interface '%s/%s' into %zu PLC array %s interfaces.",
                    interface_kind.c_str(),
                    component_name.c_str(),
                    interface_info.name.c_str(),
                    generated_count,
                    interface_kind.c_str());
            }

            interfaces = std::move(expanded_interfaces);
            return true;
        }

    } // namespace

    BeckhoffADSHardwareInterface::~BeckhoffADSHardwareInterface()
    {
        clear_ads_notifications();
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::InterfaceDescription>
    BeckhoffADSHardwareInterface::export_unlisted_state_interface_descriptions()
    {
        std::vector<hardware_interface::InterfaceDescription> interface_descriptions;

        const auto append_expanded_interfaces =
            [&](const std::vector<hardware_interface::ComponentInfo> &components)
        {
            for (const auto &component_info : components)
            {
                std::unordered_set<std::string> listed_interface_names;
                for (const auto &state_interface_info : component_info.state_interfaces)
                {
                    listed_interface_names.insert(state_interface_info.name);
                }

                auto expanded_state_interfaces = component_info.state_interfaces;
                if (!expand_interface_arrays(
                        expanded_state_interfaces, component_info.name, "state", getLogger()))
                {
                    interface_descriptions.clear();
                    return false;
                }

                for (const auto &expanded_interface_info : expanded_state_interfaces)
                {
                    if (listed_interface_names.find(expanded_interface_info.name) !=
                        listed_interface_names.end())
                    {
                        continue;
                    }

                    interface_descriptions.emplace_back(component_info.name, expanded_interface_info);
                }
            }

            return true;
        };

        if (!append_expanded_interfaces(info_.joints) ||
            !append_expanded_interfaces(info_.gpios) ||
            !append_expanded_interfaces(info_.sensors))
        {
            return {};
        }

        return interface_descriptions;
    }

    std::vector<hardware_interface::InterfaceDescription>
    BeckhoffADSHardwareInterface::export_unlisted_command_interface_descriptions()
    {
        std::vector<hardware_interface::InterfaceDescription> interface_descriptions;

        const auto append_expanded_interfaces =
            [&](const std::vector<hardware_interface::ComponentInfo> &components)
        {
            for (const auto &component_info : components)
            {
                std::unordered_set<std::string> listed_interface_names;
                for (const auto &command_interface_info : component_info.command_interfaces)
                {
                    listed_interface_names.insert(command_interface_info.name);
                }

                auto expanded_command_interfaces = component_info.command_interfaces;
                if (!expand_interface_arrays(
                        expanded_command_interfaces, component_info.name, "command", getLogger()))
                {
                    interface_descriptions.clear();
                    return false;
                }

                for (const auto &expanded_interface_info : expanded_command_interfaces)
                {
                    if (listed_interface_names.find(expanded_interface_info.name) !=
                        listed_interface_names.end())
                    {
                        continue;
                    }

                    interface_descriptions.emplace_back(component_info.name, expanded_interface_info);
                }
            }

            return true;
        };

        if (!append_expanded_interfaces(info_.joints) || !append_expanded_interfaces(info_.gpios))
        {
            return {};
        }

        return interface_descriptions;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        const auto &params = info_.hardware_parameters;

        const auto read_mode_it = params.find(ADS_READ_MODE_PARAM);
        if (read_mode_it != params.end())
        {
            std::string read_mode = read_mode_it->second;
            std::transform(read_mode.begin(), read_mode.end(), read_mode.begin(), ::tolower);
            if (read_mode == "notification" || read_mode == "notifications" || read_mode == "async")
            {
                use_ads_notifications_for_read_ = true;
            }
            else if (read_mode == "polling" || read_mode == "sync" || read_mode == "synchronous")
            {
                use_ads_notifications_for_read_ = false;
            }
            else
            {
                RCLCPP_FATAL(
                    getLogger(),
                    "Unsupported '%s' value '%s'. Expected 'notification' or 'polling'.",
                    ADS_READ_MODE_PARAM,
                    read_mode_it->second.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        const auto write_mode_it = params.find(ADS_WRITE_MODE_PARAM);
        if (write_mode_it != params.end())
        {
            std::string write_mode = write_mode_it->second;
            std::transform(write_mode.begin(), write_mode.end(), write_mode.begin(), ::tolower);
            if (write_mode == "async" || write_mode == "no_response" || write_mode == "fire_and_forget")
            {
                use_ads_async_write_ = true;
            }
            else if (write_mode == "sync" || write_mode == "synchronous")
            {
                use_ads_async_write_ = false;
            }
            else
            {
                RCLCPP_FATAL(
                    getLogger(),
                    "Unsupported '%s' value '%s'. Expected 'async' or 'sync'.",
                    ADS_WRITE_MODE_PARAM,
                    write_mode_it->second.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        uint32_t notification_cycle_time_us = 1000;
        uint32_t notification_max_delay_us = 0;
        if (!parse_uint32_param(
                params,
                ADS_NOTIFICATION_CYCLE_TIME_US_PARAM,
                notification_cycle_time_us,
                notification_cycle_time_us,
                getLogger()) ||
            !parse_uint32_param(
                params,
                ADS_NOTIFICATION_MAX_DELAY_US_PARAM,
                notification_max_delay_us,
                notification_max_delay_us,
                getLogger()) ||
            !notification_time_us_to_100ns(
                notification_cycle_time_us,
                ads_notification_cycle_time_100ns_,
                ADS_NOTIFICATION_CYCLE_TIME_US_PARAM,
                getLogger()) ||
            !notification_time_us_to_100ns(
                notification_max_delay_us,
                ads_notification_max_delay_100ns_,
                ADS_NOTIFICATION_MAX_DELAY_US_PARAM,
                getLogger()))
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Configure ADS Client Device
        if (!configure_ads_device())
        {
            RCLCPP_FATAL(getLogger(), "Failed to configure ADS device from URDF parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Fill the ADSDataLayout vectors for read and write operations
        ads_read_layout_configure();
        ads_write_layout_configure();

        // Request handles for all symbolic PLC variable names
        if (!refresh_ads_handles())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to acquire ADS handles for all configured PLC variables.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Link command interfaces to their corresponding state interfaces
        RCLCPP_INFO(getLogger(), "Linking command interfaces to state interfaces...");
        for (auto &command_layout : ads_item_layouts_write_)
        {
            for (const auto &state_layout : ads_item_layouts_read_)
            {
                if (command_layout.plc_name_symbolic == state_layout.plc_name_symbolic)
                {
                    for (size_t k = 0; k < command_layout.num_elements; ++k)
                    {
                        const auto command_interface_it = command_layout.ros2_interfaces_.find(k);
                        const auto state_interface_it = state_layout.ros2_interfaces_.find(k);
                        if (command_interface_it == command_layout.ros2_interfaces_.end() ||
                            state_interface_it == state_layout.ros2_interfaces_.end())
                        {
                            continue;
                        }

                        // The pair is made of (command_interface_name, corresponding_state_interface_name)
                        auto pair = std::make_pair(command_interface_it->second, state_interface_it->second);
                        command_layout.state_command_interfaces_map_.emplace(pair);
                    }
                }
            }
        }

        // Pre-pack what we can for SUM write commands and configure the selected read mode.
        if (use_ads_notifications_for_read_)
        {
            if (!build_notification_read_buffers() || !configure_ads_notifications())
            {
                RCLCPP_FATAL(getLogger(), "\tFailed to configure ADS notification read path.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(
                getLogger(),
                "ADS notification read mode enabled (%u us cycle, %u us max delay).",
                ads_notification_cycle_time_100ns_ / 10,
                ads_notification_max_delay_100ns_ / 10);
        }
        else if (!build_sum_read_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum read buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!build_sum_write_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum write buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            getLogger(),
            "ADS write mode: %s.",
            use_ads_async_write_ ? "async/no-response" : "synchronous");

        return CallbackReturn::SUCCESS;
    }

    bool BeckhoffADSHardwareInterface::refresh_ads_handles()
    {
        if (!ads_device_)
        {
            RCLCPP_ERROR(getLogger(), "Cannot refresh ADS handles: ADS device is not configured.");
            return false;
        }

        RCLCPP_INFO(getLogger(), "Fetching ADS handles for configured PLC variables...");

        std::vector<std::pair<ADSDataLayout *, AdsHandle>> acquired_handles;
        acquired_handles.reserve(ads_item_layouts_read_.size() + ads_item_layouts_write_.size());

        auto acquire_layout_handles =
            [&](std::vector<ADSDataLayout> &layouts, const char *operation)
        {
            bool success = true;
            for (auto &layout : layouts)
            {
                try
                {
                    auto ads_handle = ads_device_->GetHandle(layout.plc_name_symbolic);
                    acquired_handles.emplace_back(&layout, std::move(ads_handle));
                }
                catch (const std::exception &ex)
                {
                    RCLCPP_ERROR(
                        getLogger(),
                        "\tADS Exception getting handle for %s symbol '%s': %s.",
                        operation,
                        layout.plc_name_symbolic.c_str(),
                        ex.what());
                    success = false;
                }
            }
            return success;
        };

        const bool read_handles_ok = acquire_layout_handles(ads_item_layouts_read_, "read");
        const bool write_handles_ok = acquire_layout_handles(ads_item_layouts_write_, "write");
        if (!read_handles_ok || !write_handles_ok)
        {
            return false;
        }

        std::vector<AdsHandle> new_symbol_handles;
        new_symbol_handles.reserve(acquired_handles.size());
        for (auto &[layout, ads_handle] : acquired_handles)
        {
            layout->ads_handle = *ads_handle;
            new_symbol_handles.push_back(std::move(ads_handle));
        }

        ads_symbol_handles_ = std::move(new_symbol_handles);
        RCLCPP_INFO(getLogger(), "\tHandles acquired");
        return true;
    }

    bool BeckhoffADSHardwareInterface::build_sum_read_buffers()
    {
        ads_read_instructions_.clear();
        num_items_read_ = ads_item_layouts_read_.size();
        if (num_items_read_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum READ.");
            return true;
        }
        RCLCPP_INFO(getLogger(), "Building ADS sum READ buffer...");

        size_t total_error_block_size = num_items_read_ * sizeof(uint32_t);
        size_t total_data_block_size = 0;
        for (const auto &layout : ads_item_layouts_read_)
        {
            total_data_block_size += layout.plc_element_byte_size * layout.num_elements;
        }

        ads_buffer_sum_read_response_.resize(total_error_block_size + total_data_block_size);
        ads_buffer_sum_read_request_.clear();

        size_t current_data_offset = 0;
        size_t current_error_offset = 0;

        for (auto &layout : ads_item_layouts_read_)
        {
            layout.offset_in_read_response_data = total_error_block_size + current_data_offset;
            layout.offset_in_read_response_error = current_error_offset;

            ADS_ITEM_REQ_HEADER header;
            header.indexGroup = ADSIGRP_SYM_VALBYHND;
            header.indexOffset = layout.ads_handle;
            header.NumBytesData = layout.plc_element_byte_size * layout.num_elements;
            const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&header);
            ads_buffer_sum_read_request_.insert(ads_buffer_sum_read_request_.end(), ptr, ptr + sizeof(ADS_ITEM_REQ_HEADER));

            // For interfaces targeting the same PLC symbol
            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                // Fill the read instruction vector
                ReadInstruction read_instruction;
                read_instruction.read_buffer_offset_error_code = layout.offset_in_read_response_error;
                read_instruction.read_buffer_offset_data = layout.offset_in_read_response_data + index * layout.plc_element_byte_size;
                read_instruction.plc_type = layout.plc_type;
                read_instruction.state_interface_name = interface_name;

                // The state interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                ads_read_instructions_.push_back(read_instruction);
            }

            current_data_offset += header.NumBytesData;
            current_error_offset += sizeof(uint32_t);
        }
        RCLCPP_INFO(getLogger(), "ADS Sum READ configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_read_, ads_buffer_sum_read_request_.size(), ads_buffer_sum_read_response_.size());
        return true;
    }

    bool BeckhoffADSHardwareInterface::build_sum_write_buffers()
    {
        ads_write_instructions_.clear();
        RCLCPP_INFO(getLogger(), "Building ADS sum WRITE buffer...");
        num_items_write_ = ads_item_layouts_write_.size();
        if (num_items_write_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum WRITE.");
            return true;
        }

        size_t total_data_size = 0;
        size_t total_header_size = num_items_write_ * sizeof(ADS_ITEM_REQ_HEADER);
        for (const auto &layout : ads_item_layouts_write_)
        {
            total_data_size += layout.plc_element_byte_size * layout.num_elements;
        }

        ads_buffer_sum_write_request_.resize(total_header_size + total_data_size);
        ads_buffer_sum_write_response_.resize(num_items_write_ * sizeof(uint32_t));

        auto *header_block_ptr = reinterpret_cast<ADS_ITEM_REQ_HEADER *>(ads_buffer_sum_write_request_.data());
        size_t current_data_offset = 0;
        size_t i = 0; // Index for the header block pointer

        for (auto &layout : ads_item_layouts_write_)
        {
            header_block_ptr[i].indexGroup = ADSIGRP_SYM_VALBYHND;
            header_block_ptr[i].indexOffset = layout.ads_handle;
            header_block_ptr[i].NumBytesData = layout.plc_element_byte_size * layout.num_elements;

            layout.offset_in_write_request_data = total_header_size + current_data_offset;

            // For interfaces targeting the same PLC symbol
            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                // Fill the write instruction vector
                WriteInstruction write_instruction;
                write_instruction.write_buffer_offset_data = layout.offset_in_write_request_data + index * layout.plc_element_byte_size;
                write_instruction.plc_type = layout.plc_type;
                write_instruction.command_interface_name = interface_name;
                write_instruction.fallback_state_interface_name = "";

                // There exists a state interface for the same PLC symbol
                if (!layout.state_command_interfaces_map_.empty())
                {
                    const auto fallback_it = layout.state_command_interfaces_map_.find(interface_name);
                    if (fallback_it != layout.state_command_interfaces_map_.end())
                    {
                        write_instruction.fallback_state_interface_name = fallback_it->second;
                    }
                }

                // The command interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                ads_write_instructions_.push_back(write_instruction);
            }

            current_data_offset += header_block_ptr[i].NumBytesData;
            i++;
        }

        RCLCPP_INFO(getLogger(), "ADS Sum WRITE configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_write_, ads_buffer_sum_write_request_.size(), ads_buffer_sum_write_response_.size());
        return true;
    }

    bool BeckhoffADSHardwareInterface::build_notification_read_buffers()
    {
        ads_read_instructions_.clear();
        ads_notification_decode_instructions_.clear();
        ads_notification_state_values_.reset();
        ads_notification_state_value_count_ = 0;
        ads_notification_sample_received_.store(false, std::memory_order_release);

        num_items_read_ = ads_item_layouts_read_.size();
        if (num_items_read_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS notifications.");
            return true;
        }

        RCLCPP_INFO(getLogger(), "Building ADS notification read cache...");

        for (size_t layout_index = 0; layout_index < ads_item_layouts_read_.size(); ++layout_index)
        {
            auto &layout = ads_item_layouts_read_[layout_index];
            layout.offset_in_read_response_error = 0;
            layout.offset_in_read_response_data = 0;
            layout.notification_decode_offset = ads_notification_decode_instructions_.size();

            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                const size_t value_index = ads_notification_state_value_count_++;
                const size_t plc_data_offset = index * layout.plc_element_byte_size;

                ReadInstruction read_instruction;
                read_instruction.read_buffer_offset_error_code = 0;
                read_instruction.read_buffer_offset_data = plc_data_offset;
                read_instruction.plc_type = layout.plc_type;
                read_instruction.state_interface_name = interface_name;
                read_instruction.notification_value_index = value_index;
                ads_read_instructions_.push_back(read_instruction);

                ads_notification_decode_instructions_.push_back(
                    NotificationDecodeInstruction{
                        plc_data_offset,
                        layout.plc_type,
                        value_index});
            }

            layout.notification_decode_count =
                ads_notification_decode_instructions_.size() - layout.notification_decode_offset;
        }

        ads_notification_state_values_ =
            std::make_unique<std::atomic<double>[]>(ads_notification_state_value_count_);
        for (size_t i = 0; i < ads_notification_state_value_count_; ++i)
        {
            ads_notification_state_values_[i].store(
                std::numeric_limits<double>::quiet_NaN(),
                std::memory_order_relaxed);
        }

        RCLCPP_INFO(
            getLogger(),
            "ADS notifications configured for %zu PLC symbols and %zu state values.",
            num_items_read_,
            ads_notification_state_value_count_);
        return true;
    }

    bool BeckhoffADSHardwareInterface::configure_ads_notifications()
    {
        if (!ads_device_)
        {
            RCLCPP_ERROR(getLogger(), "Cannot configure ADS notifications: ADS device is not configured.");
            return false;
        }

        clear_ads_notifications();
        if (ads_item_layouts_read_.empty())
        {
            return true;
        }

        ads_notification_handles_.reserve(ads_item_layouts_read_.size());
        ads_notification_user_handles_.reserve(ads_item_layouts_read_.size());

        for (size_t layout_index = 0; layout_index < ads_item_layouts_read_.size(); ++layout_index)
        {
            const auto &layout = ads_item_layouts_read_[layout_index];
            AdsNotificationAttrib notification_attributes{};
            notification_attributes.cbLength =
                static_cast<uint32_t>(layout.plc_element_byte_size * layout.num_elements);
            notification_attributes.nTransMode = ADSTRANS_SERVERCYCLE;
            notification_attributes.nMaxDelay = ads_notification_max_delay_100ns_;
            notification_attributes.nCycleTime = ads_notification_cycle_time_100ns_;

            const uint32_t user_handle = register_notification_target(this, layout_index);
            try
            {
                auto notification_handle = ads_device_->GetHandle(
                    ADSIGRP_SYM_VALBYHND,
                    layout.ads_handle,
                    notification_attributes,
                    &BeckhoffADSHardwareInterface::ads_notification_callback,
                    user_handle);
                ads_notification_user_handles_.push_back(user_handle);
                ads_notification_handles_.push_back(std::move(notification_handle));
            }
            catch (const std::exception &ex)
            {
                unregister_notification_target(user_handle);
                RCLCPP_ERROR(
                    getLogger(),
                    "Failed to register ADS notification for PLC symbol '%s': %s.",
                    layout.plc_name_symbolic.c_str(),
                    ex.what());
                clear_ads_notifications();
                return false;
            }
        }

        return true;
    }

    void BeckhoffADSHardwareInterface::clear_ads_notifications()
    {
        for (const uint32_t user_handle : ads_notification_user_handles_)
        {
            unregister_notification_target(user_handle);
        }
        ads_notification_user_handles_.clear();
        ads_notification_handles_.clear();
        ads_notification_sample_received_.store(false, std::memory_order_release);
    }

    void BeckhoffADSHardwareInterface::ads_notification_callback(
        const AmsAddr * /*addr*/,
        const AdsNotificationHeader *notification,
        const uint32_t user_handle)
    {
        if (!notification)
        {
            return;
        }

        NotificationTarget target;
        if (!lookup_notification_target(user_handle, target) || !target.owner)
        {
            return;
        }

        const auto *data = reinterpret_cast<const uint8_t *>(notification + 1);
        target.owner->handle_ads_notification(target.layout_index, data, notification->cbSampleSize);
    }

    void BeckhoffADSHardwareInterface::handle_ads_notification(
        const size_t layout_index,
        const uint8_t *data,
        const size_t data_size)
    {
        if (!data || layout_index >= ads_item_layouts_read_.size() || !ads_notification_state_values_)
        {
            return;
        }

        const auto &layout = ads_item_layouts_read_[layout_index];
        const size_t expected_size = layout.plc_element_byte_size * layout.num_elements;
        if (data_size < expected_size)
        {
            return;
        }

        const size_t decode_begin = layout.notification_decode_offset;
        const size_t decode_end = decode_begin + layout.notification_decode_count;
        for (size_t decode_index = decode_begin; decode_index < decode_end; ++decode_index)
        {
            const auto &decode_instruction = ads_notification_decode_instructions_[decode_index];
            const uint8_t *plc_data = data + decode_instruction.notification_data_offset;
            double decoded_value = std::numeric_limits<double>::quiet_NaN();

            switch (decode_instruction.plc_type)
            {
            case PLCType::LREAL:
            {
                double val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = val;
                break;
            }
            case PLCType::REAL:
            {
                float val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = static_cast<double>(val);
                break;
            }
            case PLCType::BOOL:
            {
                uint8_t val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = val != 0 ? 1.0 : 0.0;
                break;
            }
            case PLCType::SINT:
            {
                int8_t val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = static_cast<double>(val);
                break;
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                uint8_t val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = static_cast<double>(val);
                break;
            }
            case PLCType::INT:
            {
                int16_t val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = static_cast<double>(val);
                break;
            }
            case PLCType::UINT:
            {
                uint16_t val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = static_cast<double>(val);
                break;
            }
            case PLCType::DINT:
            {
                int32_t val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = static_cast<double>(val);
                break;
            }
            case PLCType::UDINT:
            {
                uint32_t val;
                memcpy(&val, plc_data, sizeof(val));
                decoded_value = static_cast<double>(val);
                break;
            }
            case PLCType::UNKNOWN:
            case PLCType::STRING:
            default:
                break;
            }

            ads_notification_state_values_[decode_instruction.notification_value_index].store(
                decoded_value,
                std::memory_order_relaxed);
        }

        ads_notification_sample_received_.store(true, std::memory_order_release);
    }

    void BeckhoffADSHardwareInterface::ads_read_layout_configure()
    {
        // Count all state interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_state_interfaces =
            gpio_state_interfaces_.size() + joint_state_interfaces_.size() +
            sensor_state_interfaces_.size() + unlisted_state_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_read_.clear();
        ads_item_layouts_read_.reserve(num_state_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_read_layout =
            [&](const auto &type_state_interfaces_)
        {
            for (const auto &[name, descr] : type_state_interfaces_)
            {
                if (has_true_param(descr.interface_info, ARRAY_EXPAND_PARAM))
                {
                    continue;
                }

                std::string plc_symbol;
                std::string plc_type_str;
                size_t num_elements = 1;
                size_t plc_index = 0;
                try
                {
                    plc_symbol = descr.interface_info.parameters.at("PLC_symbol");
                    plc_type_str = descr.interface_info.parameters.at("PLC_type");
                    if (descr.interface_info.parameters.count("n_elements"))
                    {
                        num_elements = std::stoul(descr.interface_info.parameters.at("n_elements"));
                    }
                    if (descr.interface_info.parameters.count("index"))
                    {
                        plc_index = std::stoul(descr.interface_info.parameters.at("index"));
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getLogger(), "Error parsing PLC parameters for state interface '%s': %s. Check URDF.",
                                 name.c_str(), e.what());
                    continue;
                }

                // If this is the first time we see this symbol, create the layout
                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping state variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                    }
                    else
                    {
                        layout.plc_element_byte_size = plcTypeByteSize(layout.plc_type);
                        ads_item_layouts_read_.push_back(layout);
                        processed_plc_symbols[plc_symbol] = true;
                    }
                }
                // The symbol already exists
                else
                {
                    // Find the ADS Data Layout object of the corresponding PLC symbol
                    auto it = std::find_if(ads_item_layouts_read_.begin(), ads_item_layouts_read_.end(),
                                           [&plc_symbol](ADSDataLayout layout)
                                           { return layout.plc_name_symbolic == plc_symbol; });

                    // Add the interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                }
            }
        };

        init_ads_read_layout(joint_state_interfaces_);
        init_ads_read_layout(gpio_state_interfaces_);
        init_ads_read_layout(sensor_state_interfaces_);
        init_ads_read_layout(unlisted_state_interfaces_);
    }

    void BeckhoffADSHardwareInterface::ads_write_layout_configure()
    {
        // Count all command interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_command_interfaces =
            joint_command_interfaces_.size() + gpio_command_interfaces_.size() +
            unlisted_command_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_write_.clear();
        ads_item_layouts_write_.reserve(num_command_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_write_layout =
            [&](const auto &type_command_interfaces_)
        {
            for (const auto &[name, descr] : type_command_interfaces_)
            {
                if (has_true_param(descr.interface_info, ARRAY_EXPAND_PARAM))
                {
                    continue;
                }

                [[maybe_unused]] double initial_value = std::numeric_limits<double>::quiet_NaN();

                if (descr.interface_info.parameters.count("initial_value"))
                {
                    try
                    {
                        initial_value = std::stod(descr.interface_info.parameters.at("initial_value"));
                    }
                    catch (const std::exception &ex)
                    { // Catch conversion errors
                        RCLCPP_WARN(
                            getLogger(),
                            "Invalid 'initial_value' ('%s') for command interface '%s'. Using NaN. Error: %s",
                            descr.interface_info.parameters.at("initial_value").c_str(),
                            name.c_str(),
                            ex.what());
                    }
                }

                std::string plc_symbol;
                std::string plc_type_str;
                size_t num_elements = 1;
                size_t plc_index = 0;
                try
                {
                    plc_symbol = descr.interface_info.parameters.at("PLC_symbol");
                    plc_type_str = descr.interface_info.parameters.at("PLC_type");
                    if (descr.interface_info.parameters.count("n_elements"))
                    {
                        num_elements = std::stoul(descr.interface_info.parameters.at("n_elements"));
                    }
                    if (descr.interface_info.parameters.count("index"))
                    {
                        plc_index = std::stoul(descr.interface_info.parameters.at("index"));
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getLogger(), "Error parsing PLC parameters for command interface '%s': %s. Check URDF.",
                                 name.c_str(), e.what());
                    continue;
                }

                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping command variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                    }
                    else
                    {
                        layout.plc_element_byte_size = plcTypeByteSize(layout.plc_type);
                        ads_item_layouts_write_.push_back(layout);
                        processed_plc_symbols[plc_symbol] = true;
                    }
                }
                // The symbol already exists
                else
                {
                    // Look for the ADS Data Layout of the corresponding PLC symbol
                    auto it = std::find_if(ads_item_layouts_write_.begin(), ads_item_layouts_write_.end(),
                                           [&plc_symbol](ADSDataLayout layout)
                                           { return layout.plc_name_symbolic == plc_symbol; });

                    // Add the command interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                }
            }
        };

        init_ads_write_layout(joint_command_interfaces_);
        init_ads_write_layout(gpio_command_interfaces_);
        init_ads_write_layout(unlisted_command_interfaces_);
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: send some safety commands to the PLC?
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (num_items_read_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        if (use_ads_notifications_for_read_)
        {
            if (!ads_notification_sample_received_.load(std::memory_order_acquire) ||
                !ads_notification_state_values_)
            {
                return hardware_interface::return_type::OK;
            }

            for (const auto &read_instruction : ads_read_instructions_)
            {
                set_state(
                    read_instruction.state_interface_name,
                    ads_notification_state_values_[read_instruction.notification_value_index].load(
                        std::memory_order_relaxed));
            }
            return hardware_interface::return_type::OK;
        }

        uint32_t bytes_read_from_plc = 0;
        auto &read_response_buffer = ads_buffer_sum_read_response_;

        const long ads_sum_read_error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_READ,
            num_items_read_,
            ads_buffer_sum_read_response_.size(),
            ads_buffer_sum_read_response_.data(),
            ads_buffer_sum_read_request_.size(),
            ads_buffer_sum_read_request_.data(),
            &bytes_read_from_plc);

        if (ads_sum_read_error != ADSERR_NOERR)
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), logging_throttle_clock_, 1000,
                                  "Overall ADS Sum Read Error: 0x%lX.", ads_sum_read_error);
            return hardware_interface::return_type::ERROR;
        }

        if (bytes_read_from_plc != read_response_buffer.size())
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), logging_throttle_clock_, 1000,
                                  "ADS Sum Read size mismatch. Expected %zu, Got %u.",
                                  read_response_buffer.size(), bytes_read_from_plc);
            return hardware_interface::return_type::ERROR;
        }

        bool any_item_read_failed = false;
        bool refresh_handles_required = false;
        for (const auto &read_instruction : ads_read_instructions_)
        {
            uint32_t item_error_code;
            memcpy(&item_error_code,
                   read_response_buffer.data() + read_instruction.read_buffer_offset_error_code,
                   sizeof(uint32_t));

            if (item_error_code != ADSERR_NOERR)
            {
                const auto layout_it = std::find_if(
                    ads_item_layouts_read_.cbegin(),
                    ads_item_layouts_read_.cend(),
                    [&read_instruction](const ADSDataLayout &layout)
                    {
                        return layout.offset_in_read_response_error ==
                               read_instruction.read_buffer_offset_error_code;
                    });
                const char *plc_symbol =
                    layout_it != ads_item_layouts_read_.cend() ? layout_it->plc_name_symbolic.c_str() : "<unknown>";
                const uint32_t ads_handle =
                    layout_it != ads_item_layouts_read_.cend() ? layout_it->ads_handle : 0;
                const size_t requested_bytes =
                    layout_it != ads_item_layouts_read_.cend()
                        ? layout_it->plc_element_byte_size * layout_it->num_elements
                        : 0;

                RCLCPP_WARN_THROTTLE(getLogger(), logging_throttle_clock_, 1000,
                                     "ADS Sum Read operation corresponding to the state interface '%s' failed: 0x%X "
                                     "(PLC symbol '%s', handle 0x%X, request %zu bytes).",
                                     read_instruction.state_interface_name.c_str(),
                                     item_error_code,
                                     plc_symbol,
                                     ads_handle,
                                     requested_bytes);

                if (item_error_code == ADSERR_DEVICE_SYMBOLVERSIONINVALID)
                {
                    refresh_handles_required = true;
                }
                any_item_read_failed = true;
                continue;
            }

            // Each state interface has its corresponding read_instruction
            const uint8_t *ptr_plc_element_current = read_response_buffer.data() + read_instruction.read_buffer_offset_data;

            // TODO: performance - Hoist the switch/case above for loop?

            switch (read_instruction.plc_type)
            {
            case PLCType::LREAL:
            {
                double val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, val);
                break;
            }
            case PLCType::REAL:
            {
                float val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::BOOL:
            {
                uint8_t byte_val;
                memcpy(&byte_val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, (byte_val != 0) ? 1.0 : 0.0);
                break;
            }
            case PLCType::SINT:
            {
                int8_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                uint8_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::INT:
            {
                int16_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::UINT:
            {
                uint16_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::DINT:
            {
                int32_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::UDINT:
            {
                uint32_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            /* Not supported for now, guarded against in on_configure()
            case PLCType::STRING:
                break;
            */
            case PLCType::UNKNOWN:
            default:
                RCLCPP_ERROR_THROTTLE(getLogger(), logging_throttle_clock_, 1000,
                                      "Unhandled or UNKNOWN PLC type (%d) for the interface '%s' during read.",
                                      static_cast<int>(read_instruction.plc_type), read_instruction.state_interface_name.c_str());
                set_state(read_instruction.state_interface_name, std::numeric_limits<double>::quiet_NaN());
                any_item_read_failed = true;
                break;
            }
        }

        if (refresh_handles_required)
        {
            RCLCPP_WARN_THROTTLE(
                getLogger(), logging_throttle_clock_, 1000,
                "ADS symbol version invalid during read. Refreshing ADS handles and rebuilding sum buffers.");
            if (!refresh_ads_handles() ||
                !build_sum_read_buffers() ||
                !build_sum_write_buffers())
            {
                RCLCPP_ERROR_THROTTLE(
                    getLogger(), logging_throttle_clock_, 1000,
                    "Failed to refresh ADS handles after symbol version invalid error.");
            }
        }
        return any_item_read_failed ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (num_items_write_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        auto &write_request_buffer = ads_buffer_sum_write_request_;

        for (const auto &write_instruction : ads_write_instructions_)
        {
            uint8_t *ptr_write_buffer_destination_current =
                write_request_buffer.data() + write_instruction.write_buffer_offset_data;

            // TODO: performance - Hoist the switch/case above for loop?

            // store the current val and reset the ros-side command value
            double val = get_command(write_instruction.command_interface_name);
            set_command(write_instruction.command_interface_name, std::numeric_limits<double>::quiet_NaN());

            if (std::isnan(val))
            {
                // if the original value was NaN and there exist a state interface of the same name, write corresponding state interface
                if (!write_instruction.fallback_state_interface_name.empty())
                {
                    val = get_state(write_instruction.fallback_state_interface_name);
                }

                // if we STILL don't have a fallback value on, don't update the write buffer.
                // the last valid command is written
                if (std::isnan(val))
                {
                    continue;
                }
            }

            switch (write_instruction.plc_type)
            {
            case PLCType::LREAL:
            {
                // val is already double (LREAL is 8 bytes - 64 bit)
                memcpy(ptr_write_buffer_destination_current, &val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::REAL:
            {
                float plc_val = static_cast<float>(val);
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::BOOL:
            {
                // bool is size of byte in PLC
                uint8_t plc_val = (val != 0.0) ? 1 : 0;
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::SINT:
            {
                int8_t plc_val = static_cast<int8_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                uint8_t plc_val = static_cast<uint8_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::INT:
            {
                int16_t plc_val = static_cast<int16_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::UINT:
            {
                uint16_t plc_val = static_cast<uint16_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::DINT:
            {
                int32_t plc_val = static_cast<int32_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::UDINT:
            {
                uint32_t plc_val = static_cast<uint32_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            /* String not supported for now
            case PLCType::STRING: break;
            */
            case PLCType::UNKNOWN:
            default:
                RCLCPP_FATAL(getLogger(), "UNKNOWN PLC type (%d) for the interface '%s' during write. Sending zeroed data of size %zu.",
                             static_cast<int>(write_instruction.plc_type), write_instruction.command_interface_name.c_str(), plcTypeByteSize(write_instruction.plc_type));
                return hardware_interface::return_type::ERROR;
                break;
            }
        }

        if (use_ads_async_write_)
        {
            const long ads_sum_write_queue_error = queue_ads_read_write_no_response(
                *ads_device_,
                ADSIGRP_SUMUP_WRITE,
                num_items_write_,
                ads_buffer_sum_write_response_.size(),
                ads_buffer_sum_write_request_.size(),
                ads_buffer_sum_write_request_.data());

            if (ads_sum_write_queue_error != ADSERR_NOERR)
            {
                RCLCPP_ERROR_THROTTLE(
                    getLogger(), logging_throttle_clock_, 1000,
                    "Failed to queue async ADS Sum Write request: 0x%lX.",
                    ads_sum_write_queue_error);
                return hardware_interface::return_type::ERROR;
            }

            return hardware_interface::return_type::OK;
        }

        uint32_t bytes_response_buffer_from_plc = 0;
        long ads_sum_write_error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_WRITE,
            num_items_write_,
            ads_buffer_sum_write_response_.size(),
            ads_buffer_sum_write_response_.data(),
            ads_buffer_sum_write_request_.size(),
            ads_buffer_sum_write_request_.data(),
            &bytes_response_buffer_from_plc);

        if (ads_sum_write_error != ADSERR_NOERR)
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), logging_throttle_clock_, 1000,
                                  "Overall ADS Sum Write Error: 0x%lX.", ads_sum_write_error);
            return hardware_interface::return_type::ERROR;
        }

        if (bytes_response_buffer_from_plc != ads_buffer_sum_write_response_.size())
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), logging_throttle_clock_, 1000,
                                  "ADS Sum Write response size mismatch (error codes). Expected %zu, Got %u.",
                                  ads_buffer_sum_write_response_.size(), bytes_response_buffer_from_plc);
        }

        bool any_item_write_failed = false;
        bool refresh_handles_required = false;
        for (size_t i = 0; i < num_items_write_; ++i)
        {
            uint32_t item_error_code;
            memcpy(&item_error_code, ads_buffer_sum_write_response_.data() + i * sizeof(uint32_t), sizeof(uint32_t));
            if (item_error_code != ADSERR_NOERR)
            {
                RCLCPP_WARN_THROTTLE(getLogger(), logging_throttle_clock_, 1000,
                                     "ADS Sum Write sub-op for '%s' failed: 0x%X (handle 0x%X, request %zu bytes).",
                                     ads_item_layouts_write_[i].plc_name_symbolic.c_str(),
                                     item_error_code,
                                     ads_item_layouts_write_[i].ads_handle,
                                     ads_item_layouts_write_[i].plc_element_byte_size * ads_item_layouts_write_[i].num_elements);
                if (item_error_code == ADSERR_DEVICE_SYMBOLVERSIONINVALID)
                {
                    refresh_handles_required = true;
                }
                any_item_write_failed = true;
            }
        }
        if (refresh_handles_required)
        {
            RCLCPP_WARN_THROTTLE(
                getLogger(), logging_throttle_clock_, 1000,
                "ADS symbol version invalid during write. Refreshing ADS handles and rebuilding sum buffers.");
            if (!refresh_ads_handles() ||
                !(use_ads_notifications_for_read_
                      ? (build_notification_read_buffers() && configure_ads_notifications())
                      : build_sum_read_buffers()) ||
                !build_sum_write_buffers())
            {
                RCLCPP_ERROR_THROTTLE(
                    getLogger(), logging_throttle_clock_, 1000,
                    "Failed to refresh ADS handles after symbol version invalid write error.");
            }
        }

        return any_item_write_failed ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(getLogger(), "Releasing ADS resources...");
        clear_ads_notifications();
        if (ads_device_)
        {
            ads_device_.reset();
        }
        RCLCPP_INFO(getLogger(), "ADS resources released.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    bool BeckhoffADSHardwareInterface::configure_ads_device()
    {
        RCLCPP_INFO(getLogger(), "Configuring ADS device...");
        const auto &params = info_.hardware_parameters;
        try
        {
            std::string plc_ip = params.at("plc_ip_address");
            std::string plc_ams_net_id_str = params.at("plc_ams_net_id");
            std::string local_ams_net_id_str = params.at("local_ams_net_id");
            uint16_t plc_ams_port = std::stoul(params.at("plc_ams_port"));

            AmsNetId remote_net_id;
            if (sscanf(plc_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &remote_net_id.b[0], &remote_net_id.b[1], &remote_net_id.b[2],
                       &remote_net_id.b[3], &remote_net_id.b[4], &remote_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'plc_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            AmsNetId local_net_id;
            if (sscanf(local_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &local_net_id.b[0], &local_net_id.b[1], &local_net_id.b[2],
                       &local_net_id.b[3], &local_net_id.b[4], &local_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'local_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            bhf::ads::SetLocalAddress(local_net_id);
            ads_device_ = std::make_unique<AdsDevice>(plc_ip, remote_net_id, plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tTimeout is: %u", ads_device_->GetTimeout());

            RCLCPP_INFO(getLogger(), "\tADS Device configured for PLC: %s, Port: %u", plc_ip.c_str(), plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tPLC AMS NetID: %s", plc_ams_net_id_str.c_str());

            RCLCPP_INFO(getLogger(), "Requesting Device state...");
            AdsDeviceState deviceState = ads_device_->GetState();
            RCLCPP_INFO(getLogger(), "\tCommunication successful! ADS State: %d, DeviceState: %d", deviceState.ads, deviceState.device);
        }
        catch (const std::out_of_range &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tMissing required URDF <hardware> parameter: %s", ex.what());
            return false;
        }
        catch (const AdsException &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tADS Exception during connection: %s (Error Code: 0x%lX)", ex.what(), ex.errorCode);
            return false;
        }
        catch (const std::exception &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tError during ADS connection config: %s", ex.what());
            return false;
        }

        return true;
    }

    PLCType BeckhoffADSHardwareInterface::strToPlcType(const std::string &type_str_param)
    {
        std::string type_str = type_str_param;
        std::transform(type_str.begin(), type_str.end(), type_str.begin(), ::toupper);

        if (type_str == "LREAL")
            return PLCType::LREAL;
        if (type_str == "REAL")
            return PLCType::REAL;
        if (type_str == "BOOL")
            return PLCType::BOOL;
        if (type_str == "UDINT")
            return PLCType::UDINT;
        if (type_str == "DINT")
            return PLCType::DINT;
        if (type_str == "UINT")
            return PLCType::UINT;
        if (type_str == "INT")
            return PLCType::INT;
        if (type_str == "USINT")
            return PLCType::USINT;
        if (type_str == "SINT")
            return PLCType::SINT;
        if (type_str == "BYTE")
            return PLCType::BYTE;
        if (type_str == "STRING")
            return PLCType::STRING;

        RCLCPP_ERROR(getLogger(), "Unknown PLC type string: '%s'", type_str_param.c_str());
        return PLCType::UNKNOWN;
    }

    size_t BeckhoffADSHardwareInterface::plcTypeByteSize(PLCType plc_type_enum)
    {
        switch (plc_type_enum)
        {
        case PLCType::LREAL:
            return 8;
        case PLCType::REAL:
            return 4;
        case PLCType::BOOL:
            return 1;
        case PLCType::UDINT:
            return 4;
        case PLCType::DINT:
            return 4;
        case PLCType::UINT:
            return 2;
        case PLCType::INT:
            return 2;
        case PLCType::USINT:
            return 1;
        case PLCType::SINT:
            return 1;
        case PLCType::BYTE:
            return 1;
        // case PLCType::STRING: not currently supported
        case PLCType::UNKNOWN:
        default:
            RCLCPP_ERROR(getLogger(), "Cannot get byte size for UNKNOWN or unhandled PLC type enum value: %d", static_cast<int>(plc_type_enum));
            return 0;
        }
    }

} // namespace beckhoff_ads_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface, hardware_interface::SystemInterface)
