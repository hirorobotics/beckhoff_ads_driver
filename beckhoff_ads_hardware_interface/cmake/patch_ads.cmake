if(NOT DEFINED ADS_SOURCE_DIR)
  message(FATAL_ERROR "ADS_SOURCE_DIR is required")
endif()

set(SOCKETS_CPP "${ADS_SOURCE_DIR}/AdsLib/Sockets.cpp")
if(NOT EXISTS "${SOCKETS_CPP}")
  message(FATAL_ERROR "ADS Sockets.cpp not found at ${SOCKETS_CPP}")
endif()

file(READ "${SOCKETS_CPP}" SOCKETS_CONTENT)
if(SOCKETS_CONTENT MATCHES "const int enable = 0;")
  string(REPLACE "const int enable = 0;" "const int enable = 1;" SOCKETS_CONTENT "${SOCKETS_CONTENT}")
  file(WRITE "${SOCKETS_CPP}" "${SOCKETS_CONTENT}")
  message(STATUS "beckhoff_ads_hardware_interface: patched ADS TCP_NODELAY to 1")
elseif(SOCKETS_CONTENT MATCHES "const int enable = 1;")
  message(STATUS "beckhoff_ads_hardware_interface: ADS TCP_NODELAY patch already applied")
else()
  message(FATAL_ERROR "beckhoff_ads_hardware_interface: could not find TCP_NODELAY option in ${SOCKETS_CPP}")
endif()

set(ADS_DEVICE_H "${ADS_SOURCE_DIR}/AdsLib/AdsDevice.h")
set(ASYNC_WRITE_PATCH "${CMAKE_CURRENT_LIST_DIR}/../patches/ads_async_write_no_response.patch")
if(NOT EXISTS "${ADS_DEVICE_H}")
  message(FATAL_ERROR "ADS AdsDevice.h not found at ${ADS_DEVICE_H}")
endif()
if(NOT EXISTS "${ASYNC_WRITE_PATCH}")
  message(FATAL_ERROR "beckhoff_ads_hardware_interface: async write patch not found at ${ASYNC_WRITE_PATCH}")
endif()

file(READ "${ADS_DEVICE_H}" ADS_DEVICE_H_CONTENT)
if(ADS_DEVICE_H_CONTENT MATCHES "ReadWriteReqExAsync")
  message(STATUS "beckhoff_ads_hardware_interface: ADS async/no-response write patch already applied")
else()
  execute_process(
    COMMAND git apply --ignore-space-change --ignore-whitespace "${ASYNC_WRITE_PATCH}"
    WORKING_DIRECTORY "${ADS_SOURCE_DIR}"
    RESULT_VARIABLE ASYNC_PATCH_RESULT
    OUTPUT_VARIABLE ASYNC_PATCH_OUTPUT
    ERROR_VARIABLE ASYNC_PATCH_ERROR)
  if(NOT ASYNC_PATCH_RESULT EQUAL 0)
    message(FATAL_ERROR
      "beckhoff_ads_hardware_interface: failed to apply async/no-response write patch\n"
      "${ASYNC_PATCH_OUTPUT}\n${ASYNC_PATCH_ERROR}")
  endif()
  message(STATUS "beckhoff_ads_hardware_interface: patched ADS async/no-response write API")
endif()
