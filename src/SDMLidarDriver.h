/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** @page SDMLidarDriver
 * SDMLidarDriver API
    <table>
        <tr><th>Library     <td>SDMLidarDriver
        <tr><th>File        <td>SDMLidarDriver.h
        <tr><th>Author      <td>ZhanYi [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This SDMLidarDriver support [TYPE_SDM](\ref LidarTypeID::TYPE_SDM) LiDAR

* @copyright    Copyright (c) @2015-2023 EAIBOT
     Jump to the @link ::ydlidar::SDMLidarDriver @endlink interface documentation.
*/
#ifndef SDM_YDLIDAR_DRIVER_H
#define SDM_YDLIDAR_DRIVER_H

#include <stdlib.h>
#include <atomic>
#include <map>
#include "core/serial/serial.h"
#include "core/base/locker.h"
#include "core/base/thread.h"
#include "core/common/ydlidar_protocol.h"
#include "core/common/ydlidar_help.h"
#include "core/logger/logger.hpp"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif

#define SDK_SDM_POINT_COUNT 1
#define SDK_CMD_HEADFLAG0 0xAA //Protocol header flag 1
#define SDK_CMD_HEADFLAG1 0x55 //Protocol header flag 2
#define SDK_CMD_STARTSCAN 0x60 //Start ranging
#define SDK_CMD_STOPSCAN 0x61 //Stop ranging
#define SDK_CMD_GETVERSION 0x62 //Get version information
#define SDK_CMD_SELFCHECK 0x63 //Self-check
#define SDK_CMD_SETFREQ 0x64 //Set output frequency
#define SDK_CMD_SETFILTER 0x65 //Configure filtering
#define SDK_CMD_SETBAUDRATE 0x66 //Set serial baud rate
#define SDK_CMD_SETOUTPUT 0x67 //Set output data format
#define SDK_CMD_RESET 0x68 //Restore factory settings
#define SDK_BUFFER_MAXLEN 100 //Buffer length

//Use 1-byte alignment
#pragma pack(1)

//SDM lidar protocol header
struct SdkSdmHead {
    uint8_t head0 = 0;
    uint8_t head1 = 0;
    uint8_t cmd = 0;
    uint8_t size = 0;
};
#define SDKSDMHEADSIZE sizeof(SdkSdmHead)
//SDM lidar single-point data
struct SdkSdmPc {
    uint16_t dist = 0; //Distance
    uint8_t intensity = 0; //Intensity
    uint8_t env = 0; //Environmental interference data
};
//SDM lidar packet containing a single point
struct SdkSdmPcs {
    SdkSdmHead head;
    SdkSdmPc point; //Each packet contains a single point
    uint8_t cs = 0;
};
#define SDKSDMPCSSIZE sizeof(SdkSdmPcs)
//SDM lidar device information
struct SdkSdmDeviceInfo {
    uint8_t model = 0; //Lidar model number
    uint8_t hv = 0; //Hardware version
    uint8_t fvm = 0; //Firmware major version
    uint8_t fvs = 0; //Firmware minor version
    uint8_t sn[SDK_SNLEN] = {0}; //Serial number
};
#define SDKSDMDEVICEINFOSIZE sizeof(SdkSdmDeviceInfo)

#pragma pack()


using namespace std;

namespace ydlidar
{

using namespace core;
using namespace core::serial;
using namespace core::base;

/*!
* SDM control class
*/
class SDMLidarDriver : public DriverInterface
{
public:
  //Constructor
  SDMLidarDriver();
  //Destructor
  virtual ~SDMLidarDriver();
  /*!
  * @brief Connect to the lidar \\n
  * After connecting successfully, close it with ::disconnect
  * @param[in] port Serial port
  * @param[in] baudrate   Baud rate (YDLIDAR-SDM uses 961200)
  * @return Connection status
  * @retval 0     Success
  * @retval < 0   Failure
  * @note After connecting successfully, close it with ::disconnect
  * @see ::GSLidarDriver::disconnect ("::" marks linkable functions in the docs; click to jump there.)
  */
  result_t connect(const char *port, uint32_t baudrate);

  /*!
  * @brief Disconnect from the lidar
  */
  void disconnect();

  /*!
  * @brief Get the current SDK version \\n
  * Static function
  * @return Current SDK version
  */
  virtual std::string getSDKVersion();

  /*!
  * @brief Retrieve the available lidar ports
  * @return Map of detected lidars
  */
  static std::map<std::string, std::string> lidarPortList();

  /*!
  * @brief Scanning status \\n
  * @return Current scanning status
  * @retval true     Currently scanning
  * @retval false    Scanning stopped
  */
  bool isscanning() const;

  /*!
  * @brief Connection status \\n
  * @return Connection status
  * @retval true     Success
  * @retval false    Failure
  */
  bool isconnected() const;

  /*!
  * @brief Enable automatic reconnection when the lidar encounters errors \\n
  * @param[in] enable    Whether auto reconnection is enabled:
  *     true    enable
  *     false disable
  */
  void setAutoReconnect(const bool &enable);

  /*!
 * @brief 配置雷达地址 \n
 * @param[in] timeout  Timeout
 * @return Execution result
 * @retval RESULT_OK       Success
 * @retval RESULT_FAILE or RESULT_TIMEOUT   Timeout
 */
  result_t setDeviceAddress(uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Start scanning \\n
  * @param[in] force    Scanning mode
  * @param[in] timeout  Timeout
  * @return Execution result
  * @retval RESULT_OK       Start succeeded
  * @retval RESULT_FAILE    Start failed
  * @note A single successful start is sufficient
  */
  result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Stop scanning \\n
  * @return Execution result
  * @retval RESULT_OK       Stop succeeded
  * @retval RESULT_FAILE    Stop failed
  */
  result_t stop();

  /*!
  * @brief Retrieve laser data \\n
  * @param[in] nodebuffer Laser point information
  * @param[in] count      Number of points per revolution
  * @param[in] timeout    Timeout
  * @return Execution result
  * @retval RESULT_OK       Retrieval succeeded
  * @retval RESULT_FAILE    Retrieval failed
  * @note Call ::startScan before requesting data
  */
  result_t grabScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT) ;


  /*!
  * @brief Compensate laser angles \\n
  * Clamp angles to 0–360 degrees
  * @param[in] nodebuffer Laser point information
  * @param[in] count      Number of points per revolution
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Ensure ::grabScanData succeeds before applying compensation
  */
  result_t ascendScanData(node_info *nodebuffer, size_t count);

  /*!
  * @brief Reset the lidar (restore factory settings) \\n
  * @param[in] timeout      Timeout
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Perform this operation after stopping the scan; call stop if scanning is active
  */
  result_t reset(uint8_t addr, uint32_t timeout = DEFAULT_TIMEOUT);

  //Enable or disable filtering
  result_t enableFilter(bool yes=true);

  //Set scan frequency
  result_t setScanFreq(float sf, uint32_t timeout);

 protected:

  /*!
  * @brief Create the data parsing thread \\n
  * @note Call ::startScan successfully before creating the parsing thread
  */
  result_t createThread();


  /*!
  * @brief Restart scanning after reconnection \\n
  * @param[in] force    Scanning mode
  * @param[in] timeout  Timeout
  * @return Execution result
  * @retval RESULT_OK       Start succeeded
  * @retval RESULT_FAILE    Start failed
  * @note Called automatically during SDK reconnection
  */
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /*!
  * @brief stopScan
  * @param timeout
  * @return
  */
  result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
   * @brief waitDevicePackage
   * @param timeout
   * @return
   */
  result_t waitDevicePackage(uint32_t timeout = DEFAULT_TIMEOUT);
  /*!
  * @brief Decode lidar data \\n
  * @param[in] node Lidar point after decoding
  * @param[in] timeout     Timeout
  */
  result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Send data to the lidar \\n
  * @param[in] nodebuffer Pointer to lidar data
  * @param[in] count      Number of lidar points
  * @param[in] timeout      Timeout
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_TIMEOUT  Timed out while waiting
  * @retval RESULT_FAILE    Failure
  */
  result_t waitScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Laser data parsing thread \\n
  */
  int cacheScanData();

  /*!
  * @brief Send data to the lidar \\n
  * @param[in] cmd     Command code
  * @param[in] payload      payload
  * @param[in] payloadsize      payloadsize
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  */
  result_t sendCmd(uint8_t cmd,
                       const uint8_t *data = NULL,
                       size_t size = 0);

  /*!
  * @brief Wait for response data \\n
  * @param[in] cmd Command code
  * @param[out] data Response data
  * @param[in] timeout Timeout
  * @return Execution result
  * @retval RESULT_OK       Retrieval succeeded
  * @retval RESULT_TIMEOUT  Timed out while waiting
  * @retval RESULT_FAILE    Retrieval failed
  * @note When timeout = -1, wait indefinitely
  */
  result_t waitResp(uint8_t cmd,
                   uint32_t timeout = DEFAULT_TIMEOUT);
  result_t waitResp(uint8_t cmd,
                   std::vector<uint8_t> &data,
                   uint32_t timeout = DEFAULT_TIMEOUT);
  /*!
  * @brief Wait for a lidar packet header \\n
  * @param[out] head Packet header
  * @param[in] cmd Command code
  * @param[out] data Response data
  * @param[in] timeout Timeout
  * @return Execution result
  * @retval RESULT_OK       Retrieval succeeded
  * @retval RESULT_TIMEOUT  Timed out while waiting
  * @retval RESULT_FAILE    Retrieval failed
  * @note When timeout = -1, wait indefinitely
  */
  // result_t waitResHeader(SdkSdmHead *head,
  //                        uint8_t cmd,
  //                        uint32_t timeout = DEFAULT_TIMEOUT);
  // result_t waitResHeader(SdkSdmHead *head,
  //                        uint8_t cmd,
  //                        std::vector<uint8_t> &data,
  //                        uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Wait for a fixed amount of serial data \\n
  * @param[in] data_count     Expected data length
  * @param[in] timeout        Wait duration
  * @param[in] returned_size   Actual data length
  * @return Execution result
  * @retval RESULT_OK       Retrieval succeeded
  * @retval RESULT_TIMEOUT  Timed out while waiting
  * @retval RESULT_FAILE    Retrieval failed
  * @note When timeout = -1, wait indefinitely
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /*!
  * @brief Retrieve data from the serial port \\n
  * @param[in] data     Data pointer
  * @param[in] size    Data length
  * @return Execution result
  * @retval RESULT_OK       Retrieval succeeded
  * @retval RESULT_FAILE    Retrieval failed
  */
  result_t getData(uint8_t *data, size_t size);

  /*!
  * @brief Send data over the serial interface \\n
  * @param[in] data     Pointer to the data to send
  * @param[in] size    Data length
  * @return Execution result
  * @retval RESULT_OK       Send succeeded
  * @retval RESULT_FAILE    Send failed
  */
  result_t sendData(const uint8_t *data, size_t size);
  /*!
  * @brief Disable the data acquisition channel \\n
  */
  void disableDataGrabbing();
  /*!
  * @brief Set the serial DTR \\n
  */
  void setDTR();
  /*!
  * @brief Clear the serial DTR \\n
  */
  void clearDTR();
  /*!
   * @brief flushSerial
   */
  void flushSerial();
  /*!
   * @brief checkAutoConnecting
   */
  result_t checkAutoConnecting();
  /**
   * @brief Error information
   * @param isTCP TCP or UDP
   * @return error information
   */
  virtual const char *DescribeError(bool isTCP = false);
  /**
   * @brief GS2 lidar has no health information \\n
   * @return result status
   * @retval RESULT_OK success
   * @retval RESULT_FAILE or RESULT_TIMEOUT failed
   */
  virtual result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);
    /**
   * @brief get Device information \n
   * @param[in] info     Device information
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  virtual result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

private:
  ydlidar::core::Log::Logger logger; ///< Logger instance
  serial::Serial *_serial = nullptr; //Serial interface
  std::vector<uint8_t> recvBuff; //Packet buffer
  device_health health_;
};

} // namespace ydlidar

#endif // SDM_YDLIDAR_DRIVER_H
