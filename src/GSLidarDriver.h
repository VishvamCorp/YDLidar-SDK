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

/** @page GSLidarDriver
 * GSLidarDriver API
    <table>
        <tr><th>Library     <td>GSLidarDriver
        <tr><th>File        <td>GSLidarDriver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This GSLidarDriver support [TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE) and [TYPE_TOF](\ref LidarTypeID::TYPE_TOF) LiDAR

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::GSLidarDriver @endlink interface documentation.
*/
#ifndef GS2_YDLIDAR_DRIVER_H
#define GS2_YDLIDAR_DRIVER_H

#include <stdlib.h>
#include <atomic>
#include <map>
#include <list>
#include "core/serial/serial.h"
#include "core/base/locker.h"
#include "core/base/thread.h"
#include "core/common/ydlidar_protocol.h"
#include "core/common/ydlidar_help.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif

using namespace std;

namespace ydlidar
{

  using namespace core;
  using namespace core::serial;
  using namespace core::base;

  /*!
   * GS lidar control class
   */
  class GSLidarDriver : public DriverInterface
  {
  public:
    /*!
     * A constructor.
     * A more elaborate description of the constructor.
     */
    GSLidarDriver(uint8_t type = YDLIDAR_TYPE_SERIAL);
    /*!
     * A destructor.
     * A more elaborate description of the destructor.
     */
    virtual ~GSLidarDriver();

    /*!
     * @brief Connect to the lidar. \n
     * After a successful connection, close it with ::disconnect.
     * @param[in] port_path Serial port path
     * @param[in] baudrate   Baud rate (YDLIDAR-GS2 uses 961200)
     * @return Connection result
     * @retval 0     Success
     * @retval < 0   Failure
     * @note After connecting successfully, close it with ::disconnect.
     * @see ::GSLidarDriver::disconnect ("::" marks linkable functions in the docs; click to jump there.)
     */
    result_t connect(const char *port_path, uint32_t baudrate);

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
     * @brief Configure whether the lidar returns intensity data \\n
     * After connecting successfully, close it with ::disconnect
     * @param[in] isintensities    Whether intensity is enabled:
     *     true    include intensity values
     *	  false no intensity values
     * @note Only the S4B lidar (baud rate 153600) supports intensity; other models do not.
     */
    void setIntensities(const bool &isintensities);

    /*!
     * @brief Enable automatic reconnection when the lidar encounters errors \\n
     * @param[in] enable    Whether auto reconnection is enabled:
     *     true    enable
     *	  false disable
     */
    void setAutoReconnect(const bool &enable);

    /*!
     * @brief Retrieve lidar device information \\n
     * @param[in] parameters     Device information
     * @param[in] timeout  Timeout
     * @return Execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE or RESULT_TIMEOUT   Failure
     */
    result_t getDevicePara(gs_device_para &info, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Configure the lidar address \\n
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
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     * @note A single successful start is sufficient
     */
    result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Stop scanning \\n
     * @return Execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     */
    result_t stop();

    /*!
     * @brief Retrieve laser data \\n
     * @param[in] nodebuffer Laser point information
     * @param[in] count      Number of points per revolution
     * @param[in] timeout    Timeout
     * @return Execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Retrieval failed
     * @note Call ::startScan before requesting data
     */
    result_t grabScanData(node_info *nodebuffer, size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

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
     * @brief Reset the lidar \\n
     * @param[in] timeout      Timeout
     * @return Execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     * @note Perform this operation after stopping the scan; call ::stop if scanning is active
     */
    result_t reset(uint8_t addr, uint32_t timeout = DEFAULT_TIMEOUT);

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
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     * @note Called automatically during SDK reconnection
     */
    result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

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
    result_t sendCommand(uint8_t cmd,
                         const void *payload = NULL,
                         size_t payloadsize = 0);

    /*!
     * @brief Send data to the lidar \\n
     * @param[in] addr Module address
     * @param[in] cmd     Command code
     * @param[in] payload      payload
     * @param[in] payloadsize      payloadsize
     * @return Execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     */
    result_t sendCommand(uint8_t addr,
                         uint8_t cmd,
                         const void *payload = NULL,
                         size_t payloadsize = 0);

    /*!
     * @brief Wait for a lidar packet header \\n
     * @param[in] header     Packet header
     * @param[in] timeout      Timeout
     * @return Execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_TIMEOUT  Timed out while waiting
     * @retval RESULT_FAILE    Retrieval failed
     * @note When timeout = -1, wait indefinitely
     */
    result_t waitResponseHeader(gs_package_head *header,
                                uint32_t timeout = DEFAULT_TIMEOUT);
    result_t waitResponseHeaderEx(gs_package_head *header,
                                  uint8_t cmd,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Wait for a fixed amount of serial data \\n
     * @param[in] data_count     Expected data length
     * @param[in] timeout        Wait duration
     * @param[in] returned_size   Actual data length
     * @return Execution result
     * @retval RESULT_OK       Success
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
     * @retval RESULT_OK       Success
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
     * @brief checkTransDelay
     */
    void checkTransDelay();

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

    /*!
     * @brief Convert raw data to point distance and angle
     */
    void angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);
    void angTransform2(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);

    /**
     * @brief Serial error information
     * @param isTCP   TCP or UDP
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

    //Retrieve device information
    virtual result_t getDeviceInfo(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT/4);
    //Retrieve cascaded lidar device information
    virtual result_t getDeviceInfo(
      std::vector<device_info_ex> &dis,
      uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t getDeviceInfo1(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t getDeviceInfo2(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief Set the lidar work mode (currently GS2 only)
     * @param[in] mode Lidar work mode (0 obstacle avoidance, 1 edge-following)
     * @param[in] addr Lidar address (0x01 for the first, 0x02 for the second, 0x04 for the third)
     * @return RESULT_OK on success; otherwise a different error code
     */
    virtual result_t setWorkMode(int mode = 0, uint8_t addr = 0x00);

    //Start OTA upgrade
    virtual bool ota();
    //Start OTA
    bool startOta(uint8_t addr);
    //OTA in progress
    bool execOta(uint8_t addr, const std::vector<uint8_t>& data);
    //Stop OTA
    bool stopOta(uint8_t addr);
    //Verify whether the response is valid
    bool isOtaRespOk(uint8_t addr,
                     uint8_t cmd,
                     uint16_t offset,
                     const std::vector<uint8_t>& data);
    bool sendData(uint8_t addr,
                  uint8_t cmd,
                  const std::vector<uint8_t> &data,
                  uint8_t cmdRecv,
                  std::vector<uint8_t> &dataRecv,
                  int timeout = 500);

    //Unimplemented virtual function
    virtual result_t getScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyDis(scan_frequency &frequency,
                                         uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyAdd(scan_frequency &frequency,
                                         uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyAddMic(scan_frequency &frequency,
                                            uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyDisMic(scan_frequency &frequency,
                                            uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t getSamplingRate(sampling_rate &rate,
                                     uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setSamplingRate(sampling_rate &rate,
                                     uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t getZeroOffsetAngle(offset_angle &angle,
                                        uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanHeartbeat(scan_heart_beat &beat,
                                      uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }

  public:
    enum
    {
      DEFAULT_TIMEOUT = 2000,    /**< Default timeout. */
      DEFAULT_HEART_BEAT = 1000, /**< Default power-loss detection interval. */
      MAX_SCAN_NODES = 160 * 3,     /**< Maximum number of scan points. */
      DEFAULT_TIMEOUT_COUNT = 3, //Error count
    };

  private:
    int PackageSampleBytes; //Number of lidar points per packet
    ChannelDevice *_comm = nullptr; //Communication object
    uint32_t trans_delay; //Time to transmit one byte over serial
    int sample_rate; //Sampling rate

    gs_node_package package; //Packet with intensity data

    uint8_t CheckSum; //Checksum
    uint8_t CheckSumCal;
    bool CheckSumResult;

    uint8_t *globalRecvBuffer = nullptr;

    double k0[LIDAR_MAXCOUNT];
    double k1[LIDAR_MAXCOUNT];
    double b0[LIDAR_MAXCOUNT];
    double b1[LIDAR_MAXCOUNT];
    double bias[LIDAR_MAXCOUNT];
    int m_models[LIDAR_MAXCOUNT] = {0};
    int model = YDLIDAR_GS2; //Lidar model
    uint8_t moduleNum = 0; //Module index
    uint8_t moduleCount = 1; //Current module count
    int nodeCount = 0; //Point count in current packet
    uint64_t stamp = 0; //Timestamp
    std::list<gs_module_nodes> datas; //Data for each module
    double m_pitchAngle = Angle_PAngle;
    uint32_t lastStamp = 0; //Previous timestamp
  };

} // namespace ydlidar

#endif // GS2_YDLIDAR_DRIVER_H
