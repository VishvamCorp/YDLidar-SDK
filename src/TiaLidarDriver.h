#pragma once
#include <vector>
#include <math.h>
#include "core/base/thread.h"
#include "core/base/locker.h"
#include "core/common/ydlidar_protocol.h"
#include "core/common/ydlidar_datatype.h"
#include "core/common/DriverInterface.h"
#include "core/json/cJSON.h"


//Lidar parameters
struct EaiLidarBaseParam
{
    int type = 0; //Lidar type
};
//TIA lidar parameters
struct EaiTiaParam : public EaiLidarBaseParam
{
    std::string ip = "192.168.0.11"; //IP address
    int port = 8090; //Port number
    std::string mask = "255.255.255.0"; //Subnet mask
    std::string gateway = "192.168.0.1"; //Gateway
    std::string mac; //MAC address

    float scanFreq = .0f; //Scan frequency in Hz
    float sampleRate = .0f; //Sampling rate in K/s
};
//TIA-X lidar parameters
struct EaiTiaXParamItem //Entry
{
    float angle = 45.0f; //Mirror mounting angle (°)
    float dist = 30.0f; //Mirror mounting distance (mm)
    float opd = 0.0f; //Optical path difference (mm)
    float minAngle = 60.0f; //Mirror start angle (°)
    float maxAngle = 100.0f; //Mirror end angle (°)

    //Adjust angle and distance based on parameters
    bool correct(float &angle, float &dist) const {
        //Return immediately if the distance is invalid
        if (int(dist) <= 0)
            return false;
        bool has = angle > SDK_ANGLE180; //Flag for angles exceeding 180°
        //Check whether the angle value is valid
        if (angle >= minAngle &&
            angle <= maxAngle) //Left mirror
        {
            if (has)
                angle = SDK_ANGLE360 - angle;
            float beta = SDK_ANGLE180 - this->angle - angle;
            float a = this->dist / sin(beta * M_PI / SDK_ANGLE180); //Side length a
            float b = dist - a; //Side length b
            float C = (SDK_ANGLE180 - 2 * beta) * M_PI / SDK_ANGLE180; //Angle C (radians)
            float c = sqrt(a * a + b * b - 2 * a * b * cos(C)); //Side length c
            //Law of cosines calculation
            float B = acos((a * a + c * c - b * b) / (2 * a * c)) *
                    SDK_ANGLE180 / M_PI; //Angle B (degrees)
            if (has)
                angle = SDK_ANGLE360 - (B + angle);
            else
                angle += B;

            dist = c + this->opd; //Increase distance by the optical path difference

            return true;
        }
        return false;
    }
};
struct EaiTiaXParam : public EaiTiaParam
{
    EaiTiaXParamItem left; //Left mirror parameters
    EaiTiaXParamItem right; //Right mirror parameters
};

namespace ydlidar {
namespace core {
namespace network {
class CActiveSocket;
class CPassiveSocket;
}
}

using namespace core::common;
using namespace core::base;
using namespace core::network;

class TiaLidarDriver : public DriverInterface
{
public:
    explicit TiaLidarDriver();
    ~TiaLidarDriver();

    //Connect to the lidar
    virtual result_t connect(const char *port_path, uint32_t baudrate = 8000);
    //Describe current error
    virtual const char *DescribeError(bool isTCP = true);
    //Disconnect from the lidar
    virtual void disconnect();
    //Check if scanning
    virtual bool isscanning() const;
    //Check if connected
    virtual bool isconnected() const;

    /*!
  * @brief Configure whether the lidar returns intensity data \n
  * After a successful connection, close it with ::disconnect.
  * @param[in] isintensities    Whether intensity is enabled:
  *     true    include intensity values
  *    false no intensity values
  * @note Only the S4B lidar (baud rate 153600) supports intensity; other models do not.
  */
    virtual void setIntensities(const int &i);

    /*!
  * @brief Enable automatic reconnection when the lidar encounters errors \n
  * @param[in] enable    Whether auto reconnection is enabled:
  *     true    enable
  *    false disable
  */
    virtual void setAutoReconnect(const bool &enable);
    //Retrieve lidar health status
    virtual result_t getHealth(
            device_health &health,
            uint32_t timeout = SDK_TIMEOUT);
    //Retrieve device information
    virtual result_t getDeviceInfo(
            device_info &info,
            uint32_t timeout = SDK_TIMEOUT);
    // //Start the lidar
    // virtual result_t start(
    //         bool force = false,
    //         uint32_t timeout = SDK_TIMEOUT);
    // //Stop the lidar
    // virtual result_t stop();
    /*!
  * @brief Retrieve laser data \n
  * @param[in] nodebuffer Laser point information
  * @param[in] count      Number of points in one revolution
  * @param[in] timeout    Timeout duration
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Call ::startScan before attempting to fetch data
  */
    virtual result_t grabScanData(node_info *nodebuffer, size_t &count,
                                  uint32_t timeout = SDK_TIMEOUT) ;
    /*!
  * @brief Get the current scan frequency of the lidar \

  * @param[in] frequency    Scan frequency
  * @param[in] timeout      Timeout duration
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Perform this action only after stopping the scan
  */
    virtual result_t getScanFrequency(scan_frequency &frequency,
                                      uint32_t timeout = SDK_TIMEOUT);

    /*!
  * @brief Get the current sampling rate of the lidar \

  * @param[in] frequency    Sampling rate
  * @param[in] timeout      Timeout duration
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Perform this action only after stopping the scan
  */
    virtual result_t getSamplingRate(
            sampling_rate &rate,
            uint32_t timeout = SDK_TIMEOUT);

    /*!
  * @brief Set the current sampling rate of the lidar \

  * @param[in] rate         Sampling rate
  * @param[in] timeout      Timeout duration
  * @return Execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Perform this action only after stopping the scan
  */
    virtual result_t setSamplingRate(
            sampling_rate &rate,
            uint32_t timeout = SDK_TIMEOUT);

private:
    //Connect via TCP
    bool configConnect(const char *lidarIP, int tcpPort = 8090);
    //Disconnect TCP
    void configDisconnect();
    //Connect via UDP
    bool dataConnect(const char *lidarIP, int localPort = 8000);
    //Disconnect UDP
    void dataDisconnect();
    //Start scanning
    virtual result_t startScan(
            bool force = false,
            uint32_t timeout = SDK_TIMEOUT);
    //Stop scanning
    virtual result_t stopScan(
            uint32_t timeout = SDK_TIMEOUT);
    //Create thread
    bool createThread();
    //Destroy thread
    void deleteThread();
    //Start automatic scanning
    result_t startAutoScan(
            bool force = false,
            uint32_t timeout = SDK_TIMEOUT);
    //Stop automatic scanning
    result_t stopAutoScan(
            uint32_t timeout = SDK_TIMEOUT);

    result_t getScanData(node_info* nodes, size_t& count);
    //Check automatic reconnection
    result_t checkAutoConnecting();

    //Worker thread for parsing scan data
    int parseScanDataThread();
    //Worker thread for periodically fetching speed information
    int parseParamInfoThread();

    //Set multiple parameters
    bool setParams(const cJSON* json);
    //Set a single parameter
    bool setParam(const std::string& key, const float& value);
    bool setParam(cJSON* json);
    //Fetch multiple parameters
    bool getParams(const std::string& key, cJSON* &value);
    //Send data
    bool sendData(cJSON* json);
    //Wait for response
    bool waitResp(cJSON* &json, uint32_t timeout = SDK_TIMEOUT);

private:
    //TIA-X angle and distance correction
    bool correct(float& a, float& d);

private:
    int m_model = YDLIDAR_TIA; //Lidar model
    float m_lastAngle = 0.f;
    int m_port2 = 9000;
    /* Sockets for ydlidar */
    CActiveSocket *socket_cmd = nullptr;
    CPassiveSocket *socket_data = nullptr;
    uint8_t m_buff[TIA_PACKMAXBUFFS2]; //Buffer
    uint64_t lastZeroTime = 0; //Timestamp of the previous zero-position point
    uint8_t lastPackIndex = 0; //Packet index of the previous frame
    //TIA-X specific parameters
    EaiTiaXParam m_param; //Parameters
    Thread _thread2; //Parameter thread
    std::thread *m_thread2 = nullptr;  // STD线程对象

    Locker _lock2; //Mutex for serial or network operations (non-recursive)
};

} //ydlidar
