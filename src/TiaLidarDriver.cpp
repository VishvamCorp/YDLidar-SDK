#include <stdio.h>
#include <errno.h>
#include <algorithm>
#include "core/network/PassiveSocket.h"
#include "core/network/SimpleSocket.h"
#include "core/serial/common.h"
#include "core/common/ydlidar_help.h"
#include "TiaLidarDriver.h"

//TIA lidar command keywords
#define P_TIA_SCANTYPE "scanType" //Start/stop scanning
#define P_TIA_SCANFREQ "target_speed_addr" //Rotation speed (register value)
#define P_TIA_SAMPLERATE "measure_prf_addr" //Sampling rate (register value)
#define P_TIA_READ "Read" //Read
#define P_TIA_CONFIG "config" //Configure
#define P_TIA_SCANFREQ2 "motorSpeed" //Rotation speed (queried value)
#define P_TIA_SAMPLERATE2 "samplerate" //Sampling rate (queried value)
#define P_TIA_MODEL "model" //Model (queried value)

using namespace ydlidar;
using namespace ydlidar::core;
using namespace ydlidar::core::network;
using namespace ydlidar::core::base;
using namespace ydlidar::core::common;


TiaLidarDriver::TiaLidarDriver()
{
    socket_cmd = new core::network::CActiveSocket(CSimpleSocket::SocketTypeTcp);
    socket_data = new core::network::CPassiveSocket(CSimpleSocket::SocketTypeUdp);
    socket_data->SetSocketType(CSimpleSocket::SocketTypeUdp);
    socket_cmd->SetConnectTimeout(DEFAULT_CONNECTION_TIMEOUT_SEC,
                                  DEFAULT_CONNECTION_TIMEOUT_USEC);
    scan_node_buf = new node_info[LIDAR_MAXNODES];
    scan_node_count = 0;
    nodeIndex = 0;
    retryCount = 0;
    isAutoReconnect = true;
    isAutoconnting = false;
    m_Debug = false;
    m_SupportMotorDtrCtrl = true;

    m_isScanning = false;
    m_isConnected = false;
    m_port = "192.168.0.11";
    m_baudrate = 8000;
    m_SingleChannel = false;
    m_LidarType = TYPE_TIA;
    m_PointTime = 1e9 / 20000;
}

TiaLidarDriver::~TiaLidarDriver()
{
    disconnect();

    ScopedLocker l2(_lock2);
    if (socket_data) {
        delete socket_data;
        socket_data = NULL;
    }
    ScopedLocker l(_lock);
    if (socket_cmd) {
        delete socket_cmd;
        socket_cmd = NULL;
    }

    if (scan_node_buf) {
        delete[] scan_node_buf;
        scan_node_buf = nullptr;
    }
}

result_t TiaLidarDriver::connect(const char *ip, uint32_t port)
{
    m_port = ip;
    m_port2 = port;
    m_isConnected = false;

    if (!configConnect(m_port.c_str(), m_port2))
    {
        error("Fail to connect TCP [%s:%d],Error [%s]",
                     m_port.c_str(), m_port2,
                     DescribeError(true));
        return RESULT_FAIL;
    }
    info("Connect TCP [%s:%d]", m_port.c_str(), m_port2);

    if (!dataConnect(m_port.c_str(), m_baudrate))
    {
        error("Fail to connect UDP [%s:%d],Error [%s]",
            m_port.c_str(), m_baudrate,
            DescribeError(false));
        configDisconnect();
        return RESULT_FAIL;
    }
    info("Connect UDP [%s:%d]", m_port.c_str(), m_baudrate);

    //Set rotation speed
    setParam(P_TIA_SCANFREQ2, double(m_ScanFreq));
    //Set sampling rate
    setParam(P_TIA_SAMPLERATE2, double(m_SampleRate));

    m_isConnected = true;
    return RESULT_OK;
}

bool TiaLidarDriver::isconnected() const {
    return m_isConnected;
}

bool TiaLidarDriver::isscanning() const {
    return m_isScanning;
}

void TiaLidarDriver::setIntensities(const int &i) {
    m_intensities = i;
}

void TiaLidarDriver::setAutoReconnect(const bool &enable) {
    isAutoReconnect = enable;
}

const char *TiaLidarDriver::DescribeError(bool isTCP)
{
    if (isTCP)
    {
        ScopedLocker lock(_lock);
        return socket_cmd != NULL ? socket_cmd->DescribeError() : "NO Socket";
    }
    else
    {
        ScopedLocker lock(_lock2);
        return socket_data != NULL ? socket_data->DescribeError() : "NO Socket";
    }
}

bool TiaLidarDriver::configConnect(const char *ip, int port)
{
    ScopedLocker lock(_lock);

    if (!socket_cmd)
        return false;
    if (!socket_cmd->IsSocketValid()) {
        if (!socket_cmd->Initialize()) {
            return false;
        }
    } else {
        return socket_cmd->IsSocketValid();
    }

//    socket_cmd->SetNonblocking();
    socket_cmd->bindport(ip, port);
    if (!socket_cmd->open())
//    if (!socket_cmd->Open(ip, port)) //This approach fails
    {
        socket_cmd->Close();
        return false;
    }

    socket_cmd->SetReceiveTimeout(
                SDK_TIMEOUT / 1000,
                (SDK_TIMEOUT % 1000) * 1000);
    socket_cmd->SetSendTimeout(
                SDK_TIMEOUT / 1000,
                (SDK_TIMEOUT % 1000) * 1000);
    socket_cmd->SetBlocking();

    return socket_cmd->IsSocketValid();
}

void TiaLidarDriver::configDisconnect()
{
    ScopedLocker lock(_lock);
    if (!socket_cmd)
        return;
    if (socket_cmd->IsSocketValid())
    {
        socket_cmd->Close();
        info("Disconnect TCP");
    }
}

void TiaLidarDriver::disconnect()
{
    stopScan();

    dataDisconnect();
    configDisconnect();
}

result_t TiaLidarDriver::getHealth(device_health &health, uint32_t timeout)
{
    UNUSED(timeout);
    health.error_code = 0;
    health.status = 0;
    return RESULT_OK;
}

result_t TiaLidarDriver::getDeviceInfo(device_info &di, uint32_t timeout)
{
    UNUSED(timeout);

    result_t ret = RESULT_FAIL;

    //Set default values
    m_model = YDLIDAR_TIA;

    //Obtain device information
    cJSON *o = nullptr;
    if (getParams(P_TIA_CONFIG, o))
    {
        //Lidar model
        cJSON *jm = cJSON_GetObjectItem(o, P_TIA_MODEL);
        if (cJSON_IsNumber(jm))
        {
            m_model = jm->valueint; //Model number
            ret = RESULT_OK;
        }
    }
    cJSON_Delete(o);

    di.firmware_version = 0;
    di.hardware_version = 0;
    di.model = m_model;

//    debug("TIA model[%d]", m_model);

    return ret;
}

// result_t TiaLidarDriver::start(bool force, uint32_t timeout)
// {
//     UNUSED(force);
//     UNUSED(timeout);

//     result_t ret = RESULT_FAIL;

//     if (m_isScanning)
//         return RESULT_OK;

//     if (!IS_OK(startScan()) &&
//         !IS_OK(startScan()))
//         return RESULT_FAIL;

//     ret = createThread();

//     return ret;
// }

// result_t TiaLidarDriver::stop()
// {
//     //Mark flag to prevent automatic reconnection
//     if (isAutoconnting)
//         isAutoconnting = false;

//     //If scanning is not enabled, return immediately
//     if (!isscanning())
//         return RESULT_OK;
//     m_isScanning = false;

//     deleteThread();

//     if (!IS_OK(stopScan()) &&
//         !IS_OK(stopScan()))
//         return RESULT_FAIL;

//     return RESULT_OK;
// }

bool TiaLidarDriver::createThread()
{
    m_thread = std::thread(&TiaLidarDriver::parseScanDataThread, this);
    if (!m_thread.joinable())
    {
      error("Fail to create data thread");
      return false;
    }

    info("Create TIA thread 0x%X", m_thread.get_id());

    // m_thread2 = new std::thread(&TiaLidarDriver::parseParamInfoThread, this);
    // if (!m_thread2)
    // {
    //   warn("Fail to create TIA parameter thread");
    // }

    return true;
}

result_t TiaLidarDriver::startScan(bool force, uint32_t timeout)
{
    UNUSED(force);
    UNUSED(timeout);

    bool ret = setParam(P_TIA_SCANTYPE, 0);

    ret &= createThread();

    return ret ? RESULT_OK : RESULT_FAIL;
}

result_t TiaLidarDriver::stopScan(uint32_t timeout)
{
    UNUSED(timeout);

    //Mark flag to prevent automatic reconnection
    if (isAutoconnting)
        isAutoconnting = false;
    //If scanning is not enabled, return immediately
    if (!isscanning())
        return RESULT_OK;
    m_isScanning = false;

    deleteThread();

    //Stop scanning
    bool ret = setParam(P_TIA_SCANTYPE, -1);

    return ret ? RESULT_OK : RESULT_FAIL;
}

result_t TiaLidarDriver::startAutoScan(bool force, uint32_t timeout)
{
    UNUSED(force);
    UNUSED(timeout);

    bool ret = setParam(P_TIA_SCANTYPE, 0);
    return ret ? RESULT_OK : RESULT_FAIL;
}

result_t TiaLidarDriver::stopAutoScan(uint32_t timeout)
{
    UNUSED(timeout);

    //Stop scanning
    bool ret = setParam(P_TIA_SCANTYPE, -1);
    return ret ? RESULT_OK : RESULT_FAIL;
}

bool TiaLidarDriver::dataConnect(const char *ip, int port)
{
    UNUSED(ip);

    ScopedLocker l(_lock2);
    if (!socket_data)
        return false;

    if (!socket_data->IsSocketValid())
    {
        if (socket_data->Initialize())
        {
            if (!socket_data->Listen(NULL, port)) //NULL
            {
                socket_data->Close();
                return false;
            }

            socket_data->SetReceiveTimeout(SDK_TIMEOUT / 1000,
                (SDK_TIMEOUT % 1000) * 1000);
        }
    }

    return socket_data->IsSocketValid();
}

void TiaLidarDriver::dataDisconnect()
{
    ScopedLocker l(_lock2);
    if (!socket_data)
        return;
    if (socket_data->IsSocketValid())
    {
        socket_data->Close();
        info("Disconnect UDP");
    }
}

void TiaLidarDriver::deleteThread()
{
    if (m_isScanning)
    {
        m_isScanning = false;
        ScopedLocker l(_lock);
        _dataEvent.set();
    }

    if (m_thread.joinable()) {
        m_thread.join();
        m_thread = std::thread();
    }

    if (m_thread2)
    {
      if (m_thread2->joinable())
        m_thread2->join();
      delete m_thread2;
      m_thread2 = nullptr;
    }
}

result_t TiaLidarDriver::grabScanData(
        node_info *nodebuffer,
        size_t &count,
        uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    switch (_dataEvent.wait(timeout))
    {
    case Event::EVENT_TIMEOUT:
        count = 0;
        return RESULT_TIMEOUT;

    case Event::EVENT_OK:
    {
        if (scan_node_count == 0)
        {
            ret = RESULT_FAIL;
        }
        else
        {
            ScopedLocker l(_lock);
            size_t size_to_copy = std::min(count, scan_node_count);
            memcpy(nodebuffer, scan_node_buf, size_to_copy * sizeof(node_info));
            count = size_to_copy;
            scan_node_count = 0;
            ret = RESULT_OK;
        }
        _dataEvent.set(false); //Reset event state
        break;
    }
    default:
        count = 0;
        return RESULT_FAIL;
    }

    return ret;
}

result_t TiaLidarDriver::getScanFrequency(
        scan_frequency &frequency,
        uint32_t timeout)
{
    UNUSED(frequency);
    UNUSED(timeout);

    result_t ans = RESULT_FAIL;
    return ans;
}

result_t TiaLidarDriver::getSamplingRate(
        sampling_rate &rate,
        uint32_t timeout)
{
    UNUSED(rate);
    UNUSED(timeout);
    return RESULT_FAIL;
}

result_t TiaLidarDriver::setSamplingRate(
        sampling_rate &rate,
        uint32_t timeout)
{
    UNUSED(rate);
    UNUSED(timeout);
    return RESULT_FAIL;
}

result_t TiaLidarDriver::checkAutoConnecting()
{
    result_t ans = RESULT_FAIL;
    isAutoconnting = true; //Mark as reconnecting
    if (m_driverErrno != BlockError)
      setDriverError(TimeoutError);

    while (isAutoReconnect && isscanning())
    {
        //Disconnect first
        {
            configDisconnect();
            dataDisconnect();
        }
        //Reconnect
        while (isAutoconnting &&
            !IS_OK(connect(m_port.c_str(), m_port2)))
        {
            setDriverError(NotOpenError);
            delay(300); //Delay
        }
        if (!isAutoconnting)
        {
            m_isScanning = false;
            return RESULT_FAIL;
        }
        //Restart the lidar
        if (isconnected() &&
            isAutoconnting)
        {
            stopAutoScan(); //Attempt to stop scanning
            ans = startAutoScan();
            if (IS_OK(ans))
            {
                if (getDriverError() == DeviceNotFoundError)
                    setDriverError(NoError);
                isAutoconnting = false;
                return ans;
            }
            else
            {
                setDriverError(DeviceNotFoundError);
                delay(300); //Delay
            }
        }
    }

    isAutoconnting = false;
    setDriverError(NoError);
    return RESULT_FAIL;
}

int TiaLidarDriver::parseScanDataThread()
{
    node_info      local_scan[LIDAR_MAXNODES];
    node_info      local_buf[TIA_PACKMAXNODES];
    size_t         count = TIA_PACKMAXNODES;
    size_t         scan_count = 0;
    result_t       ans = RESULT_FAIL;
    int timeout_count = 0;

    memset(local_scan, 0, sizeof(local_scan));
    lastZeroTime = getms();
    lastPackIndex = 0;

    m_isScanning = true;

    while (m_isScanning)
    {
        count = 0;
        memset(local_buf, 0, sizeof(local_buf));
        ans = getScanData(local_buf, count);
        if (!IS_OK(ans))
        {
            if (timeout_count > DEFAULT_TIMEOUT_COUNT)
            {
                if (!isAutoReconnect)
                {
                    error("Exit scanning thread!");
                    m_isScanning = false;
                    return RESULT_FAIL;
                }
                else
                {
                    ans = checkAutoConnecting(); //Reconnect
                    if (IS_OK(ans)) {
                        timeout_count = 0;
                        local_scan[0].sync = NODE_UNSYNC;
                    } else {
                        m_isScanning = false;
                        return RESULT_FAIL;
                    }
                }
            }
            else
            {
                timeout_count ++;
                local_scan[0].sync = NODE_UNSYNC;
                error("Timout count [%d]", timeout_count);
            }
        }
        else
        {
            timeout_count = 0;

            for (size_t i = 0; i < count; ++i)
            {
                if (NODE_SYNC == local_buf[i].sync)
                {
                    {
                        ScopedLocker l(_lock);
                        memcpy(scan_node_buf, local_scan, scan_count * SDKNODESIZE);
                        scan_node_count = scan_count;
                        _dataEvent.set();
                    }
                    scan_count = 0;
                }

                local_scan[scan_count++] = local_buf[i];
                if (scan_count >= LIDAR_MAXNODES)
                    scan_count = 0;
            }
        }
    }

    m_isScanning = false;

    return RESULT_OK;
}

int TiaLidarDriver::parseParamInfoThread()
{
    //Periodically fetch rotation speed and sampling rate information
    while (m_isScanning)
    {
        {
            cJSON *o = nullptr;
            if (getParams(P_TIA_CONFIG, o))
            {
                cJSON *sf = cJSON_GetObjectItem(o, P_TIA_SCANFREQ2);
                if (cJSON_IsNumber(sf))
                    m_param.scanFreq = sf->valuedouble / 100;
                cJSON *sr = cJSON_GetObjectItem(o, P_TIA_SAMPLERATE2);
                if (cJSON_IsNumber(sr))
                    m_param.sampleRate = sr->valuedouble / 100;
                debug("ScanFreq: %f SampleRate: %f",
                    m_param.scanFreq, m_param.sampleRate);
            }
            cJSON_Delete(o);
        }
        int count = 0;
        while (count++ < 4 && m_isScanning)
            delay(TIMEOUT_500);
    }

    return RESULT_OK;
}

bool TiaLidarDriver::setParams(const cJSON* json)
{
    bool ret = true;
    if (json->child)
    {
        //Iterate through all values
        cJSON *child = json->child;
        while (child)
        {
            cJSON *o = cJSON_CreateObject();
            cJSON_AddItemToObject(o, child->string, child);
            ret = setParam(o);
            if (!ret)
                break;
            child = child->next;
        }
    }

    return ret;
}

bool TiaLidarDriver::setParam(
    const std::string &key,
    const float &value)
{
    cJSON *json = cJSON_CreateObject();

    if (P_TIA_SCANFREQ == key) //Set register value
    {
        //390625 / setting value = rotation speed (Hz)
        float v = value;
        if (v > 0 && v <= 200)
            cJSON_AddNumberToObject(json, key.data(), int(390625 / v));
        else
            warn("TIA scanning frequency [%.02fHz] setting is incorrect", v);
    }
    else if (P_TIA_SAMPLERATE == key) //Set register value
    {
        //100000000 / setting value = sampling rate (K/s)
        float v = value;
        if (v > 0 && v <= 300)
            cJSON_AddNumberToObject(json, key.data(), int(100000 / v));
        else
            warn("TIA sampling rate [%.02fK/s] setting is incorrect", v);
    }
    else if (P_TIA_SCANFREQ2 == key)
    {
        //Rotation speed (Hz)
        int v = value;
        if (v > 0 && v <= 200)
            cJSON_AddNumberToObject(json, key.data(), v);
        else
            warn("TIA scanning frequency [%dHz] setting is incorrect", v);
    }
    else if (P_TIA_SAMPLERATE2 == key)
    {
        //100000000 / setting value = sampling rate (K/s)
        int v = value;
        if (v > 0 && v <= 300)
            cJSON_AddNumberToObject(json, key.data(), v);
        else
            warn("TIA sampling rate [%dK/s] setting is incorrect", v);
    }
    else
    {
        cJSON_AddNumberToObject(json, key.data(), value);
    }

    return setParam(json);
}

bool TiaLidarDriver::setParam(cJSON *json)
{
    bool ret = false;
    do
    {
        if (!sendData(json))
	    break;
        cJSON *js = nullptr;
        if (!waitResp(js, TIMEOUT_500))
	    break;
        ret = cJSON_Compare(json, js, cJSON_bool(0));
        cJSON_Delete(js); //Free memory
    } while (false);
    cJSON_Delete(json); //Free memory
    if (!ret)
    {
        warn("Fail to set lidar parameter,Error to response");
        return false;
    }
    return true;
}

bool TiaLidarDriver::getParams(const std::string &key, cJSON* &value)
{
    bool ret = false;
    //Assemble data to send
    cJSON* json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, P_TIA_READ, key.data());
    ret = sendData(json);
    cJSON_Delete(json); //Free memory
    if (!ret)
        return false;
   //wait for response
    if (!waitResp(value, TIMEOUT_1S))
        return false;
    return true;
}

bool TiaLidarDriver::sendData(cJSON* json)
{
    if (!json)
        return false;
    char* ss = cJSON_PrintUnformatted(json); //Commands require compact JSON format
    std::string sd(ss);
    free(ss);
    //Send data
    ScopedLocker l(_lock);
    int32_t rs = socket_cmd->Send(
        reinterpret_cast<const uint8_t*>(sd.data()), sd.size());
    if (m_Debug)
        debug("[SEND] %s", sd.c_str());
    if (rs <= 0)
    {
        warn("Fail to send TCP data");
        return false;
    }

    return true;
}

bool TiaLidarDriver::waitResp(cJSON* &json, uint32_t timeout)
{
    char buff[300] = {0};
    ScopedLocker l(_lock);
    if (socket_cmd->Select(timeout / 1000, timeout * 1000))
    {
        int32_t rs = socket_cmd->Receive(sizeof(buff),
            reinterpret_cast<uint8_t*>(buff));
        if (rs > 0) //When there is response data
        {
            std::string rd(buff, rs);
            if (m_Debug)
                debug("[RECV] %s", rd.c_str());
            //Parse JSON data
            json = cJSON_Parse(rd.data());
            if (!json)
            {
                warn("Fail to wait response,Error json format");
                return false;
            }
            return true;
        }
    }

    warn("Waiting for response timeout [%u]ms", timeout);
    return false;
}

bool TiaLidarDriver::correct(float &angle, float &dist)
{
    return m_param.left.correct(angle, dist) ||
        m_param.right.correct(angle, dist);
}

result_t TiaLidarDriver::getScanData(node_info* nodes, size_t& count)
{
    int32_t rs = 0;
    memset(m_buff, 0, TIA_PACKMAXBUFFS);
    {
        ScopedLocker l(_lock2);
        if (!socket_data->IsSocketValid())
            return RESULT_FAIL;
        rs = socket_data->Receive(sizeof(m_buff),
            reinterpret_cast<uint8_t *>(m_buff));
        if (rs <= 0)
        {
            warn("Fail to recv UDP data");
            return RESULT_TIMEOUT;
        }
        else if (rs < TIA_PACKMAXBUFFS)
        {
            warn("The packet length [%d] < the expected value [%d]",
                rs, TIA_PACKMAXBUFFS);
            return RESULT_FAIL;
        }
    }
    uint8_t* data = m_buff;
    if (m_Debug)
        debugh(m_buff, rs);

    uint8_t packIndex = 0; //Current packet index
    uint64_t stamp = 0; //Timestamp
    int idx = 0; //Data index
    //Parse timestamp and packet index
    idx = TIA_PACKMAXBUFFS - 8;
    if (YDLIDAR_TIA_H == m_model ||
        YDLIDAR_TIA_X == m_model)
    {
        if (rs >= TIA_PACKMAXBUFFS)
        {
            stamp = getBigValue(&data[idx], 4); //Timestamp
            idx += 4;
            packIndex = getBigValue(&data[idx], 1); //Packet index
            idx += 1;
        }
    }
    else //YDLIDAR_TIA == m_model
    {
        if (rs >= TIA_PACKMAXBUFFS2)
        {
            stamp = getBigValue(&data[idx], 4) * 1000; //Timestamp (seconds)
            idx += 4;
            stamp += getBigValue(&data[idx], 4); //Timestamp (milliseconds)
            stamp *= 1000000; //Convert milliseconds to nanoseconds
            idx += 4;
            packIndex = uint8_t(getBigValue(&data[idx], 1) & 0x0F); //Low 4 bits of the byte represent the packet index
            idx += 1;
        }
    }
    if (abs(packIndex - lastPackIndex) > 1 &&
        packIndex != 0 &&
        lastPackIndex != 0)
        warn("Data packet lost,current packet index [%u],last packet index [%u]",
            packIndex, lastPackIndex);
    lastPackIndex = packIndex;

    //Big-endian
    uint16_t h = 0; //Header
    uint16_t a = 0; //Angle
    uint8_t as = 0; //Angle increment
    float sa = .0;
    float ea = .0;
    uint8_t p = 0; //Intensity
    uint16_t d = 0; //Distance
    int index = 0; //Point index
    bool invalid = true; //Invalid flag
    idx = 0; //Reset position
    for (int i=0; i<TIA_PACKWIDTH; ++i)
    {
        h = getBigValue(&data[idx], 2); //Header
        idx += 2;
        if (FRAME_PREAMBLE != h)
        {
            warn("The format of the [%d] packet header is incorrect", i + 1);
            return RESULT_FAIL;
        }
        a = getBigValue(&data[idx], 2); //Angle value (scaled by 100)
        idx += 2;

        {
            if (0 == i)
                sa = a / 100.0;
            else
                ea = m_lastAngle / 100.0;
        }
        for (int j=0; j<TIA_PACKHEIGHT; ++j)
        {
            as = getBigValue(&data[idx], 1); //Angle increment
            idx += 1;
            a += (as & 0x3F);
            p = getBigValue(&data[idx], 1); //Intensity
            idx += 1;
            d = getBigValue(&data[idx], 2); //Distance
            idx += 2;
            invalid = (!as && !p && (!d || d==0xFFFF)); //All zeros indicate invalid data

            nodes[index].stamp = stamp ? stamp : getTime();
            //If the angle decreases by more than 1°, treat it as the zero position
            if (m_lastAngle - a > 1.0 * 100.0)
            {
                nodes[index].sync = NODE_SYNC;
                //Calculate rotation speed
//                nodes[index].scanFreq = 1e9 / (getTime() - lastZeroTime);
                //Assign the actual rotation speed obtained
                nodes[index].scanFreq = m_ScanFreq; //m_param.scanFreq;
                lastZeroTime = getTime();
                if (m_Debug)
                    debug("Zero time [%llu]ns", lastZeroTime);
            }
            else
            {
                nodes[index].sync = NODE_UNSYNC;
            }
            //Store the current angle value
            m_lastAngle = a;

            //Assign values
            nodes[index].index = 0;
            nodes[index].is = (as >> 6);
            nodes[index].angle = float(a) / 100.0 * 128; //Scale angle value by 64 to align with other lidars
            nodes[index].dist = d;
            nodes[index].qual = p;

            //For TIA-X, adjust angles and distances within the specified range based on mirror parameters
            if (YDLIDAR_TIA_X == m_model)
            {
                float angle = a / 100.0;
                float dist = d;
                if (correct(angle, dist))
                {
                    nodes[index].angle = angle * 128;
                    nodes[index].dist = dist;

//                    SeLog::debug("Point %d original[%.02f,%.0f] updated[%.02f,%.0f]",
//                        index, a / 100.0, float(d), angle, dist);
                }
            }

//            SeLog::debug("Point %d [a:%.02f+%.02f d:%u i:%u]",
//                index, a / 100.0, (as && 0x3F) / 100.0, d, p);

            index ++;

            //If data is invalid, stop parsing
            if (invalid)
                break;
        }
        if (invalid)
            break;
    }
    if (m_Debug)
        debug("Angle range [%.02f~%.02f] stamp [%lu]ns index [%u]",
            sa, ea, stamp, packIndex);

    count = index;

    return RESULT_OK;
}
