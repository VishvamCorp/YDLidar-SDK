/**
 * @file neo_logger.hpp
 * @brief Logger class implementation for the NeoProcessor project
 *
 * @version 0.1
 * @author Nikita Bulaev, Grovety Inc, agency@grovety.com
 */

#include "logger.hpp"

#include <atomic>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>

using namespace ydlidar::core::Log;

namespace ydlidar::core::Log {
struct LogMessage {
    std::chrono::milliseconds ts;       /*!< Time of the message */
    enum Facility             facility; /*!< Facility level */
    std::string               ident;    /*!< Identifier */
    std::string               msg;      /*!< Message itself */
};

static std::mutex        cntMutex;         /*!< Mutex for the logger counter */
static int               loggerCnt = 0;    /*!< Counter for the number of loggers (made for later use, Syslogger for example) */
static std::atomic<bool> run_tasks {true}; /*!< Flag to run the tasks */

static std::mutex             logQueueMutex; /*!< Mutex for the log queue */
static std::deque<LogMessage> logQueue;      /*!< Queue for the log messages */
static std::thread            outputThread;  /*!< Thread for the output */

// condition_variable for the log queue
static std::mutex              logQueueCVMutex;
static std::condition_variable logQueueCV;

static LogCallback callback = nullptr;

static void outputTask(void);
static void pushMessage(std::chrono::milliseconds ts, enum Facility facility, const std::string &ident, const std::string &msg);
};  // namespace ydlidar::core::Log

Logger::Logger(const std::string &ident, enum Facility maxFacility) : _ident(ident), _maxFacility(maxFacility) {
    std::lock_guard<std::mutex> lock(cntMutex);
    loggerCnt++;

    if (loggerCnt == 1) {
        run_tasks.store(true, std::memory_order_release);

        // Start output thread
        if (!outputThread.joinable()) {
            outputThread = std::thread([]() {
                outputTask();
            });

            auto handle = outputThread.native_handle();
            pthread_setname_np(handle, "LoggerOutputTask");
        }

        // Possible future use: init Syslog here if cnt == 1
    }
}

Logger::~Logger() {
    std::lock_guard<std::mutex> lock(cntMutex);
    loggerCnt--;

    if (loggerCnt == 0) {
        run_tasks.store(false, std::memory_order_release);
        logQueueCV.notify_all();

        // Stop output thread
        if (outputThread.joinable()) {
            outputThread.join();
        }

        // Possible future use: close Syslog here if cnt == 0
    }
}

void Logger::msg(enum Facility facility, const std::string &msg) {
    if (facility > _maxFacility) {
        return;
    }

    const auto now         = std::chrono::system_clock::now();
    const auto currentTime = std::chrono::system_clock::to_time_t(now);

    pushMessage(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()), facility, _ident, msg);
}

void Logger::emerg(const std::string &message) {
    msg(Facility::EMERG, message);
}

void Logger::alert(const std::string &message) {
    msg(Facility::ALERT, message);
}

void Logger::critical(const std::string &message) {
    msg(Facility::CRIT, message);
}

void Logger::error(const std::string &message) {
    msg(Facility::ERR, message);
}

void Logger::warning(const std::string &message) {
    msg(Facility::WARNING, message);
}

void Logger::notice(const std::string &message) {
    msg(Facility::NOTICE, message);
}

void Logger::info(const std::string &message) {
    msg(Facility::INFO, message);
}

void Logger::debug(const std::string &message) {
    msg(Facility::DEBUG, message);
}

void Logger::setFacility(Facility facility) {
    _maxFacility = facility;
}

void Logger::setCallback(LogCallback callback) {
    ydlidar::core::Log::callback = callback;
}

const std::string Logger::facilityToColor(Facility facility) {
    switch (facility) {
        case EMERG:          /* system is unusable */
            return "1;91m";  // Bold High Intensty Red

        case ALERT:          /* action must be taken immediately */
            return "0;91m";  // High Intensty Red

        case CRIT:           /* critical conditions */
        case ERR:            /* error conditions */
            return "0;31m";  // Red

        case WARNING:        /* warning conditions */
            return "0;93m";  // High Intensty Yellow

        case NOTICE:         /* normal but significant condition */
            return "0;35m";  // Purple

        case INFO:           /* informational */
            return "0;32m";  // Green

        case DEBUG:          /* debug-level messages */
            return "0;34m";  // Blue

        default:
            return "0m";  // White
    }
}

const std::string Logger::facilityToString(Facility facility) {
    switch (facility) {
        case EMERG:
            return "\033[1m EMERG\033[0m";
        case ALERT:
            return "\033[1m ALERT\033[0m";
        case CRIT:
            return "\033[1m  CRIT\033[0m";
        case ERR:
            return "\033[1m ERROR\033[0m";
        case WARNING:
            return "\033[1m  WARN\033[0m";
        case NOTICE:
            return "NOTICE";
        case INFO:
            return "  INFO";
        case DEBUG:
            return " DEBUG";

        default:
            return "unknown";
    }
}

namespace ydlidar::core::Log {
void outputTask(void) {
    try {
        for (;;) {
            std::deque<LogMessage> messages;
            {
                std::unique_lock<std::mutex> lock(logQueueMutex);

                logQueueCV.wait_for(lock, std::chrono::seconds(1), [] {
                    return !logQueue.empty() || !run_tasks.load(std::memory_order_acquire);
                });

                if (!run_tasks.load(std::memory_order_acquire)) {
                    break;
                }

                if (logQueue.empty()) {
                    continue;
                }

                messages = std::move(logQueue);
            }

            for (const auto &msg : messages) {
                std::stringstream output;

                if (callback != nullptr) {
                    output << "[" << msg.ident << "] " << msg.msg;
                    callback(msg.facility, output.str());

                    continue;
                }

                auto colorCode    = Logger::facilityToColor(msg.facility);
                auto facilityName = Logger::facilityToString(msg.facility);

                auto ts           = std::chrono::system_clock::to_time_t(std::chrono::system_clock::time_point(msg.ts));
                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(msg.ts).count() % 1000;

                std::tm timeInfo = *std::localtime(&ts);
                output << "\033[0;37m[" << std::put_time(&timeInfo, "%F %T") << "." << std::setfill('0') << std::setw(3) << milliseconds
                       << "] \033[0m"
                       << "\033[" << colorCode << facilityName << "\033[0m "
                       << "\033[1;90m[" << msg.ident << "]\033[0m " << msg.msg << std::endl;

                if (msg.facility < Facility::NOTICE) {
                    std::cout << output.str();
                } else {
                    std::cerr << output.str();
                }
            }
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "outputTask fatal: %s\n", e.what());
        std::fflush(stderr);
        std::terminate();  // чтобы поведение оставалось тем же
    } catch (...) {
        std::fprintf(stderr, "outputTask fatal: unknown\n");
        std::fflush(stderr);
        std::terminate();
    }
}

/**
 * @brief Push message to the log queue
 *
 * @param[in] ts       Time of the message
 * @param[in] facility Facility level
 * @param[in] ident    Identifier
 * @param[in] msg      Message itself
 */
void pushMessage(std::chrono::milliseconds ts, Facility facility, const std::string &ident, const std::string &msg) {
    {
        std::lock_guard<std::mutex> lock(logQueueMutex);
        logQueue.push_back({ts, facility, ident, msg});
    }

    logQueueCV.notify_one();
}
}  // namespace ydlidar::core::Log
