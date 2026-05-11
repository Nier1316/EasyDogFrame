#ifndef COMMON_H_
#define COMMON_H_

#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <stdio.h>

#ifdef WIN32
#define snprintf _snprintf
#endif


#define cc_count_of(arr) (sizeof(arr)/sizeof(arr[0]))
#define cc_max(a, b) ((a) > (b) ? (a) : (b))
#define cc_min(a, b) ((a) < (b) ? (a) : (b))

#define cc_sleep(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

// Set the minimum timer resolution for an application or device driver. 
void cc_time_begin_period(int ms);
// Clear a previously set minimum timer resolution. 
void cc_time_end_period(int ms);

std::string cc_get_hex_string(const unsigned char* data, unsigned int len, bool space = true);
int cc_parse_hex_string(const std::string &str, unsigned char* data, unsigned int size);

std::vector<std::string> cc_split(const std::string &str, const std::string &delim);
std::string cc_combine(const std::vector<std::string> &items, const std::string &delim);

// The dir path of dll or exe file 
std::string cc_get_module_file_path();

// timer
class cc_timer
{
public:
    cc_timer() {
        restart();
    }
    void restart() {
        m_start = std::chrono::steady_clock::now();
    }
    int elapse_ms() {
        auto now = std::chrono::steady_clock::now();
        return (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start).count();
    }
private:
    std::chrono::steady_clock::time_point m_start;
};


#endif


