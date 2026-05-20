#ifndef LOG_H_
#define LOG_H_

#include <stdio.h>
#include <string>
#include <mutex>
             
#define LOG_FILE_MAX_SIZE           (50*1000*1000)

class CLog
{
    friend CLog* TheLog();
private:
    CLog();
public:
    ~CLog();

    // level: 0-info, 1-error
    void Write(int level, const char* fmt, ...);

    // mode: a+ may create file, r+ file must exist
    void SetFileName(const char* filename, const char* mode = "a+");

private:
    bool Open();
    void Close();

private:
    FILE *m_fp = nullptr;
    std::string m_filepath = "log.txt";
    std::string m_fileopenmode = "a+";
    std::recursive_mutex m_mutex;
};

CLog* TheLog();

#ifdef _DEBUG
#   define LOG_CONSOLE(fmt, ...) \
{ \
    printf(fmt, ##__VA_ARGS__); \
    if (fmt[strlen(fmt)-1] != '\n') { \
        printf("\n"); \
    } \
}
#else
#   define LOG_CONSOLE(fmt, ...)
#endif

#define LOG_INFO(fmt, ...)  TheLog()->Write(0, fmt, ##__VA_ARGS__); LOG_CONSOLE(fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)   TheLog()->Write(1, fmt, ##__VA_ARGS__); LOG_CONSOLE(fmt, ##__VA_ARGS__)
 


#endif


