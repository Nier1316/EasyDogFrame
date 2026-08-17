#include "log.h"
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <memory>
#include <chrono>

#ifdef WIN32
#define snprintf _snprintf
#endif

CLog* TheLog()
{
    static CLog _inst;
    return &_inst;
}

CLog::CLog() : m_filepath("motor_control.log"), m_fileopenmode("a+"), m_fp(nullptr)
{
}

CLog::~CLog()
{
    Close();
}

void CLog::SetFileName(const char* filename, const char* mode /*= "a+"*/)
{
    m_filepath = filename;
    m_fileopenmode = mode;
    Close();
    Open();
}

bool CLog::Open()
{
    std::unique_lock<std::recursive_mutex> lk(m_mutex);

    m_fp = fopen(m_filepath.c_str(), m_fileopenmode.c_str());

    // "r+" 要求文件必须已存在，首次运行必然失败。
    // 厂商库 libCANET_TCP 用 SetFileName("CANET.log", "r+") 调进来，
    // 于是每次全新环境启动都打印一行 "The log file open failed!"。
    // 这种情况下退回 "a+" 创建文件，语义（追加写）与 r+ 一致。
    if (!m_fp && m_fileopenmode.find('r') != std::string::npos) {
        m_fp = fopen(m_filepath.c_str(), "a+");
        if (m_fp) {
            m_fileopenmode = "a+";
        }
    }

    if (!m_fp) {
        printf("[ERR] The log file open failed: %s (mode %s)\n",
               m_filepath.c_str(), m_fileopenmode.c_str());
        return false;
    }

    Write(0, "================= Logging =================");
    return true;
}

void CLog::Close()
{
    if (m_fp) {
        fflush(m_fp);
        fclose(m_fp);
        m_fp = nullptr;
    }
}

void CLog::Write(int level, const char* fmt, ...)
{
    if (!m_fp) {
        return;
    }

    std::unique_lock<std::recursive_mutex> lk(m_mutex);

    // limit file size 
    fseek(m_fp, 0, SEEK_END);
    long len = ftell(m_fp);
    if (len > LOG_FILE_MAX_SIZE) {
        fclose(m_fp);
        m_fp = fopen(m_filepath.c_str(), "w");
        if (!m_fp) {
            printf("[ERR] Clear the log file failed! \n");
        }
    }

    char tmbuf[100];
    memset(tmbuf, 0, sizeof(tmbuf));
    time_t t = time(0);
    tm* local = localtime(&t);
	int ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000;
	snprintf(tmbuf, sizeof(tmbuf), "[%04d-%02d-%02d %02d:%02d:%02d.%03d] ",
		local->tm_year + 1900, local->tm_mon + 1, local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, ms);
    fprintf(m_fp, "%s", tmbuf);

    fprintf(m_fp, level == 1 ? "[ERR] " : "");

    va_list args;
    va_start(args, fmt);

    vfprintf(m_fp, fmt, args);
    va_end(args);

    // 补换行：空文件时 fseek(-1, SEEK_END) 会失败，此时不必补
    if (fseek(m_fp, -1, SEEK_END) == 0) {
        char c = (char)fgetc(m_fp);
        if (c != '\n') {
            fseek(m_fp, 0, SEEK_END);
            fprintf(m_fp, "\n");
        }
    }
    fflush(m_fp);
}







