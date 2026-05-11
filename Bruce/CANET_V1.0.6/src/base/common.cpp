#include "common.h"
#include <string.h>
#include <regex>

#ifdef WIN32
#include <windows.h>
#endif

void cc_time_begin_period(int ms)
{
#ifdef WIN32
	timeBeginPeriod(ms);
#endif
}

void cc_time_end_period(int ms)
{
#ifdef WIN32	
	timeEndPeriod(ms);
#endif
}

std::string cc_get_hex_string(const unsigned char* data, unsigned int len, bool space /*= true*/)
{
	std::string str;
	char tmp[10];
	for (unsigned int i=0; i<len; i++) {
		memset(tmp, 0, sizeof(tmp));
		snprintf(tmp, sizeof(tmp), "%02X", data[i]);	
		
        if (space && !str.empty()) {
			str += " ";
		}
		str += tmp;
	}
	return str;
}

int cc_parse_hex_string(const std::string &str, unsigned char* data, unsigned int size)
{
	int count = 0;
	const char separate = ' ';
	unsigned int pos = 0;
	while (pos < str.length()) {
		if (str.at(pos) == separate) {
			pos++;
		}
		else if (!isxdigit(str.at(pos))) {
			return -1;
		}
		else {
			unsigned int byteEnd = pos + 1;
			while (byteEnd < str.length()) {
				if (str.at(byteEnd) == separate) {
					break;
				}
				else {
					byteEnd++;
				}
			}
			if (byteEnd - pos > 2) {
				return -1;
			}
			char tmp[3] = {0,0,0};
			memcpy(tmp, &str.at(pos), byteEnd - pos);
			data[count++] = (unsigned char)strtol(tmp, nullptr, 16);
			pos = byteEnd;
		}
	}

	return count;
}

std::vector<std::string> cc_split(const std::string &str, const std::string &delim)
{
    if (str.empty()) {
        return std::vector<std::string>();
    }

	std::regex re(delim);
	if (delim == ".") {
		re = "\\.";
	}

	return std::vector<std::string> (
		std::sregex_token_iterator(str.begin(), str.end(), re, -1),
		std::sregex_token_iterator()
	);
}

std::string cc_combine(const std::vector<std::string> &items, const std::string &delim)
{
	std::string str;
	for (auto item : items) {
		if (!str.empty()) {
			str += delim;
		}
		str += item;
	}
	return str;
}

std::string cc_get_module_file_path()
{
#ifdef WIN32
    char sDrive[_MAX_DRIVE];
    char sDir[_MAX_DIR];
    char sFilename[_MAX_FNAME], Filename[_MAX_FNAME];
    char sExt[_MAX_EXT];

    GetModuleFileNameA(GetModuleHandleA(NULL), Filename, _MAX_PATH);
    _splitpath_s(Filename, sDrive, sDir, sFilename, sExt);

    std::string strRet = sDrive;
    strRet += sDir;
    return strRet;
#else
    return "";
#endif
}
