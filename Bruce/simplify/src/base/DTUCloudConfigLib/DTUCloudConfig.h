#ifndef DTU_CLOUD_CONFIG_H_
#define DTU_CLOUD_CONFIG_H_

typedef void * DEVICE_HANDLE;      ///< 设备句柄
#define INVALID_DEVICE_HANDLE 0    ///< 无效设备句柄

typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;

#define ZCLOUD_MAX_DEVICES          100
#define ZCLOUD_FILENAME_MAX_LEN     260
#define ZCLOUD_FILELIST_MAX_COUNT   100



///< 结构体按1字节对齐
#pragma pack(push,1)

///< 错误代码
typedef enum _ZCLOUD_ERR_CODE
{
    ZC_ERR_SUCCESS = 0,                     ///< 成功
    ZC_ERR_FAILURE = 1,                     ///< 失败
    ZC_ERR_HTTPS_COMM = 2,                  ///< HTTPS通讯失败
    ZC_ERR_USER_LOGIN = 3,                  ///< 用户登录失败
    ZC_ERR_MQTT_CONNECT = 4,                ///< mqtt 连接服务器失败
    ZC_ERR_NO_DEVICE = 5,                   ///< 没有设备
    ZC_ERR_DEV_NOT_OPEN = 6,                ///< 设备未打开
    ZC_ERR_DEV_ID_NOT_VALID = 7,            ///< 设备ID无效
    ZC_ERR_CMD_DEVCTRL_UNKOWN = 8,          ///< 不支持的命令/DEVCTRL
    ZC_ERR_ARG_INVALID = 9,                 ///< 执行cmd命令/DevCtrl的arg参数无效
    ZC_ERR_CMD_DATA_BUFFER_TOO_SMALL = 10,  ///< ZCLOUD_CMD_DATA中申请的缓冲区太小，不够存放请求的数据
    ZC_ERR_DEV_RESPONSE_TIMEOUT = 11,       ///< 执行需要设备响应的命令，设备响应超时
    ZC_ERR_DEV_RESPONSE_FAILED = 12,        ///< 执行需要设备响应的命令，接收到响应，但是响应信息表示失败
    ZC_ERR_DEV_CONFIG_FAILED = 13,          ///< 设备配置失败
    ZC_ERR_DEV_NOT_ONLINE = 14,             ///< 设备当前不在线
    ZC_ERR_FILE_OPEN_FAILED = 15,           ///< 文件打开失败
    ZC_ERR_FILE_SAVE_FAILED = 16,           ///< 文件保存失败
    ZC_ERR_FILE_SIZE = 17,                  ///< 文件大小异常，为0或者超出限制
    ZC_ERR_DEV_BUSY = 18,                   ///< 设备正忙，当前有正在处理的事务，无法响应
    ZC_ERR_INVALID_FIRMWARE = 19,           ///< 固件无效
    ZC_ERR_USER_CANCEL = 20,                ///< 回调用户取消
    ZC_ERR_DEV_ADD_FAILED = 21,             ///< 添加设备失败
    ZC_ERR_DEV_ALREADY_ADDED = 22,          ///< 添加设备失败，设备已经添加到账户
    ZC_ERR_FILE_CHECKSUM_MISMATCH = 23,     ///< 下载文件MD5验证失败
}ZCLOUD_ERR_CODE;

///< 通用命令代码
typedef enum _ZCloud_CMD
{
    ZC_CMD_GET_CONFIG_SIZE = 1,     ///< 获取配置数据长度,argIn:NULL, argOut参数为 int 指针，获取配置文件大小成功后可以通过ZC_CMD_GET_CONFIG命令获取配置内容
    ZC_CMD_GET_CONFIG = 2,          ///< 获取配置数据到用户申请的内存, argIn:NULL, argOut参数为 ZCLOUD_CMD_DATA 指针，
                                    ///< ZCLOUD_CMD_DATA.data指向用户申请的内存，ZCLOUD_CMD_DATA.size填写申请内存的大小(Byte)
    ZC_CMD_GET_CONFIG_FILE = 3,     ///< 获取配置数据并保存为文件, argIn参数为 char * 指针，c风格字符串，指向用户希望保存配置文件的路径,argOut:NULL
    ZC_CMD_SET_CONFIG = 4,          ///< 设置设备配置信息, argIn参数为 ZCLOUD_CMD_DATA 指针，ZCLOUD_CMD_DATA.data指向用户配置文件内容
                                    ///< ZCLOUD_CMD_DATA.size填写配置的大小(Byte),argOut:NULL
    ZC_CMD_SET_CONFIG_FILE = 5,     ///< 通过配置文件设置设备配置, argIn参数为 char *指针，c风格字符串，指向配置文件路径, argOut:NULL
    ZC_CMD_GET_DEVINFO = 6,         ///< 获取设备信息, argIn:NULL, argOut参数为 ZCLOUD_DEVINFO 指针
    ZC_CMD_GET_LOG_SIZE = 7,        ///< 获取日志数据长度, argIn:NULL, argOut参数为 int 指针，获取日志大小成功后可以通过ZC_CMD_GET_LOG命令获取日志内容
    ZC_CMD_GET_LOG = 8,             ///< 获取日志到用户申请的内存, argIn:NULL, argOut参数为 ZCLOUD_CMD_DATA 指针，
                                    ///< ZCLOUD_CMD_DATA.data指向用户申请的内存，ZCLOUD_CMD_DATA.size填写申请内存的大小(Byte)
    ZC_CMD_GET_LOG_FILE = 9,        ///< 获取日志并保存为文件，argIn参数为 char * 指针，c风格字符串，指向用户希望保存日志文件的路径,argOut:NULL
    ZC_CMD_GET_DEV_FILE_LIST = 10,  ///< 查询设备文件列表，根据查询到的信息使用ZC_CMD_GET_DEV_FILE_DATA保存文件, argIn指向ZCLOUD_QUERY_DEV_FILE_LIST_INFO
                                    ///< argOut指向ZCLOUD_FILE_LIST
    ZC_CMD_GET_DEV_FILE_DATA = 11,  ///< 请求设备文件内容, argIn指向ZCLOUD_GET_FILE_INFO, argOut:NULL
    ZC_CMD_DEV_FILE_DATA_ABORT = 12,///< 取消文件传输,argIn参数为 char * 指针，c风格字符串，指向用户希望取消传输的文件的名称,argOut:NULL
    ZC_CMD_UPGRADE_FIRMWARE = 13,   ///< 升级固件,argIn参数为 char * 指针，c风格字符串，指向固件的路径,argOut:NULL
    ZC_CMD_UPGRADE_FIRMWARE_WITH_CB = 14,///< 带有callback参数的固件升级,argIn参数为 ZCLOUD_FILE_PATH_WITH_CALLBACK 指针，argOut:NULL
                                         ///< ZCLOUD_FILE_PATH_WITH_CALLBACK直接提供固件文件路径进行固件升级
    //超时时间设置和获取 argIn,argOut都指向int类型的超时时间，时间单位ms，-1表示一直等待
    //argIn表示要设置的超时时间,NULL表示不设置此超时
    //argOut表示获取当前的超时时间,NULL表示不获取此超时
    ZC_CMD_TIMEOUT_DEV_RESPONSE = 15,    ///< 设备响应命令超时，此时间表示连接服务器后发送控制指令，设备需要在此时间内发送响应信息，默认值:5000ms
    ZC_CMD_TIMEOUT_GET_RECORD_FILE = 16, ///< 获取设备记录文件超时时间，此时间表示发送获取设备记录文件指令后，设备需要在此时间内将记录文件传输完毕，默认值:-1

    ZC_CMD_GET_DEV_DETAIL_INFO_SIZE = 17,///< 获取设备详细信息数据长度,argIn:NULL, argOut参数为 int 指针，取配置设备信息大小成功后可以通过ZC_CMD_GET_DEV_DETAIL_INFO命令获取设备信息
    ZC_CMD_GET_DEV_DETAIL_INFO = 18,     ///< 获取设备详细信息, argIn:NULL, argOut参数为 ZCLOUD_CMD_DATA 指针，
                                         ///< ZCLOUD_CMD_DATA.data指向用户申请的内存，ZCLOUD_CMD_DATA.size填写申请内存的大小(Byte)

    ZC_CMD_SET_DEV_RTC = 19,             ///< 设置设备的RTC时间，argIn为double类型指针，指向要设置的UTC时间,单位s，argOut:NULL。
    ZC_CMD_GET_DEV_RTC = 20,             ///< 获取设备的RTC时间，argIn:NULL，argOut参数为 double 指针, 指向获取的UTC时间,单位s。

    ZC_CMD_GET_DEV_STATUS_SIZE = 21,     ///< 获取设备状态信息长度,argIn:NULL, argOut参数为 int 指针，取配置设备状态信息大小成功后可以通过ZC_CMD_GET_DEV_STATUS命令获取设备状态信息
    ZC_CMD_GET_DEV_STATUS = 22,          ///< 获取设备状态信息, argIn:NULL, argOut参数为 ZCLOUD_CMD_DATA 指针，
                                         ///< ZCLOUD_CMD_DATA.data指向用户申请的内存，ZCLOUD_CMD_DATA.size填写申请内存的大小(Byte)
    ZC_CMD_UPGRADE_FIRMWARE_DATA_WITH_CB = 23,///< 带有callback参数的固件升级,argIn参数为 ZCLOUD_BUFF_DATA_WITH_CALLBACK 指针,
                                              ///< ZCLOUD_BUFF_DATA_WITH_CALLBACK直接提供固件数据缓冲地址进行固件升级，argOut:NULL
    ZC_CMD_DEV_LOG_CANCEL = 24,          ///< 取消当前日志的传输，argIn:NULL, argOut:NULL
    ZC_CMD_IS_MQTT_CONNECT = 25,         ///< 查询是否连接到mqtt服务器，argIn:NULL, argOut:指向int类型的指针，0表示mqtt未连接，1表示mqtt已连接

} ZCLOUD_CMD;

///< 设备控制代码
typedef enum _ZCloud_Dev_Ctrl
{
    ZC_DEV_CTRL_START_REC = 1,      ///< 控制设备开始记录数据
    ZC_DEV_CTRL_STOP_REC = 2,       ///< 控制设备停止记录数据
    ZC_DEV_CTRL_CLR_DEV_STORAGE = 3,///< 控制设备清空存储数据
    ZC_DEV_CTRL_RESET_DEV = 4,      ///< 控制设备复位

} ZCLOUD_DEV_CTRL;

///< 文件信息
typedef struct tagZCloudFileInfo
{
    uint32_t        size;   ///< 文件大小，单位字节(Byte)
    uint64_t        time;   ///< 文件创建时间，UTC时间戳
    char            name[ZCLOUD_FILENAME_MAX_LEN];
}ZCLOUD_FILE_INFO;

///< 文件列表信息
typedef struct tagZCloudFileList
{
    uint32_t            count;      ///< 文件列表文件个数
    uint64_t            time;       ///< 文件列表结束时间戳（UTC时间戳），若该时间段内超过100个文件，则该时间戳为第100个文件的时间戳，否则与下发的时间戳一致。
    ZCLOUD_FILE_INFO    files[ZCLOUD_FILELIST_MAX_COUNT];
}ZCLOUD_FILE_LIST;

///< 查询设备文件列表请求信息
typedef struct tagZCloudQueryDevFileListInfo
{
    uint64_t            timeBegin;  ///< 起始UTC时间戳,单位秒(s),从UTC时间1970.1.1 00:00:00 开始的秒数
    uint64_t            timeEnd;    ///< 结束UTC时间戳,单位秒(s),从UTC时间1970.1.1 00:00:00 开始的秒数
}ZCLOUD_QUERY_DEV_FILE_LIST_INFO;

///< 进度, ctx-上下文参数，progress-进度百分比(0.0-100.00表示0.0%-100.00%)
///< 返回值返回0表示用户中止操作，返回1程序继续执行
typedef int(*ZC_CallbackFun)(void* ctx, float progress);

///< 请求传输文件信息
typedef struct tagZCloudGetFileInfo
{
    const char* requestFileName;    ///< 请求传输的文件名称
    uint32_t    offset;             ///< 请求传输的文件偏移，可以从文件指定位置读取数据
    uint32_t    tag;                ///< 文件标签，用户自定义数值
    const char* saveFilePath;       ///< 用户指定要保存的文件路径
    ZC_CallbackFun cb;              ///< 下载文件回调函数，用于指示下载进度，不需要回调可以设置为NULL
    void*          ctx;             ///< 下载文件回调函数参数
}ZCLOUD_GET_FILE_INFO;

///< 请求传输文件信息
typedef struct tagZCloudFilePathWithCallback
{
    const char*     file;           ///< 文件名称
    ZC_CallbackFun  cb;             ///< 回调函数,指示进度
    void*           ctx;            ///< 回调函数参数
}ZCLOUD_FILE_PATH_WITH_CALLBACK;

///< 请求传输缓存数据信息
typedef struct tagZCloudBuffDataWithCallback
{
    const void*     pData;          ///< 缓存数据地址
    int             nDataLen;       ///< 缓存数据长度
    ZC_CallbackFun  cb;             ///< 回调函数,指示进度
    void*           ctx;            ///< 回调函数参数
}ZCLOUD_BUFF_DATA_WITH_CALLBACK;

///< 设备信息
typedef struct tagZCLOUD_DEVINFO
{
    int  devIndex;
    char type[64];
    char id[64];
    char owner[64];
    char model[64];
    char fwVer[16];
    char hwVer[16];
    char serial[64];
    char name[64];
    unsigned char canNum;
    int  status;             // 0:online, 1:offline
    unsigned char bCanUploads[16];    // each channel enable can upload
    unsigned char bGpsUpload;
}ZCLOUD_DEVINFO;

typedef struct tagZCLOUD_USER_DATA
{
    char username[64];
    char mobile[64];
    ZCLOUD_DEVINFO devices[ZCLOUD_MAX_DEVICES];
    unsigned int devCnt;
}ZCLOUD_USER_DATA;

///< 可变数据长度结构，data数据空间由用户申请
typedef struct tagZCloudCMDData
{
    int   size;     ///< data数据缓存大小
    void* data;     ///< 数据缓存指针
} ZCLOUD_CMD_DATA;

#pragma pack(pop)


#ifdef WIN32
#define FUNCALL __stdcall 
#else
#define FUNCALL 
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置连接服务器信息
 * @note  库中已经内置默认服务器，需要使用非默认服务器才需要进行设置
 * @param[in] httpSvr  登录服务器域名，用户登录设备管理等
 * @param[in] httpPort 登录服务器端口
 * @param[in] authSvr  认证服务器域名，获取mqtt服务器信息
 * @param[in] authPort 认证服务器端口
 * @return 无
 */
void FUNCALL ZC_SetServerInfo(const char* httpSvr, unsigned short httpPort, const char* authSvr, unsigned short authPort);

/**
 * @brief 使用用户名/密码登录服务器
 * @param[in] username  用户名
 * @param[in] password  密码
 * @return 返回错误码
 */
ZCLOUD_ERR_CODE FUNCALL ZC_ConnectServer(const char* username, const char* passord);

/**
 * @brief 断开服务器连接
 * @return 返回 错误码
 */
ZCLOUD_ERR_CODE FUNCALL ZC_DisconnectServer();

/**
 * @brief 检测服务器是否已经连接
 * @return 返回 1-是，0-否
 */
int FUNCALL ZC_IsServerConnected();

/**
 * @brief 用户登录成功后可以获取用户数据，包含设备列表
 * @return 返回 指向用户信息结构指针
 */
const ZCLOUD_USER_DATA* FUNCALL ZC_GetUserData();

/**
 * @brief  用户账户新增设备
 * @param[in] devid     设备云ID
 * @param[in] devtype   设备类型
 * @param[in] devname   设备显示名字
 * @param[in] devgroup  设备组，暂时不支持分组，传递null或者空字符串
 * @return 返回 错误代码
 */
ZCLOUD_ERR_CODE FUNCALL ZC_CloudAddDevice(const char* devid, const char* devtype, const char* devname, const char* devgroup);

/**
 * @brief 打开设备，执行设备相关操作需要使用返回的设备句柄
 * @return 返回 设备句柄，失败返回 
 */
DEVICE_HANDLE FUNCALL ZC_OpenDevice(int deviceType, int deviceIndex, int reserved);

/**
 * @brief  关闭设备
 * @return 返回 1-关闭成功，0-关闭失败
 */
int FUNCALL ZC_CloseDevice(DEVICE_HANDLE deviceHandle);

/**
 * @brief  设备是否打开
 * @return 返回 1-已打开，0-未打开
 */
int FUNCALL ZC_IsDeviceOpened(DEVICE_HANDLE deviceHandle);

/**
 * @brief 设备是否在线
 * @return 返回 1-是，0-否
 */
int FUNCALL ZC_IsDeviceOnLine(DEVICE_HANDLE deviceHandle);

/**
 * @brief  控制设备执行开始/停止记录，清空设备存储等操作
 * @param[in] deviceHandle 设备句柄
 * @param[in] devCtrl 设备控制命令码
 * @param[in] needResponse 是否需要设备响应，0:不需要设备响应；1:需要设备响应
 * @return 返回错误码
 */
ZCLOUD_ERR_CODE FUNCALL ZC_DeviceCtrl(DEVICE_HANDLE deviceHandle, ZCLOUD_DEV_CTRL devCtrl, int needResponse);

/**
 * @brief 操作命令通用接口
 * @param[in] deviceHandle 设备句柄，如无需指定设备可为INVALID_DEVICE_HANDLE
 * @param[in] cmd 命令码
 * @param[in] argIn 额外参数信息，根据不同的命令码传入对应的参数信息
 * @param[out] argOut 额外参数信息，根据不同的命令码传入对应的参数信息
 * @return 返回错误码。
 */
ZCLOUD_ERR_CODE FUNCALL ZC_ExecCmd(DEVICE_HANDLE deviceHandle, ZCLOUD_CMD cmd, void* argIn, void* argOut);


#ifdef __cplusplus
};
#endif

#endif //DTU_CLOUD_CONFIG_H_
