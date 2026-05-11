#ifndef  __C_MYBLUETOOTH_DLL__
#define __C_MYBLUETOOTH_DLL__

#define ZLL_BLE_API                             __declspec(dllexport)
#define CALLBACK                                __stdcall

/* 连接错误类型 */
#define ZLL_ERR_CODE                            int
#define ZLL_SUCCESS                             0                                   ///< 执行成功
#define ZLL_ERR_DEV                             -1                                  ///< 设备名输入无效
#define ZLL_ERR_SERVER                          -2                                  ///< 获取蓝牙服务无效
#define ZLL_ERR_CHARACTER                       -3                                  ///< 获取蓝牙特征值无效

/**
* @brief 回调函数，用于开启订阅时，获取接收到的数据
* @param[out] recv_size 接收到的数据大小
* @param[out] recv_data 接收到的数据
* @param[out] ReadBufLen 为接收到的数据长度
*/
typedef void(CALLBACK* ZLL_NotifyCall)(char* device_name, char* device_mac, int recv_size, unsigned char* recv_data);

/**
* @brief 回调函数，用于开启订阅时，获取接收到的数据
* @param[out] recv_size 接收到的数据大小
* @param[out] recv_data 接收到的数据
* @param[out] ReadBufLen 为接收到的数据长度
*/
typedef void(CALLBACK* ZLL_ConnectCall)(char* device_name, char* device_mac, bool isConnect);

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief 该函数扫描附近的ble设备并通过字符串数组返回设备名
* @param[in] scanTimes 定扫描时间，以ms为单位
* @param[in] flag_count 指定扫描设备数量
* @param[out] device 为扫描结果字符串数组
* @return 返回实际搜索到的ble设备数量
*/
ZLL_BLE_API int ZLL_ScanBluetoothLe(int scanTimes, int flag_count, char* device_name_arr[], char* device_mac_arr[]);

/**
* @brief 该函数会连接指定名字的ble设备
* @param[in] device_name 输入要连接的ble设备名
* @return 返回错误码
*/
ZLL_BLE_API int ZLL_ConnectBluetoothLe(const char *device_name, const char* device_mac, ZLL_ConnectCall connectCall);

/**
* @brief 断开连接ble设备
* @param[in] device_name 输入要断开连接的ble设备名
* @return 返回0执行成功 -1表示输入设备名无效（在存储列表中找无此设备）
*/
ZLL_BLE_API int ZLL_DisconnectBluetoothLe(const char* device_name, const char* device_mac);

/**
* @brief 该函数用于向连接ble设备发送数据
* @param[in] send_data 为待写入的字符串
* @param[in] length 为待写入的字符串长度
* @return 返回0执行成功 -1发送数据为空 -2写特征值无效
*/
ZLL_BLE_API int ZLL_Write(char* device_name, char* device_mac, int send_size, unsigned char* send_data);

/**
* @brief 该函数用于开启读订阅，当特征值发生改变时会通过回调函数返回特征值
* @param[in] notify_flag notify_flag为1表示开启订阅，0则表示取消订阅
* @param[out] notifyCall 为获取订阅数据的回调函数
* @return 返回0执行成功 -1订阅特征值无效
*/
ZLL_BLE_API int ZLL_Notify(int notify_flag, char* device_name, char* device_mac, ZLL_NotifyCall notifyCall);

#ifdef __cplusplus
}
#endif
#endif