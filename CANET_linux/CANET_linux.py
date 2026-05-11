from ctypes import *
import threading
import time
lib = cdll.LoadLibrary("./libCANET_TCP.so")
import platform
import ctypes

ZCAN_DEVICE_TYPE  = c_uint32
ZCAN_CANET_TCP = ZCAN_DEVICE_TYPE(17)
ZCAN_CANET_UDP = ZCAN_DEVICE_TYPE()

CMD_TCP_TYPE = 4  # 设置电脑的TCP模式
CMD_DESIP = 0  # 设置目标IP
CMD_DESPORT = 1  # 设置目标端口

g_thd_run = 1  # 线程运行标志
threads = []  # 接收线程
#对于CANET，此结构体不生效
class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode",c_int),
                ("AccMask",c_int),
                ("Reserved",c_int),
                ("Filter",c_ubyte),
                ("Timing0",c_ubyte),
                ("Timing1",c_ubyte),
                ("Mode",c_ubyte)]

class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID",c_uint32),
                ("TimeStamp",c_uint32),
                ("TimeFlag",c_uint8),
                ("SendType",c_byte),
                ("RemoteFlag",c_byte),
                ("ExternFlag",c_byte),
                ("DataLen",c_byte),
                ("Data",c_ubyte*8),
                ("Reserved",c_ubyte*3)]


def tcp_start(DEVCIE_TYPE, device_index, dest_port, chn_index):

    Value = "0"
    lib.VCI_SetReference(DEVCIE_TYPE, device_index, chn_index, CMD_TCP_TYPE, c_char_p(Value.encode("utf-8")))  # 设置电脑的TCP模式为客户端模式

    Value = "192.168.0.117"
    lib.VCI_SetReference(DEVCIE_TYPE, device_index, chn_index, CMD_DESIP, c_char_p(Value.encode("utf-8")))  # 设置目标IP

    value = c_uint32(dest_port)
    lib.VCI_SetReference(DEVCIE_TYPE, device_index, chn_index, CMD_DESPORT, byref(value))  # 设置目标端口

    init_config = VCI_INIT_CONFIG()
    memset(byref(init_config), 0, sizeof(init_config))
    chn_handle = lib.VCI_InitCAN(DEVCIE_TYPE, device_index, chn_index, byref(init_config))
    if chn_handle == 0:
        print("InitCAN fail!")
    else:
        print("InitCAN success!")
    ret = lib.VCI_StartCAN(DEVCIE_TYPE, device_index, chn_index)
    if ret == 0:
        print("StartCAN fail!")
    else:
        print("StartCAN success!")
    return chn_handle

def rx_thread(DevType, DevIdx, ChIdx):
    global g_thd_run

    while g_thd_run == 1:
        time.sleep(0.1)
        count = lib.VCI_GetReceiveNum(DevType, DevIdx, ChIdx) # CAN 报文数量
        if count > 0:
            can_data = (VCI_CAN_OBJ * count)()
            print('can-data',can_data)
            rcount = lib.VCI_Receive(DevType, DevIdx, ChIdx, can_data, count, 100) # 读报文

            for i in range(rcount):
                print('rcount',rcount)
                print("CAN ID: 0x%x "%(can_data[i].ID & 0x1FFFFFFF), end='')
                print("扩展帧  " if can_data[i].ExternFlag == 1 else "标准帧  ", end='')
                print("Data: ", end='')
                if(can_data[i].RemoteFlag == 0):   #数据帧
                    for j in range(can_data[i].DataLen):
                        print("%02x " % can_data[i].Data[j], end='')
                print("")

if __name__=="__main__":

    DEVCIE_TYPE =ZCAN_CANET_TCP

    chn_num = 1  # 设备通道数量；决定后面循环开启几个通道
    dest_port = 4001  # 目标端口号
    for i in range (chn_num):
        device_index = i
        Device_handle = lib.VCI_OpenDevice(DEVCIE_TYPE,device_index,0)
        if Device_handle ==0:
            print("Opendevice fail!")
        else:
            print("Opendevice success!")
        chn_index = i
        CHN_handle = tcp_start(DEVCIE_TYPE,device_index,dest_port + i, chn_index)

    device_index = 0  # 用于发送和接收的设备索引
    chn_index = 0  # 用于发送和接收的通道索引
    transmit_num = 8
    msgs = (VCI_CAN_OBJ * transmit_num)()
    for i in range(transmit_num):
        msgs[i].SendType = 0  # 0-正常发送，2-自发自收
        msgs[i].ExternFlag = 0  # 0-标准帧，1-扩展帧
        msgs[i].RemoteFlag = 0  # 0-数据帧，1-远程帧
        msgs[i].ID = i
        msgs[i].DataLen = 8
        for j in range(msgs[i].DataLen):
            msgs[i].Data[j] = j
    input()
    print("=============1=============")
    ret = lib.VCI_Transmit(DEVCIE_TYPE, device_index, chn_index, msgs, transmit_num)
    print("=============2=============")
    print("Tranmit Num: %d." % ret)


    thread = threading.Thread(target=rx_thread, args=(DEVCIE_TYPE, device_index, chn_index))  # 开启某一个通道的接收线程
    threads.append(thread)  # 独立接收线程
    thread.start()

    # 阻塞等待
    input()
    g_thd_run = 0

    # 等待所有线程完成
    for thread in threads:
        thread.join()

    for i in range(chn_num):
        device_index = i
        chn_index = i
        ret = lib.VCI_ResetCAN(DEVCIE_TYPE, device_index, chn_index)
        if ret == 0:
            print("ResetCAN(%d) fail" % i)
        else:
            print("ResetCAN(%d) success!" % i)

        ret = lib.VCI_CloseDevice(DEVCIE_TYPE, device_index)
        if ret == 0:
            print("Closedevice fail!")
        else:
            print("Closedevice success")