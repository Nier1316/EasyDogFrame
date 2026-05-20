#include "example.h"




// ================= 示例 1：原始 CAN 帧测试（仅依赖 BSP） =================
void Example1_RawCanFrameTest() {
    printf("\n========== Example 6: Raw CAN Frame Test (BSP Only) ==========\n");
    printf("[INFO] This example only depends on BSP layer, no motor needed\n");

    BspCan& bsp = BspCan::GetInstance();

    // 初始化 can0
    CanDeviceConfig config;
    config.device_idx = 0;
    config.port = 4001;
    config.server_ip = "192.168.0.178";
    config.work_mode = TCP_CLIENT;

    if (!bsp.InitDevice(0, config)) {
        printf("[ERROR] Failed to initialize device 0\n");
        return;
    }

    if (!bsp.StartDevice(0)) {
        printf("[ERROR] Failed to start device 0\n");
        return;
    }

    printf("[INFO] Device 0 initialized and started\n");

    // 发送测试帧
    printf("\n[ACTION] Sending test frames\n");
    for (int i = 0; i < 3; i++) {
        BspCanFrame frame;
        frame.id = 0x001 + i;
        frame.dlc = 8;
        frame.is_extended = 0;
        for (int j = 0; j < 8; j++) {
            frame.data[j] = i * 8 + j;
        }

        printf("[ACTION] Sending frame %d: ID=0x%03x, data=[", i + 1, frame.id);
        for (int j = 0; j < 8; j++) {
            printf("%02x ", frame.data[j]);
        }
        printf("]\n");

        if (!bsp.SendFrame(0, frame)) {
            printf("[ERROR] Failed to send frame\n");
        }

        sleep(1);
    }

    bsp.StopDevice(0);
    printf("[INFO] Example 6 completed\n");
}