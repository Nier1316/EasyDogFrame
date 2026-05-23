#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "RoboTasks/robot_app.h"

static RobotApp g_app;
static volatile bool g_running = true;

static void signal_handler(int) {
    g_running = false;
}

int main() {
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    if (!g_app.init()) {
        printf("[ERROR] RobotApp init failed\n");
        return -1;
    }

    g_app.start();
    printf("[INFO] Running. Press Ctrl+C to exit.\n");

    while (g_running) {
        sleep(1);
    }

    g_app.stop();
    return 0;
}
