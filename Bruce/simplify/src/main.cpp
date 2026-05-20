#include <stdio.h>
#include <unistd.h>       // sleep()
#include <signal.h>       // signal() / SIGINT / SIGTERM
#include "bsp/bsp_can.h"  
#include "example.h"




int main(int argc, char* argv[]){
    // int example = 1;
    //     if (argc > 1) {
    //     example = atoi(argv[1]);
    // }

    // printf("========================================\n");
    // printf("   CAN Motor Control Framework Demo\n");
    // printf("========================================\n\n");


    // printf("Usage: %s [example_number]\n", argv[0]);
    // printf("  1 - Raw CAN Frame Test (BSP Only)\n\n");


    // switch (example) {
    //     case 1:
    //         Example1_RawCanFrameTest();
    //         break;
    //     default:
    //         printf("[ERROR] Unknown example number: %d\n", example);
    //         return -1;
    // }

    // printf("\n========================================\n");
    // printf("   Demo Completed\n");
    // printf("========================================\n");

    // Example5_DuplicateRegister();
    // Example6_ThreadRestart();
    Example7_AutoCleanup();
    // Example8_StateQuery();

    return 0;
}