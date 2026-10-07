#ifdef __FREERTOS

// arduino-pico's startFreeRTOS() runs setup()/loop() - and so the whole Meshtastic packet path - in a
// task it creates as xTaskCreate(__core0, "CORE0", 1024, ...): 1024 words, 4 KB, with no build option
// to change it. That is not enough. xeddsa_verify() alone takes ~1.85 KB, and the loopback decode of a
// position broadcast sent through the Ethernet HTTP API reached it with 1.37 KB left: the stack
// overflowed and the watchdog reset the node. The TCP API path cleared it with 232 bytes to spare.
//
// The rp2350 base links with -Wl,--wrap=xTaskCreate, so the framework's call lands here and the CORE0
// task gets a bigger stack. Every other task is created as asked.

#include <FreeRTOS.h>
#include <string.h>
#include <task.h>

#ifndef MESHTASTIC_CORE0_STACK_WORDS
#define MESHTASTIC_CORE0_STACK_WORDS 2048 // 8 KB; StackType_t is 32-bit
#endif

extern "C" BaseType_t __real_xTaskCreate(TaskFunction_t pxTaskCode, const char *const pcName,
                                         const configSTACK_DEPTH_TYPE uxStackDepth, void *const pvParameters,
                                         UBaseType_t uxPriority, TaskHandle_t *const pxCreatedTask);

extern "C" BaseType_t __wrap_xTaskCreate(TaskFunction_t pxTaskCode, const char *const pcName,
                                         const configSTACK_DEPTH_TYPE uxStackDepth, void *const pvParameters,
                                         UBaseType_t uxPriority, TaskHandle_t *const pxCreatedTask)
{
    configSTACK_DEPTH_TYPE depth = uxStackDepth;
    if (pcName && strcmp(pcName, "CORE0") == 0 && depth < MESHTASTIC_CORE0_STACK_WORDS)
        depth = MESHTASTIC_CORE0_STACK_WORDS;
    return __real_xTaskCreate(pxTaskCode, pcName, depth, pvParameters, uxPriority, pxCreatedTask);
}

#endif
