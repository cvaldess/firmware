#include "HardwareRNG.h"
#include "configuration.h"
#include "hardware/xosc.h"
#include <cstring>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>

#if defined(FAULT_CAPTURE) && defined(__FREERTOS)
#include <FreeRTOS.h>
#include <task.h>
#endif

#ifdef HEAP_WATCH
// Temporary: last known free-heap / live packetPool-bytes snapshot, updated periodically
// from Power.cpp (see HEAP_WATCH block there). Plain globals (not heap) so they're safe to
// read from the fault ISR below without touching malloc/memaudit state that may itself be
// the thing that's corrupted.
volatile uint32_t heapWatchLastFreeHeap = 0;
volatile uint32_t heapWatchLastPktPoolBytes = 0;

void heapWatchUpdate(uint32_t freeHeap, uint32_t pktPoolBytes)
{
    heapWatchLastFreeHeap = freeHeap;
    heapWatchLastPktPoolBytes = pktPoolBytes;
}
#endif

#ifdef FAULT_CAPTURE
// Temporary: catch a hard fault, stash the faulting PC/LR/CFSR in the watchdog scratch
// registers (they survive the reset that follows), and report them once syslog is up on
// the next boot. This is how we pin the traceroute crash to an exact instruction.
#define FAULT_MAGIC 0xFA017CAFu

extern "C" void faultCaptureHandler(uint32_t *stacked)
{
    volatile uint32_t *const cfsr = (uint32_t *)0xE000ED28;
    watchdog_hw->scratch[0] = FAULT_MAGIC;
    watchdog_hw->scratch[1] = stacked[6]; // stacked PC
    watchdog_hw->scratch[2] = stacked[5]; // stacked LR
    watchdog_hw->scratch[3] = *cfsr;      // configurable fault status
#ifdef HEAP_WATCH
    // Last periodic sample, not live at the instant of the fault - good enough to tell a
    // starved pool/heap from a healthy one without calling into (possibly corrupted) malloc.
    watchdog_hw->scratch[4] = heapWatchLastFreeHeap;
    watchdog_hw->scratch[5] = heapWatchLastPktPoolBytes;
#endif
    watchdog_reboot(0, 0, 0);
    while (true) {
    }
}

#ifdef __FREERTOS
// All rp2350 targets build with -D__FREERTOS=1: setup()/loop() - and so the whole packet path,
// traceroute+MQTT included - run inside the "CORE0" task on a 4 KB stack (freertos-main.cpp,
// xTaskCreate(__core0, "CORE0", 1024, ...)), the same task fix(se050) 8634e479a found tight via
// a completely different path. FreeRTOS is built with configCHECK_FOR_STACK_OVERFLOW=2, so an
// overflow there is *detected*, not silent corruption - but arduino-pico's default
// vApplicationStackOverflowHook (weak) just calls panic("Stack overflow"), which prints to
// whatever stdout is wired to (USB serial, not syslog) and then spins in _exit(1) - never
// through isr_hardfault, so FAULT_CAPTURE above has been completely blind to this failure mode.
// Overriding the weak hook here catches it the same way: stash a marker (distinct from
// FAULT_MAGIC) across the reset and let reportFaultCrumb() report it once syslog is back up.
#define STACK_OVERFLOW_MAGIC 0xFA017570u

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    watchdog_hw->scratch[0] = STACK_OVERFLOW_MAGIC;
    uint32_t nameWord = 0;
    for (int i = 0; i < 4 && pcTaskName[i]; i++)
        nameWord |= ((uint32_t)(uint8_t)pcTaskName[i]) << (i * 8);
    watchdog_hw->scratch[1] = nameWord;
#ifdef HEAP_WATCH
    watchdog_hw->scratch[4] = heapWatchLastFreeHeap;
    watchdog_hw->scratch[5] = heapWatchLastPktPoolBytes;
#endif
    watchdog_reboot(0, 0, 0);
    while (true) {
    }
}
#endif

// Configurable faults escalate to HardFault, so this one vector catches bus/mem/usage
// faults too. Pick the stack (MSP/PSP) the exception frame was pushed onto.
extern "C" __attribute__((naked)) void isr_hardfault(void)
{
    __asm volatile("movs r0, #4      \n"
                   "mov  r1, lr      \n"
                   "tst  r0, r1      \n"
                   "beq  1f          \n"
                   "mrs  r0, psp     \n"
                   "b    2f          \n"
                   "1:  mrs r0, msp  \n"
                   "2:  ldr r1, =faultCaptureHandler \n"
                   "bx   r1          \n");
}

static void reportFaultCrumb()
{
    if (watchdog_hw->scratch[0] == FAULT_MAGIC) {
        LOG_ERROR("FAULT: previous boot HARD FAULT pc=0x%08x lr=0x%08x cfsr=0x%08x", (unsigned)watchdog_hw->scratch[1],
                  (unsigned)watchdog_hw->scratch[2], (unsigned)watchdog_hw->scratch[3]);
#ifdef HEAP_WATCH
        LOG_ERROR("FAULT: last HEAPWATCH sample before fault: free=%u pktpool=%u", (unsigned)watchdog_hw->scratch[4],
                  (unsigned)watchdog_hw->scratch[5]);
#endif
        watchdog_hw->scratch[0] = 0;
        return;
    }
#ifdef __FREERTOS
    if (watchdog_hw->scratch[0] == STACK_OVERFLOW_MAGIC) {
        char name[5] = {0};
        uint32_t w = watchdog_hw->scratch[1];
        for (int i = 0; i < 4; i++)
            name[i] = (char)((w >> (i * 8)) & 0xFF);
        LOG_ERROR("FAULT: previous boot STACK OVERFLOW task='%s'", name);
#ifdef HEAP_WATCH
        LOG_ERROR("FAULT: last HEAPWATCH sample before fault: free=%u pktpool=%u", (unsigned)watchdog_hw->scratch[4],
                  (unsigned)watchdog_hw->scratch[5]);
#endif
        watchdog_hw->scratch[0] = 0;
        return;
    }
#endif
}
#endif

#ifdef __PLAT_RP2040__
#include <pico/sleep.h>

static bool awake;

static void sleep_callback(void)
{
    awake = true;
}

void epoch_to_datetime(time_t epoch, datetime_t *dt)
{
    struct tm *tm_info;

    tm_info = gmtime(&epoch);
    dt->year = tm_info->tm_year;
    dt->month = tm_info->tm_mon + 1;
    dt->day = tm_info->tm_mday;
    dt->dotw = tm_info->tm_wday;
    dt->hour = tm_info->tm_hour;
    dt->min = tm_info->tm_min;
    dt->sec = tm_info->tm_sec;
}

void debug_date(datetime_t t)
{
    LOG_DEBUG("%d %d %d %d %d %d %d", t.year, t.month, t.day, t.hour, t.min, t.sec, t.dotw);
    uart_default_tx_wait_blocking();
}

void cpuDeepSleep(uint32_t msecs)
{

    time_t seconds = (time_t)(msecs / 1000);
    datetime_t t_init, t_alarm;

    awake = false;
    // Start the RTC
    rtc_init();
    epoch_to_datetime(0, &t_init);
    rtc_set_datetime(&t_init);
    epoch_to_datetime(seconds, &t_alarm);
    // debug_date(t_init);
    // debug_date(t_alarm);
    uart_default_tx_wait_blocking();
    sleep_run_from_dormant_source(DORMANT_SOURCE_ROSC);
    sleep_goto_sleep_until(&t_alarm, &sleep_callback);

    // Make sure we don't wake
    while (!awake) {
        delay(1);
    }

    /* For now, I don't know how to revert this state
        We just reboot in order to get back operational */
    rp2040.reboot();

    /* Set RP2040 in dormant mode. Will not wake up. */
    // xosc_dormant();
}

#else
void cpuDeepSleep(uint32_t msecs)
{
    /* Set RP2040 in dormant mode. Will not wake up. */
    xosc_dormant();
}
#endif

void setBluetoothEnable(bool enable)
{
    // not needed
}

void updateBatteryLevel(uint8_t level)
{
    // not needed
}

void getMacAddr(uint8_t *dmac)
{
    pico_unique_board_id_t src;
    pico_get_unique_board_id(&src);
    dmac[5] = src.id[7];
    dmac[4] = src.id[6];
    dmac[3] = src.id[5];
    dmac[2] = src.id[4];
    dmac[1] = src.id[3];
    dmac[0] = src.id[2];
}

bool getDeviceId(uint8_t *deviceId)
{
    // RP2040/RP2350: 64-bit unique board id (flash serial / OTP) in bytes 0-7 (rest stay zero).
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    memcpy(deviceId, board_id.id, sizeof(board_id.id));
    return true;
}

void rp2040Setup()
{
    if (watchdog_caused_reboot()) {
        LOG_WARN("Rebooted by watchdog!");
    }

    /* Sets a random seed to make sure we get different random numbers on each boot. */
    uint32_t seed = 0;
    if (!HardwareRNG::seed(seed)) {
        seed = rp2040.hwrand32();
    }
    randomSeed(seed);

#ifdef RP2040_SLOW_CLOCK
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    LOG_INFO("Clock speed:");
    LOG_INFO("pll_sys  = %dkHz", f_pll_sys);
    LOG_INFO("pll_usb  = %dkHz", f_pll_usb);
    LOG_INFO("rosc     = %dkHz", f_rosc);
    LOG_INFO("clk_sys  = %dkHz", f_clk_sys);
    LOG_INFO("clk_peri = %dkHz", f_clk_peri);
    LOG_INFO("clk_usb  = %dkHz", f_clk_usb);
    LOG_INFO("clk_adc  = %dkHz", f_clk_adc);
    LOG_INFO("clk_rtc  = %dkHz", f_clk_rtc);
#endif
}

void rp2040Loop()
{
    static bool watchdog_running = false;
    if (!watchdog_running) {
        watchdog_enable(8000, true); // 8s timeout; pauses during debug
        watchdog_running = true;
    }
    watchdog_update();

#ifdef FAULT_CAPTURE
    // Report the previous boot's fault once, after syslog is up (~15s).
    static bool faultReported = false;
    if (!faultReported && millis() > 15000) {
        faultReported = true;
        reportFaultCrumb();
    }
#endif
}

void enterDfuMode()
{
    reset_usb_boot(0, 0);
}

/* Init in early boot state. */
#ifdef RP2040_SLOW_CLOCK
void initVariant()
{
    /* Set the system frequency to 18 MHz. */
    set_sys_clock_khz(18 * KHZ, false);
    /* The previous line automatically detached clk_peri from clk_sys, and
       attached it to pll_usb. We need to attach clk_peri back to system PLL to keep SPI
       working at this low speed.
       For details see https://github.com/jgromes/RadioLib/discussions/938
    */
    clock_configure(clk_peri,
                    0,                                                // No glitchless mux
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
                    18 * MHZ,                                         // Input frequency
                    18 * MHZ                                          // Output (must be same as no divider)
    );
    /* Run also ADC on lower clk_sys. */
    clock_configure(clk_adc, 0, CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 18 * MHZ, 18 * MHZ);
    /* Run RTC from XOSC since USB clock is off */
    clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12 * MHZ, 47 * KHZ);
    /* Turn off USB PLL */
    pll_deinit(pll_usb);
}
#endif