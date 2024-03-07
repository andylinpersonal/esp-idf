/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_assert.h"

#include "soc/soc.h"
#include "soc/lp_aon_reg.h"
#include "soc/reset_reasons.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup rtc_apis, rtc registers and memory related apis
  * @brief rtc apis
  */

/** @addtogroup rtc_apis
  * @{
  */

/**************************************************************************************
  *                                       Note:                                       *
  *       Some Rtc memory and registers are used, in ROM or in internal library.      *
  *          Please do not use reserved or used rtc memory or registers.              *
  *                                                                                   *
  *************************************************************************************
  *                          LP  Memory & Store Register usage
  *************************************************************************************
  *     rtc memory addr         type    size            usage
  *     0x3f421000(0x50000000)  Slow    SIZE_CP         Co-Processor code/Reset Entry
  *     0x3f421000+SIZE_CP      Slow    8192-SIZE_CP
  *
  *     0x3ff80000(0x40070000)  Fast    8192            deep sleep entry code
  *
  *************************************************************************************
  *     RTC store registers     usage
  *     LP_AON_STORE0_REG       Reserved
  *     LP_AON_STORE1_REG       RTC_SLOW_CLK calibration value
  *     LP_AON_STORE2_REG       Boot time, low word
  *     LP_AON_STORE3_REG       Boot time, high word
  *     LP_AON_STORE4_REG       External XTAL frequency
  *     LP_AON_STORE5_REG       FAST_RTC_MEMORY_LENGTH
  *     LP_AON_STORE6_REG       FAST_RTC_MEMORY_ENTRY
  *     LP_AON_STORE7_REG       FAST_RTC_MEMORY_CRC
  *     LP_AON_STORE8_REG       Store light sleep wake stub addr
  *     LP_AON_STORE9_REG       Store the sleep mode at bit[0]  (0:light sleep 1:deep sleep)
  *************************************************************************************
  */

#define RTC_SLOW_CLK_CAL_REG                LP_AON_STORE1_REG
#define RTC_BOOT_TIME_LOW_REG               LP_AON_STORE2_REG
#define RTC_BOOT_TIME_HIGH_REG              LP_AON_STORE3_REG
#define RTC_XTAL_FREQ_REG                   LP_AON_STORE4_REG
#define RTC_ENTRY_LENGTH_REG                LP_AON_STORE5_REG
#define RTC_ENTRY_ADDR_REG                  LP_AON_STORE6_REG
#define RTC_RESET_CAUSE_REG                 LP_AON_STORE6_REG
#define RTC_MEMORY_CRC_REG                  LP_AON_STORE7_REG
#define LIGHT_SLEEP_WAKE_STUB_ADDR_REG      LP_AON_STORE8_REG
#define SLEEP_MODE_REG                      LP_AON_STORE9_REG

#define RTC_DISABLE_ROM_LOG ((1 << 0) | (1 << 16)) //!< Disable logging from the ROM code.

typedef enum {
    AWAKE = 0,             //<CPU ON
    LIGHT_SLEEP = BIT0,    //CPU waiti, PLL ON.  We don't need explicitly set this mode.
    DEEP_SLEEP  = BIT1     //CPU OFF, PLL OFF, only specific timer could wake up
} SLEEP_MODE;

typedef enum {
    NO_MEAN                = 0,
    POWERON_RESET          = 1,   /**<1,  Power on reset*/
    RTC_SW_HPSYS_RESET     = 3,   /**<3,  Software reset hp system*/
    SLEEP_WAKEUP           = 5,   /**<5,  Deep Sleep reset hp system*/
    TG0_WDT_HPSYS_RESET    = 7,   /**<7,  Timer Group0 Watch dog reset hp system*/
    TG1_WDT_HPSYS_RESET    = 8,   /**<8,  Timer Group1 Watch dog reset hp system*/
    RTC_WDT_HPSYS_RESET    = 9,   /**<9,  RTC Watch dog Reset hp system*/
    TG0_WDT_CPU_RESET      = 11,  /**<11, Time Group0 reset CPU*/
    SW_CPU_RESET           = 12,  /**<12, Software reset CPU*/
    RTC_WDT_CPU_RESET      = 13,  /**<13, RTC Watch dog reset CPU*/
    RTC_BOD_SYS_RESET      = 15,  /**<15, System reset when the vdd voltage is not stable*/
    RTC_WDT_SYS_RESET      = 16,  /**<16, RTC Watch dog reset system*/
    TG1_WDT_CPU_RESET      = 17,  /**<17, Time Group1 reset CPU*/
    RTC_SWDT_SYS_RESET     = 18,  /**<18, super watchdog reset system*/
    EFUSE_HPSYS_RESET      = 20,  /**<20, efuse reset hp system*/
    USB_UART_HPSYS_RESET   = 21,  /**<21, usb uart reset hp system*/
    USB_JTAG_HPSYS_RESET   = 22,  /**<22, usb jtag reset hp system*/
    JTAG_CPU_RESET         = 24,  /**<24, jtag reset CPU*/
    RTC_PWR_GLITCH_RESET   = 25,  /**<25, RTC power glitch reset system*/
    CPU_LOCKUP_RESET       = 26,  /**<26, cpu lockup reset*/
} RESET_REASON;

// Check if the reset reason defined in ROM is compatible with soc/reset_reasons.h
ESP_STATIC_ASSERT((soc_reset_reason_t)POWERON_RESET == RESET_REASON_CHIP_POWER_ON, "POWERON_RESET != RESET_REASON_CHIP_POWER_ON");
ESP_STATIC_ASSERT((soc_reset_reason_t)RTC_SW_HPSYS_RESET == RESET_REASON_CORE_SW, "RTC_SW_HPSYS_RESET != RESET_REASON_CORE_SW");
ESP_STATIC_ASSERT((soc_reset_reason_t)SLEEP_WAKEUP == RESET_REASON_CORE_DEEP_SLEEP, "SLEEP_WAKEUP != RESET_REASON_CORE_DEEP_SLEEP");
ESP_STATIC_ASSERT((soc_reset_reason_t)TG0_WDT_HPSYS_RESET == RESET_REASON_CORE_MWDT0, "TG0_WDT_HPSYS_RESET != RESET_REASON_CORE_MWDT0");
ESP_STATIC_ASSERT((soc_reset_reason_t)TG1_WDT_HPSYS_RESET == RESET_REASON_CORE_MWDT1, "TG1_WDT_HPSYS_RESET != RESET_REASON_CORE_MWDT1");
ESP_STATIC_ASSERT((soc_reset_reason_t)RTC_WDT_HPSYS_RESET == RESET_REASON_CORE_RTC_WDT, "RTC_WDT_HPSYS_RESET != RESET_REASON_CORE_RTC_WDT");
ESP_STATIC_ASSERT((soc_reset_reason_t)TG0_WDT_CPU_RESET == RESET_REASON_CPU0_MWDT0, "TG0_WDT_CPU_RESET != RESET_REASON_CPU0_MWDT0");
ESP_STATIC_ASSERT((soc_reset_reason_t)SW_CPU_RESET == RESET_REASON_CPU0_SW, "SW_CPU_RESET != RESET_REASON_CPU0_SW");
ESP_STATIC_ASSERT((soc_reset_reason_t)RTC_WDT_CPU_RESET == RESET_REASON_CPU0_RTC_WDT, "RTC_WDT_CPU_RESET != RESET_REASON_CPU0_RTC_WDT");
ESP_STATIC_ASSERT((soc_reset_reason_t)RTC_BOD_SYS_RESET == RESET_REASON_SYS_BROWN_OUT, "RTC_BOD_SYS_RESET != RESET_REASON_SYS_BROWN_OUT");
ESP_STATIC_ASSERT((soc_reset_reason_t)RTC_WDT_SYS_RESET == RESET_REASON_SYS_RTC_WDT, "RTC_WDT_SYS_RESET != RESET_REASON_SYS_RTC_WDT");
ESP_STATIC_ASSERT((soc_reset_reason_t)TG1_WDT_CPU_RESET == RESET_REASON_CPU0_MWDT1, "TG1_WDT_CPU_RESET != RESET_REASON_CPU0_MWDT1");
ESP_STATIC_ASSERT((soc_reset_reason_t)RTC_SWDT_SYS_RESET == RESET_REASON_SYS_SUPER_WDT, "RTC_SWDT_SYS_RESET != RESET_REASON_SYS_SUPER_WDT");
ESP_STATIC_ASSERT((soc_reset_reason_t)EFUSE_HPSYS_RESET == RESET_REASON_CORE_EFUSE_CRC, "EFUSE_HPSYS_RESET != RESET_REASON_CORE_EFUSE_CRC");
ESP_STATIC_ASSERT((soc_reset_reason_t)USB_UART_HPSYS_RESET == RESET_REASON_CORE_USB_UART, "USB_UART_HPSYS_RESET != RESET_REASON_CORE_USB_UART");
ESP_STATIC_ASSERT((soc_reset_reason_t)USB_JTAG_HPSYS_RESET == RESET_REASON_CORE_USB_JTAG, "USB_JTAG_HPSYS_RESET != RESET_REASON_CORE_USB_JTAG");
ESP_STATIC_ASSERT((soc_reset_reason_t)JTAG_CPU_RESET == RESET_REASON_CPU0_JTAG, "JTAG_CPU_RESET != RESET_REASON_CPU0_JTAG");
ESP_STATIC_ASSERT((soc_reset_reason_t)RTC_PWR_GLITCH_RESET == RESET_REASON_CORE_PWR_GLITCH, "RTC_PWR_GLITCH_RESET != RESET_REASON_CORE_PWR_GLITCH");
ESP_STATIC_ASSERT((soc_reset_reason_t)CPU_LOCKUP_RESET == RESET_REASON_CPU0_LOCKUP, "CPU_LOCKUP_RESET != RESET_REASON_CPU0_LOCKUP");

typedef enum {
    NO_SLEEP        = 0,
    EXT_EVENT0_TRIG = BIT0,
    EXT_EVENT1_TRIG = BIT1,
    GPIO_TRIG       = BIT2,
    TIMER_EXPIRE    = BIT3,
    SDIO_TRIG       = BIT4,
    MAC_TRIG        = BIT5,
    UART0_TRIG      = BIT6,
    UART1_TRIG      = BIT7,
    TOUCH_TRIG      = BIT8,
    SAR_TRIG        = BIT9,
    BT_TRIG         = BIT10,
    RISCV_TRIG      = BIT11,
    XTAL_DEAD_TRIG  = BIT12,
    RISCV_TRAP_TRIG = BIT13,
    USB_TRIG        = BIT14
} WAKEUP_REASON;

typedef enum {
    DISEN_WAKEUP       = NO_SLEEP,
    EXT_EVENT0_TRIG_EN = EXT_EVENT0_TRIG,
    EXT_EVENT1_TRIG_EN = EXT_EVENT1_TRIG,
    GPIO_TRIG_EN       = GPIO_TRIG,
    TIMER_EXPIRE_EN    = TIMER_EXPIRE,
    SDIO_TRIG_EN       = SDIO_TRIG,
    MAC_TRIG_EN        = MAC_TRIG,
    UART0_TRIG_EN      = UART0_TRIG,
    UART1_TRIG_EN      = UART1_TRIG,
    TOUCH_TRIG_EN      = TOUCH_TRIG,
    SAR_TRIG_EN        = SAR_TRIG,
    BT_TRIG_EN         = BT_TRIG,
    RISCV_TRIG_EN      = RISCV_TRIG,
    XTAL_DEAD_TRIG_EN  = XTAL_DEAD_TRIG,
    RISCV_TRAP_TRIG_EN = RISCV_TRAP_TRIG,
    USB_TRIG_EN        = USB_TRIG
} WAKEUP_ENABLE;

/**
  * @brief  Get the reset reason for CPU.
  *
  * @param  int cpu_no : CPU no.
  *
  * @return RESET_REASON
  */
RESET_REASON rtc_get_reset_reason(int cpu_no);

/**
  * @brief  Get the wakeup cause for CPU.
  *
  * @param  int cpu_no : CPU no.
  *
  * @return WAKEUP_REASON
  */
WAKEUP_REASON rtc_get_wakeup_cause(void);

typedef void (* esp_rom_wake_func_t)(void);

/**
  * @brief Read stored RTC wake function address
  *
  * Returns pointer to wake address if a value is set in RTC registers, and stored length & CRC all valid.
  * valid means that both stored stub length and stored wake function address are four-byte aligned non-zero values
  * and the crc check passes
  *
  * @param  None
  *
  * @return esp_rom_wake_func_t : Returns pointer to wake address if a value is set in RTC registers
  */
esp_rom_wake_func_t esp_rom_get_rtc_wake_addr(void);

/**
  * @brief Store new RTC wake function address
  *
  * Set a new RTC wake address function. If a non-NULL function pointer is set then the function
  * memory is calculated and stored also.
  *
  * @param entry_addr Address of function. should be 4-bytes aligned otherwise it will not start from the stub after wake from deepsleep，
  *                   if NULL length will be ignored and all registers are cleared to 0.
  *
  * @param length length of function in RTC fast memory. should be less than RTC Fast memory size and aligned to 4-bytes.
  *               otherwise all registers are cleared to 0.
  *
  * @return None
  */
void esp_rom_set_rtc_wake_addr(esp_rom_wake_func_t entry_addr, size_t length);

/**
  * @brief Suppress ROM log by setting specific RTC control register.
  * @note This is not a permanent disable of ROM logging since the RTC register can not retain after chip reset.
  *
  * @param  None
  *
  * @return None
  */
static inline void rtc_suppress_rom_log(void)
{
    /* To disable logging in the ROM, only the least significant bit of the register is used,
     * but since this register is also used to store the frequency of the main crystal (RTC_XTAL_FREQ_REG),
     * you need to write to this register in the same format.
     * Namely, the upper 16 bits and lower should be the same.
     */
    REG_SET_BIT(LP_AON_STORE4_REG, RTC_DISABLE_ROM_LOG);
}

/**
  * @brief Software Reset digital core.
  *
  * It is not recommended to use this function in esp-idf, use
  * esp_restart() instead.
  *
  * @param  None
  *
  * @return None
  */
void software_reset(void);

/**
  * @brief Software Reset digital core.
  *
  * It is not recommended to use this function in esp-idf, use
  * esp_restart() instead.
  *
  * @param  int cpu_no : The CPU to reset, 0 for PRO CPU, 1 for APP CPU.
  *
  * @return None
  */
void software_reset_cpu(int cpu_no);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif