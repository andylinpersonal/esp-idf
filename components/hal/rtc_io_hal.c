/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// The HAL layer for RTC IO (common part)

#include "hal/rtc_io_hal.h"
#include "soc/soc_caps.h"

#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED

void rtcio_hal_set_direction(int rtcio_num, rtc_gpio_mode_t mode)
{
    switch (mode) {
    case RTC_GPIO_MODE_INPUT_ONLY:
        rtcio_ll_output_mode_set(rtcio_num, RTCIO_OUTPUT_NORMAL);
        rtcio_ll_output_disable(rtcio_num);
        rtcio_ll_input_enable(rtcio_num);
        break;
    case RTC_GPIO_MODE_OUTPUT_ONLY:
        rtcio_ll_output_mode_set(rtcio_num, RTCIO_OUTPUT_NORMAL);
        rtcio_ll_output_enable(rtcio_num);
        rtcio_ll_input_disable(rtcio_num);
        break;
    case RTC_GPIO_MODE_INPUT_OUTPUT:
        rtcio_ll_output_mode_set(rtcio_num, RTCIO_OUTPUT_NORMAL);
        rtcio_ll_output_enable(rtcio_num);
        rtcio_ll_input_enable(rtcio_num);
        break;
    case RTC_GPIO_MODE_DISABLED:
        rtcio_ll_output_mode_set(rtcio_num, RTCIO_OUTPUT_NORMAL);
        rtcio_ll_output_disable(rtcio_num);
        rtcio_ll_input_disable(rtcio_num);
        break;
    case RTC_GPIO_MODE_OUTPUT_OD:
        rtcio_ll_output_mode_set(rtcio_num, RTCIO_OUTPUT_OD);
        rtcio_ll_output_enable(rtcio_num);
        rtcio_ll_input_disable(rtcio_num);
        break;
    case RTC_GPIO_MODE_INPUT_OUTPUT_OD:
        rtcio_ll_output_mode_set(rtcio_num, RTCIO_OUTPUT_OD);
        rtcio_ll_output_enable(rtcio_num);
        rtcio_ll_input_enable(rtcio_num);
        break;
    default:
        break;
    }
}

rtc_gpio_mode_t rtcio_hal_get_direction(int rtcio_num)
{
    enum {
        _RTC_GPIO_MODE_DISABLED_DEF = 0x0,
        _RTC_GPIO_MODE_INPUT_DEF = 0x1,
        _RTC_GPIO_MODE_OUTPUT_DEF = 0x2,
        _RTC_GPIO_MODE_OD_DEF = 0x4,

        RTC_GPIO_MODE_DISABLED_ = _RTC_GPIO_MODE_DISABLED_DEF,
        RTC_GPIO_MODE_INPUT_ONLY_ = _RTC_GPIO_MODE_INPUT_DEF,
        RTC_GPIO_MODE_OUTPUT_ONLY_ = _RTC_GPIO_MODE_OUTPUT_DEF,
        RTC_GPIO_MODE_OUTPUT_OD_ = _RTC_GPIO_MODE_OUTPUT_DEF | _RTC_GPIO_MODE_OD_DEF,
        RTC_GPIO_MODE_INPUT_OUTPUT_ = _RTC_GPIO_MODE_INPUT_DEF | _RTC_GPIO_MODE_OUTPUT_DEF,
        RTC_GPIO_MODE_INPUT_OUTPUT_OD_ = _RTC_GPIO_MODE_INPUT_DEF | _RTC_GPIO_MODE_OUTPUT_DEF | _RTC_GPIO_MODE_OD_DEF,

    } mode = _RTC_GPIO_MODE_DISABLED_DEF;

    if (rtcio_ll_input_is_enabled(rtcio_num)) {
        mode |= _RTC_GPIO_MODE_INPUT_DEF;
    }

    if (rtcio_ll_output_is_enabled(rtcio_num)){
        mode |= _RTC_GPIO_MODE_OUTPUT_DEF;
    }

    if (rtcio_ll_output_mode_get(rtcio_num) == RTCIO_OUTPUT_OD) {
        mode |= _RTC_GPIO_MODE_OD_DEF;
    }

    switch (mode)
    {
    case RTC_GPIO_MODE_OUTPUT_ONLY_:
        return RTC_GPIO_MODE_OUTPUT_ONLY;
    case RTC_GPIO_MODE_OUTPUT_OD_:
        return RTC_GPIO_MODE_OUTPUT_OD;
    case RTC_GPIO_MODE_INPUT_OUTPUT_:
        return RTC_GPIO_MODE_INPUT_OUTPUT;
    case RTC_GPIO_MODE_INPUT_OUTPUT_OD_:
        return RTC_GPIO_MODE_INPUT_OUTPUT_OD;
    default:
        if (mode & _RTC_GPIO_MODE_INPUT_DEF) {
          return RTC_GPIO_MODE_INPUT_ONLY;
        } else {
          return RTC_GPIO_MODE_DISABLED;
        }
    }
}

void rtcio_hal_set_direction_in_sleep(int rtcio_num, rtc_gpio_mode_t mode)
{
    switch (mode) {
    case RTC_GPIO_MODE_INPUT_ONLY:
        rtcio_ll_enable_input_in_sleep(rtcio_num);
        rtcio_ll_disable_output_in_sleep(rtcio_num);
        rtcio_ll_enable_sleep_setting(rtcio_num);
        break;
    case RTC_GPIO_MODE_OUTPUT_ONLY:
        rtcio_ll_enable_output_in_sleep(rtcio_num);
        rtcio_ll_disable_input_in_sleep(rtcio_num);
        rtcio_ll_enable_sleep_setting(rtcio_num);
        break;
    case RTC_GPIO_MODE_INPUT_OUTPUT:
        rtcio_ll_enable_input_in_sleep(rtcio_num);
        rtcio_ll_enable_output_in_sleep(rtcio_num);
        rtcio_ll_enable_sleep_setting(rtcio_num);
        break;
    case RTC_GPIO_MODE_DISABLED:
        rtcio_ll_disable_input_in_sleep(rtcio_num);
        rtcio_ll_disable_output_in_sleep(rtcio_num);
        rtcio_ll_disable_sleep_setting(rtcio_num);
        break;
    default:
        break;
    }
}

rtc_gpio_mode_t rtcio_hal_get_direction_in_sleep(int rtcio_num) {
    enum {
      _RTC_GPIO_MODE_IN_SLEEP_DISABLED_DEF = 0x0,
      _RTC_GPIO_MODE_IN_SLEEP_INPUT_DEF = 0x1,
      _RTC_GPIO_MODE_IN_SLEEP_OUTPUT_DEF = 0x2,
      RTC_GPIO_MODE_IN_SLEEP_DISABLED_ = _RTC_GPIO_MODE_IN_SLEEP_DISABLED_DEF,
      RTC_GPIO_MODE_IN_SLEEP_INPUT_ONLY_ = _RTC_GPIO_MODE_IN_SLEEP_INPUT_DEF,
      RTC_GPIO_MODE_IN_SLEEP_OUTPUT_ONLY_ = _RTC_GPIO_MODE_IN_SLEEP_OUTPUT_DEF,
      RTC_GPIO_MODE_IN_SLEEP_INPUT_OUTPUT_ = _RTC_GPIO_MODE_IN_SLEEP_INPUT_DEF | _RTC_GPIO_MODE_IN_SLEEP_OUTPUT_DEF,
    } mode = _RTC_GPIO_MODE_IN_SLEEP_DISABLED_DEF;

    if (rtcio_ll_input_in_sleep_is_enabled(rtcio_num)) {
        mode |= _RTC_GPIO_MODE_IN_SLEEP_INPUT_DEF;
    }

    if (rtcio_ll_output_in_sleep_is_enabled(rtcio_num)) {
        mode |= _RTC_GPIO_MODE_IN_SLEEP_OUTPUT_DEF;
    }

    switch (mode) {
    case RTC_GPIO_MODE_IN_SLEEP_INPUT_ONLY_:
        return RTC_GPIO_MODE_INPUT_ONLY;
    case RTC_GPIO_MODE_IN_SLEEP_OUTPUT_ONLY_:
        return RTC_GPIO_MODE_OUTPUT_ONLY;
    case RTC_GPIO_MODE_IN_SLEEP_INPUT_OUTPUT_:
        return RTC_GPIO_MODE_INPUT_OUTPUT;
    default:
        return RTC_GPIO_MODE_DISABLED;
    }
}

#if SOC_RTCIO_HOLD_SUPPORTED
void rtcio_hal_isolate(int rtcio_num)
{
    rtcio_ll_pullup_disable(rtcio_num);
    rtcio_ll_pulldown_disable(rtcio_num);
    rtcio_ll_output_disable(rtcio_num);
    rtcio_ll_input_disable(rtcio_num);
    rtcio_ll_force_hold_enable(rtcio_num);
}
#endif

#endif //SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
