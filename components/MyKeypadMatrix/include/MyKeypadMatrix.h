// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * 4x4 keypad matrix
     * r0~r3 为输出，扫描时逐行设置低电平，然后逐列读取，低电平表示按下。
     * c0~c3 为上拉输入。 
    */
    void initMyKeypadMatrix(gpio_num_t r0, gpio_num_t r1, gpio_num_t r2, gpio_num_t r3, gpio_num_t c0, gpio_num_t c1, gpio_num_t c2, gpio_num_t c3);
    /**
     * 扫描键盘
     * 返回16位数，每4位代表一行，第一个4位代表第一行，每4位中第一位代表第一列，以此类推。
     * btnLevelSwitchDelayNopCount：该参数是在扫描时，行输出和列输入之间插入nop指令的个数，因为按键的电平切换会有延迟。
    */
    uint16_t scanMyKeyMatrix(uint8_t btnLevelSwitchDelayNopCount);
    /**
     * ESP_LOGI(TAG, )打印scanMyKeyMatrix返回的值
    */
    void printMyKeypadMatrix(const char *TAG, uint16_t keys, uint8_t oneRowFormate);
    /**
     * 防抖
    */
    void keepHighMyKeypadMatrix(uint16_t *keys, uint32_t dt, uint32_t keepHighMS);
#ifdef __cplusplus
}
#endif
