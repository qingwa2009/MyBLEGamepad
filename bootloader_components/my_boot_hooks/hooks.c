#include "esp_log.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"
#include "bootloader_common.h"
#include "bootloader_config.h"

#define PIN_BTN_XBOX GPIO_NUM_46
#define PIN_SWITCH_ON_OFF GPIO_NUM_45

#define PIN_LED_1 GPIO_NUM_21
#define PIN_LED_2 GPIO_NUM_47
#define PIN_LED_3 GPIO_NUM_48
#define PIN_LED_4 GPIO_NUM_38

/* Function used to tell the linker to include this file
 * with all its symbols.
 */
void bootloader_hooks_include(void)
{
}

void bootloader_before_init(void)
{
    /* Keep in my mind that a lot of functions cannot be called from here
     * as system initialization has not been performed yet, including
     * BSS, SPI flash, or memory protection. */
    ESP_LOGI("HOOK", "-------------------在二级引导程序之前-------------------");
    //PIN_BTN_XBOX 设成输入模式，用来判断是按下开关开机，还是插入USB开机。
    esp_rom_gpio_pad_pullup_only(PIN_BTN_XBOX);
    esp_rom_gpio_pad_select_gpio(PIN_BTN_XBOX);
    gpio_ll_input_enable(&GPIO, PIN_BTN_XBOX);
    uint8_t isPowOnByPushBtn = !gpio_ll_get_level(&GPIO, PIN_BTN_XBOX);

    //PIN_SWITCH_ON_OFF 设成输入输出模式，如果是按xbox键开机就接通电池，否则不接通电池。
    esp_rom_gpio_pad_select_gpio(PIN_SWITCH_ON_OFF);
    gpio_ll_output_enable(&GPIO, PIN_SWITCH_ON_OFF);
    gpio_ll_input_enable(&GPIO, PIN_SWITCH_ON_OFF);
    gpio_ll_set_level(&GPIO, PIN_SWITCH_ON_OFF, isPowOnByPushBtn);

    //点亮所有led
    esp_rom_gpio_pad_select_gpio(PIN_LED_1);
    esp_rom_gpio_pad_select_gpio(PIN_LED_2);
    esp_rom_gpio_pad_select_gpio(PIN_LED_3);
    esp_rom_gpio_pad_select_gpio(PIN_LED_4);
    gpio_ll_output_enable(&GPIO, PIN_LED_1);
    gpio_ll_output_enable(&GPIO, PIN_LED_2);
    gpio_ll_output_enable(&GPIO, PIN_LED_3);
    gpio_ll_output_enable(&GPIO, PIN_LED_4);
    gpio_ll_set_level(&GPIO, PIN_LED_1, 1);
    gpio_ll_set_level(&GPIO, PIN_LED_2, 1);
    gpio_ll_set_level(&GPIO, PIN_LED_3, 1);
    gpio_ll_set_level(&GPIO, PIN_LED_4, 1);

    ESP_LOGI("HOOK", "is pow on by push btn: %d", isPowOnByPushBtn);
}

void bootloader_after_init(void)
{
    ESP_LOGI("HOOK", "-------------------在二级引导程序之后-------------------");
}
