#include <stdio.h>
#include "MyKeypadMatrix.h"
#include "driver/dedic_gpio.h"
#include "esp_log.h"

#define PIN_ROW_COUNT 4
#define PIN_COL_COUNT 4

static dedic_gpio_bundle_handle_t rowHandles = NULL;
static dedic_gpio_bundle_handle_t colHandles = NULL;

void initMyKeypadMatrix(gpio_num_t r0, gpio_num_t r1, gpio_num_t r2, gpio_num_t r3, gpio_num_t c0, gpio_num_t c1, gpio_num_t c2, gpio_num_t c3)
{
    gpio_reset_pin(r0);
    gpio_reset_pin(r1);
    gpio_reset_pin(r2);
    gpio_reset_pin(r3);
    gpio_reset_pin(c0);
    gpio_reset_pin(c1);
    gpio_reset_pin(c2);
    gpio_reset_pin(c3);
    int rowPins[] = {r0, r1, r2, r3};
    int colPins[] = {c0, c1, c2, c3};

    gpio_config_t ioConf = {
        .mode = GPIO_MODE_OUTPUT,
    };

    for (size_t i = 0; i < PIN_ROW_COUNT; i++)
    {
        ioConf.pin_bit_mask = BIT64(rowPins[i]);
        gpio_config(&ioConf);
    }

    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pull_up_en = 1;
    for (size_t i = 0; i < PIN_COL_COUNT; i++)
    {
        ioConf.pin_bit_mask = BIT64(colPins[i]);
        gpio_config(&ioConf);
    }

    dedic_gpio_bundle_config_t rowConf = {
        .gpio_array = rowPins,
        .array_size = PIN_ROW_COUNT,
        .flags = {
            .out_en = 1,
        }};
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&rowConf, &rowHandles));

    dedic_gpio_bundle_config_t colConf = {
        .gpio_array = colPins,
        .array_size = PIN_COL_COUNT,
        .flags = {
            .in_en = 1,
            .in_invert = 1,
        }};

    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&colConf, &colHandles));
    dedic_gpio_bundle_write(rowHandles, (1 << PIN_ROW_COUNT) - 1, 0);
}

static void waitForStable()
{
    dedic_gpio_bundle_write(rowHandles, (1 << PIN_ROW_COUNT) - 1, 0xFF);
    uint32_t v = 0;
    do
    {
        v = dedic_gpio_bundle_read_in(colHandles);
    } while (v);
}
//电平的切换速度不行，不知具体原因。
uint16_t scanMyKeyMatrix(uint8_t btnLevelSwitchDelayNopCount)
{
    uint16_t v = 0;

    uint32_t colValues = 0;

    //row 0
    dedic_gpio_bundle_write(rowHandles, (1 << PIN_ROW_COUNT) - 1, ~BIT(0));
    for (size_t i = 0; i < btnLevelSwitchDelayNopCount; i++)
        asm volatile("nop");
    colValues = dedic_gpio_bundle_read_in(colHandles);
    v |= (uint16_t)colValues;

    waitForStable();
    //row 1
    dedic_gpio_bundle_write(rowHandles, (1 << PIN_ROW_COUNT) - 1, ~BIT(1));
    for (size_t i = 0; i < btnLevelSwitchDelayNopCount; i++)
        asm volatile("nop");
    colValues = dedic_gpio_bundle_read_in(colHandles);
    v |= (uint16_t)colValues << 4;

    waitForStable();
    //row 2
    dedic_gpio_bundle_write(rowHandles, (1 << PIN_ROW_COUNT) - 1, ~BIT(2));
    for (size_t i = 0; i < btnLevelSwitchDelayNopCount; i++)
        asm volatile("nop");
    colValues = dedic_gpio_bundle_read_in(colHandles);
    v |= (uint16_t)colValues << 8;

    waitForStable();
    //row 3
    dedic_gpio_bundle_write(rowHandles, (1 << PIN_ROW_COUNT) - 1, ~BIT(3));
    for (size_t i = 0; i < btnLevelSwitchDelayNopCount; i++)
        asm volatile("nop");
    colValues = dedic_gpio_bundle_read_in(colHandles);
    v |= (uint16_t)colValues << 12;

    dedic_gpio_bundle_write(rowHandles, (1 << PIN_ROW_COUNT) - 1, 0xff);

    return v;
}

void printMyKeypadMatrix(const char *TAG, uint16_t keys, uint8_t oneRowFormate)
{
    if (oneRowFormate)
    {
        char s[] = "0000 0000 0000 0000";
        if (keys & 0b1)
            s[0] = '1';
        if (keys & 0b10)
            s[1] = '1';
        if (keys & 0b100)
            s[2] = '1';
        if (keys & 0b1000)
            s[3] = '1';
        keys >>= 4;
        if (keys & 0b1)
            s[5] = '1';
        if (keys & 0b10)
            s[6] = '1';
        if (keys & 0b100)
            s[7] = '1';
        if (keys & 0b1000)
            s[8] = '1';
        keys >>= 4;
        if (keys & 0b1)
            s[10] = '1';
        if (keys & 0b10)
            s[11] = '1';
        if (keys & 0b100)
            s[12] = '1';
        if (keys & 0b1000)
            s[13] = '1';
        keys >>= 4;
        if (keys & 0b1)
            s[15] = '1';
        if (keys & 0b10)
            s[16] = '1';
        if (keys & 0b100)
            s[17] = '1';
        if (keys & 0b1000)
            s[18] = '1';
        ESP_LOGI(TAG, "%s", s);
    }
    else
    {
        char s[] = ""
                   "0000\r\n"
                   "0000\r\n"
                   "0000\r\n"
                   "0000\r\n"
                   "----";
        if (keys & 0b1)
            s[0] = '1';
        if (keys & 0b10)
            s[1] = '1';
        if (keys & 0b100)
            s[2] = '1';
        if (keys & 0b1000)
            s[3] = '1';
        keys >>= 4;
        if (keys & 0b1)
            s[6] = '1';
        if (keys & 0b10)
            s[7] = '1';
        if (keys & 0b100)
            s[8] = '1';
        if (keys & 0b1000)
            s[9] = '1';
        keys >>= 4;
        if (keys & 0b1)
            s[12] = '1';
        if (keys & 0b10)
            s[13] = '1';
        if (keys & 0b100)
            s[14] = '1';
        if (keys & 0b1000)
            s[15] = '1';
        keys >>= 4;
        if (keys & 0b1)
            s[18] = '1';
        if (keys & 0b10)
            s[19] = '1';
        if (keys & 0b100)
            s[20] = '1';
        if (keys & 0b1000)
            s[21] = '1';
        ESP_LOGI(TAG, "\r\n%s", s);
    }
}