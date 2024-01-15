#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv/ulp_riscv.h"
#include "ulp_riscv/ulp_riscv_utils.h"
#include "ulp_riscv/ulp_riscv_gpio.h"

#define KEY_IRQ GPIO_NUM_10
#define IMU_IRQ GPIO_NUM_9
#define TOUCH_IRQ GPIO_NUM_11
static bool gpio_level_1 = false;
static bool gpio_level_2 = false;
// static bool gpio_level_2 = false;

/* this variable will be exported as a public symbol, visible from main CPU: */

int main(void)
{
    //delay
    long long int ii = 3000000ULL;
    while (ii--);

    while (1)
    {
        gpio_level_1 = (bool)ulp_riscv_gpio_get_level(IMU_IRQ);
        gpio_level_2 = (bool)ulp_riscv_gpio_get_level(KEY_IRQ);
        /* Wakes up the main CPU if pin changed its state */

        if (gpio_level_1 == 1||gpio_level_2 == 0)
        {
            ulp_riscv_wakeup_main_processor();
            break;
        }
    }
    /* ulp_riscv_shutdown() is called automatically when main exits */
    return 0;
}