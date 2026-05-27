#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"

int main(void)
{
    SYSCFG_DL_init();
    TB6612_Init();

    lc_printf("\r\nTB6612 rear-drive differential demo start\r\n");
    delay_ms(1000);

    while (1) {
        lc_printf("Forward: left=250 right=250\r\n");
        TB6612_SetDifferential(250, 250);
        delay_ms(1500);
        TB6612_Brake();
        delay_ms(800);

        lc_printf("Turn right: left=300 right=140\r\n");
        TB6612_SetDifferential(300, 140);
        delay_ms(1000);
        TB6612_Brake();
        delay_ms(800);

        lc_printf("Turn left: left=140 right=300\r\n");
        TB6612_SetDifferential(140, 300);
        delay_ms(1000);
        TB6612_Brake();
        delay_ms(1500);
    }
}
