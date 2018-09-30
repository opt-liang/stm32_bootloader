#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define ApplicationAddress 0x8005800

typedef  void (*pFunction)(void);

uint32_t JumpAddress;
pFunction Jump_To_Application;

void Jump_Address(void){
    if( ((*(volatile uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000 ){
        JumpAddress = *(volatile uint32_t*) (ApplicationAddress + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        __disable_irq();
        __set_MSP(*(volatile uint32_t*) ApplicationAddress);
        Jump_To_Application();
    }
}











