#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "insideflash.h"
#include "stm32f1xx_hal.h"

#define ApplicationAddress 0x8005800

#define BOOT_DEBUG 1

#if BOOT_DEBUG

#define BOOT_INFO( fmt, args... ) 	printf( fmt, ##args )//FLASH_INFO(fmt, ##args)
#else
#define BOOT_INFO( fmt, args... )
#endif

#define ITM_Port8(n)   (*((volatile unsigned char*)(0xE0000000+4*n)))
#define ITM_Port16(n)  (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)  (*((volatile unsigned long*)(0xE0000000+4*n)))
#define DEMCR          (*((volatile unsigned long*)(0xE000edfc)))
#define TRCENA         0x01000000
struct __FILE{int handle;		/*Add whatever you need here*/};
FILE __stdout;
FILE __STDIN;
 
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

int fputc(int ch, FILE *f)
{
	if(DEMCR & TRCENA)
	{
		while(ITM_Port32(0) == 0);
		ITM_Port8(0)=ch;
	}
	return ch;
}

bool EraeseFlash( uint32_t addr, uint8_t sector ){
    return flash_erase( addr, sector );
}

bool CopyBackupArea( void ){
    
    uint8_t erase_count = 5;
    while( !EraeseFlash( ADDRESS_MAPPING( 11 ), 50 ) && erase_count-- ){
        BOOT_INFO( "ERASE FAILEL" );
        if( erase_count == 0 ){
            BOOT_INFO( "Failure of main program area erasure\r\n" );
            return false;
        }
    }
    
    BOOT_INFO( "Successful erasure of main program area\r\n" );
    
    for( uint8_t i = 0; i < 50; i ++ ){
        if( !flash_write( (uint8_t *)(( 62 )+ i * 2048), ( 11 ) + i * 2048, 2048 ) ){
            BOOT_INFO( "Backup program to override the main program area failure\r\n" );
            return false;
        }
    }
    BOOT_INFO( "Backup program to cover the main program area success\r\n" );
    return true;
}

bool Jump_Address(void){
    if( ((*(volatile uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000 ){
        BOOT_INFO( "###### Program jump success ######\r\n\r\n" );
        HAL_Delay( 100 );
        JumpAddress = *(volatile uint32_t*) (ApplicationAddress + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        __disable_irq();
        __set_MSP(*(volatile uint32_t*) ApplicationAddress);
        Jump_To_Application();
    }
    return false;
}

bool toggle_flag = true;
void HAL_IncTick(void){
    extern __IO uint32_t uwTick;
    uwTick += 1;
    if( uwTick % 50 == 0 && toggle_flag ){
        HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin );
    }
}











