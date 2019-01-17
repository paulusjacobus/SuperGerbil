/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
/*
 * author Paul use the same technique for ATC as used for the probe
 */
volatile uint8_t sys_tool_state;// toggle state value.  Used to coordinate the toggle cycle with stepper ISR.
volatile uint8_t sys_m6_state;// M6 state value.  Used to coordinate the M6 cycle with stepper ISR.
/*
 *
 */
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
volatile uint8_t sys_rt_exec_debug;
#endif

#if defined (STM32F103C8)
#include "usb_lib.h"
#ifdef USEUSB
#include "usb_desc.h"
#endif
#include "hw_config.h"
#ifdef USEUSB
#include "usb_pwr.h"
#endif
#include "stm32eeprom.h"
//#ifndef USEUSB
#include "stm32f10x_usart.h"
void USART1_Configuration(u32 BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);                 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;	  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	 
	USART_InitStructure.USART_Parity = USART_Parity_No;	 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);
	USART_Init(USART1, &USART_InitStructure);
	//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
#endif

//#endif


#ifdef WIN32
int main(int argc, char *argv[])
#else
int main(void)
#endif
{
#if defined (STM32F103C8)
	/* Timing and initialization
	 * Author: Paul
	 * We have re-mapped the SWDIO/CLK pins for Tool Change purposes which means these
	 * ISCP functions has disappeared after we have flashed the Bobby Dazzler.
	 * A work around is to erase the chip first when it has been flashed before
	 * so it's restored into the default pin configuration
	 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // added to release the jtag pins for gpio functions paul
    AFIO->MAPR = AFIO_MAPR_SWJ_CFG_DISABLE;

    RCC_HCLKConfig(RCC_SYSCLK_Div1); // High speed data bus
    RCC_PCLK1Config(RCC_HCLK_Div2);//paul high speed peripheral bus
    RCC_PCLK2Config(RCC_HCLK_Div1); // low speed peripheral bus
    //SystemInit();


    //RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//paul

#define LEDBLINK
#ifdef LEDBLINK
	GPIO_InitTypeDef GPIO_InitStructure;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//GPIOC
#endif
uint8_t setflagmessage = 0;

	//Set_System();
//#ifndef USEUSB
	USART1_Configuration(115200);
//#else
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
//#endif

#ifndef NOEEPROMSUPPORT
	FLASH_Unlock();
	eeprom_init();
#endif
	SysTick->CTRL &= 0xfffffffb;
	//SysTick_Config(1000);//paul lost usb on a device with no recovery until a full erase done

#endif
  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts

  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt
  spindle_init(0); // paul, added to set the spindle timers for pwm
  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.

  // Initialize system state.


  #ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif
  
  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // Reset system variables.
//    delay_ms(2000);
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
		memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
    sys_probe_state = 0;
    /*
     * author Paul ATC
     */
    sys_m6_state = 0;
    sys_tool_state = 0;
    /*
     * ATC ends
     */
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    /*
      * Author Paul, added pwm mode for freq settings and tool changer
      */
     spindle_init(settings.pwm_mode);
     settings_init(settings.pwm_mode); // Paul's frequency add on: added parm in order to Load Grbl settings from EEPROM
     tool_init(); //Paul, when ready to test tool change
     /*
      * end
      */
    coolant_init();
    //limits_init(); //Done now in system_init(). 14/01/19 Paul
    system_init();
    // end
    probe_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();
      //delay_ms(32000);
//    while ((Virtual_Com_port_IsHostPortOpen()) == false){
//        delay_ms(1000);
//		LedBlink();
//    }
    /*
     * Author Paul
     * We read out the jumper bridge to determine whether we have selected USB or UART
     * USB mode: flash every second
     * USART mode: flash twice as fast as USB mode
     *
     */
    if (GPIO_ReadInputDataBit(SERIALSWITCH_PORT, SERIALSWITCH_BIT) == 1){ //if the jumper is bridged->USB
		while (Virtual_Com_port_IsHostPortOpen() == false){
			delay_ms(500);
			LedBlink();
			}
		}
    else { //Serial switch jumper is open ->USART
		while (USART_GetFlagStatus(USART1, USART_FLAG_IDLE)== 0){
			delay_ms(250);
			LedBlink();
			}
    }
      // Print welcome message. Indicates an initialization has occurred at power-up or with a reset.
      report_init_message();


    LedBlink();
	//Delay(500);
//    delay_ms(500); // Paul

    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();
  }

  return 0;   /* Never reached */
}
#if defined (STM32F103C8)
void _delay_ms(uint32_t x)
{
	u32 temp;
	//SysTick->LOAD = (u32)72000000 / 72000;                     // Loading time
	SysTick->LOAD = (u32)72000000 / 8000 * x;                     // fix the ms delay variation
	SysTick->VAL = 0x00;                                            // Empty the counter
	SysTick->CTRL = 0x01;                                           // Start from bottom
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp&(1 << 16)));                             // Wait time arrive
	SysTick->CTRL = 0x00;                                            // Close the counter
	SysTick->VAL = 0X00;                                            // Empty the counter
}



void LedBlink()
{
	static BitAction nOnFlag = Bit_SET;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, nOnFlag); // C13 is connected to led which flashes to demonstrate the program is running
	nOnFlag = (nOnFlag == Bit_SET) ? Bit_RESET : Bit_SET;
}
void myputchar(unsigned char c)
{
uart_putc(c, USART1);
}
unsigned char mygetchar()
{
return uart_getc(USART1);
}

#endif
