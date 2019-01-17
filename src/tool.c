/*
  tool_control.c - tool control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

void tool_init()
{

#ifdef STM32F103C8
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_TOOL_CHANGER_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = 1 << TOOL_CHANGER_BIT;
	GPIO_Init(TOOL_CHANGER_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_TOOL_M6_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin =  1 << TOOL_M6_BIT;
	GPIO_Init(TOOL_M6_PORT, &GPIO_InitStructure);

	ResetM6Enablebit();
	ResetToolChangerEnablebit();
#endif

  tool_stop();
}


// Returns current tool output state. Overrides may alter it from programmed state.
uint8_t tool_get_state()
{
  uint8_t cl_state = TOOL_T_DISABLE;

  #ifdef INVERT_TOOL_CHANGER_PIN

    if (bit_istrue(GPIO_ReadOutputData(TOOL_CHANGER_PORT),(1 << TOOL_CHANGER_BIT))){
    	cl_state &= TOOL_T_DISABLE;
    }

  #else

    if (bit_istrue(GPIO_ReadOutputData(TOOL_CHANGER_PORT),(1 << TOOL_CHANGER_BIT))){

    cl_state |= TOOL_T_DISABLE;
  }
  #endif


  return(cl_state);
}

// Returns current tool output state. Overrides may alter it from programmed state.
uint8_t m6_get_state()
{
  uint8_t cl_state = TOOL_M6_DISABLE;

    #ifdef INVERT_TOOL_M6_PIN

    if (bit_istrue(GPIO_ReadOutputData(TOOL_M6_PORT),(1 << TOOL_M6_BIT))){
    	cl_state &= TOOL_M6_DISABLE;
    }

    #else

      if (bit_istrue(GPIO_ReadOutputData(TOOL_M6_PORT),(1 << TOOL_M6_BIT))){
    	cl_state |= TOOL_M6_DISABLE;
      }

    #endif

  return(cl_state);
}

// Directly called by tool_init(), tool_set_state(), and mc_reset(), which can be at
// an interrupt-level. No report flag set, but only called by routines that don't need it.
void tool_stop()
{
	ResetToolChangerEnablebit(); //GPIO_ResetBits(TOOL_CHANGER_PORT, TOOL_CHANGER_BIT);
	ResetM6Enablebit(); //GPIO_ResetBits(TOOL_M6_PORT, TOOL_M6_BIT);
}

void tool_stop_tn()
{
    ResetToolChangerEnablebit();
	//GPIO_ResetBits(TOOL_CHANGER_PORT, TOOL_CHANGER_BIT);
}

void tool_stop_m6()
{
	ResetM6Enablebit();

}
// Directly called by stepper ISR, No report flag set, but only called by routines that don't need it.

// Main program only. Immediately sets flood tool running state and also mist tool,
// if enabled. Also sets a flag to report an update to a tool state.
// Called by tool toggle override, parking restore, parking retract, sleep mode, g-code
// parser program end, and g-code parser tool_sync().
void tool_set_t_state(uint8_t mode)
{
	//printPgmString(PSTR("\r\n tool_set_t_state"));
  if (sys.abort) { return; } // Block during abort.

  if (mode == TOOL_T_DISABLE) {
	  //printPgmString(PSTR("\r\n tool_stop_tn();"));
    tool_stop_tn();

  }

  if (mode == TOOL_T_ENABLE) {
	  //printPgmString(PSTR("\r\n (mode & TOOL_T_ENABLE) true"));
    #ifdef INVERT_TOOL_CHANGER_PIN
		sys_tool_state = TOOL_STATE_CHANGING;
		sys.tool_counter = settings.tool_delay; // load the counter full
		ResetToolChangerEnablebit(); //GPIO_ResetBits(TOOL_CHANGER_PORT, TOOL_CHANGER_BIT);

    #else
// here we need to toggle the tool changer pin n times according to the T command for duration $29 setting
		//printPgmString(PSTR("\r\n (mode & TOOL_T_ENABLE) false"));
		sys_tool_state = TOOL_T_ENABLE;
		sys.tool_counter = settings.tool_delay; // load the counter full
//		value = value + 1;
//		while (gc_block.values.t > 0){
		if (bit_istrue(GPIO_ReadOutputData(TOOL_CHANGER_PORT),(1 << TOOL_CHANGER_BIT))){
			//printPgmString(PSTR("\r\nTool Changer Bit = 1"));
			ResetToolChangerEnablebit();

		  } else {
			  {
				  //printPgmString(PSTR("\r\n Tool Changer Bit = 0"));
				  SetToolChangerEnablebit();

			  }
		}

//		value = value - 1;
//		}
		//GPIO_SetBits(TOOL_CHANGER_PORT, TOOL_CHANGER_BIT); // On
		/* pass on the changer tool state to isr via the state monitor
		 * ISR counts ticks and when reached $29 delay turn off the port via tool_stop()
		 */
    #endif
    }


  sys.report_ovr_counter = 0; // Set to report change immediately
}

void tool_set_m6_state(uint8_t mode)
{
	//printPgmString(PSTR("\r\n tool_set_m6_state"));
  if (sys.abort) { return; } // Block during abort.

  if (mode == TOOL_M6_DISABLE) {
	  //printPgmString(PSTR("\r\n tool_stop_m6();"));
    tool_stop_m6();

  }

  if (mode == TOOL_M6_ENABLE) {
    #ifdef INVERT_TOOL_M6_PIN
        sys_m6_state = TOOL_M6_ENABLE;
    	sys.m6_counter = settings.m6_delay;
    	ResetM6Enablebit();

    #else
// here we need to energise the tool valve M6 for a set time according to $9 setting
//    if (settings.m6_ff == 0){ // M6 is working with one duration pulse via stepper isr
//    	sys_m6_state = TOOL_M6_ENABLE;
//    	sys.m6_counter = settings.m6_delay;
//    	SetM6Enablebit();
//    	//GPIO_SetBits(TOOL_M6_PORT, TOOL_M6_BIT);
//	/* pass on the M6 tool state to isr via the state monitor?
//	 * ISR counts ticks and when reached $9 delay turn off the port via tool_stop()
//	 */} else { // M6 works as a manual flip flop, so no need for the isr to time it.
		  if (bit_istrue(GPIO_ReadOutputData(TOOL_M6_PORT),(1 << TOOL_M6_BIT))){
			  //printPgmString(PSTR("\r\n Tool M6 Bit = 1"));
		         ResetM6Enablebit();
		  } else {
			  //printPgmString(PSTR("\r\n Tool M6 Bit = 0"));
				  SetM6Enablebit();
			     }
//		}
    #endif
    }

  sys.report_ovr_counter = 0; // Set to report change immediately
}

// G-code parser entry-point for setting tool state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
void tool_m6_sync(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  //printPgmString(PSTR("tool.c executes tool_m6sync"));
  protocol_buffer_synchronize(); // Ensure tool turns on when specified in program.
  tool_set_m6_state(mode);
}
void tool_t_sync(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  //printPgmString(PSTR("tool.c executes tool_tsync"));
  protocol_buffer_synchronize(); // Ensure tool turns on when specified in program.
  tool_set_t_state(mode);
}
void tool_state_monitor() //TBD based on the Probe pattern to create a duration via the ISR
{
  if (tool_get_state()) {
    sys.tool_counter--; // is the number of ISR cycles @ 35 Usec each
    //decrement tool counter
	 if (sys.tool_counter <= 10) {
		 tool_set_state(TOOL_T_DISABLE); //if tool counter = 0 then call stop tool
	 }
    //do something; do we have enough isr ticks for the delay?
  }
}
void m6_state_monitor()
{
  if (settings.m6_ff == 0 ) { // generate one pulse with x length
    if (m6_get_state()) {
	    sys.m6_counter--;// decrement m6 counter
		   if (sys.m6_counter <= 10 ) {
			   tool_set_state(TOOL_M6_DISABLE);
		   }	  // if m6 counter =0 then call stop m6
    //do something;
    } // else don't do anything
  }
}

