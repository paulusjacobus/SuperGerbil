/*
  cpu_map.h - CPU and pin mapping configuration file
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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h
#endif
#include "grbl.h"

//#ifdef CPU_MAP_STM32F103

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.


#define STEP_PORT       GPIOA
#define RCC_STEP_PORT   RCC_APB2Periph_GPIOA
#define X_STEP_BIT      0 //GPIO_Pin_0
#define Y_STEP_BIT      1 //GPIO_Pin_1
#define Z_STEP_BIT      2 //GPIO_Pin_2
#define A_STEP_BIT      3 // GPIO_Pin_3
#define B_STEP_BIT		4 //GPIO_Pin_4
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<A_STEP_BIT)|(1<<B_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT        GPIOA
#define RCC_DIRECTION_PORT    RCC_APB2Periph_GPIOA
#define X_DIRECTION_BIT       5 //5//GPIO_Pin_5
#define Y_DIRECTION_BIT       6 //GPIO_Pin_6
#define Z_DIRECTION_BIT       7 //GPIO_Pin_7
#define A_DIRECTION_BIT       8 //GPIO_Pin_8
#define B_DIRECTION_BIT       15 //GPIO_Pin_15
#define DIRECTION_MASK        ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)|(1<<A_DIRECTION_BIT)|(1<<B_DIRECTION_BIT))// All direction bits

// port to measure the ISR duration
//#define ISR_PORT     GPIOC
//#define RCC_ISR_PORT  RCC_APB2Periph_GPIOC
//#define ISR_BIT       14 // Pin Gpio_Pin_14 on Port C is used for testing the ISR duration with the CRO

  // Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT     GPIOC                //
#define RCC_STEPPERS_DISABLE_PORT RCC_APB2Periph_GPIOC //
#define STEPPERS_DISABLE_BIT      15                   //GPIO_Pin_15
#define STEPPERS_DISABLE_MASK     (1<<STEPPERS_DISABLE_BIT)
#define SetStepperDisableBit()    GPIO_WriteBit(STEPPERS_DISABLE_PORT,1 << STEPPERS_DISABLE_BIT, Bit_SET) //GPIO_WriteBit(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, Bit_SET)
#define ResetStepperDisableBit()  GPIO_WriteBit(STEPPERS_DISABLE_PORT,1 << STEPPERS_DISABLE_BIT, Bit_RESET)


  // Define homing/hard limit switch input pins and limit interrupt vectors. 
  // NOTE: All limit bit pins must be on the same port
#define LIMIT_PIN        GPIOB
#define LIMIT_PORT       GPIOB
#define RCC_LIMIT_PORT   RCC_APB2Periph_GPIOB
#define GPIO_LIMIT_PORT  GPIO_PortSourceGPIOB
#define X_LIMIT_BIT      10 //GPIO_Pin_10
#define Y_LIMIT_BIT      12 //GPIO_Pin_11
#define Z_LIMIT_BIT      11 //GPIO_Pin_12  // swapped Y,Z to allow pcb routing
#define A_LIMIT_BIT      13  // additional axis Paul
#define B_LIMIT_BIT      14  // additional 2nd axis
/*
 * Author Paul added fault pin for DC motor feed back. In limit pins and not in control, because pin 15 falls in EXTI_15
 * All AFIO pins on Port B must be configured together otherwise only one (last one) with get active
 * and leave previous IO pins hanging in ***void*** or cyber space (whatever, just pick one).
 */
#define CONTROL_FAULT_BIT             15 //PortB, GPIO_Pin_15, Added as an additional state, DC motor feedback pin (Low is a Fault condition)
//#define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<A_LIMIT_BIT)|(1<<B_LIMIT_BIT)|(1<<CONTROL_FAULT_BIT)) // All limit bits
#define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<A_LIMIT_BIT)|(1<<B_LIMIT_BIT)) // All limit bits

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIOB //GPIOC
#define RCC_SPINDLE_ENABLE_PORT     RCC_APB2Periph_GPIOB     //RCC_APB2Periph_GPIOC
#define SPINDLE_ENABLE_BIT          1 //GPIO_Pin_1      // B1 Sets the LO relay EN=DIR
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
//#define SPINDLE_DIRECTION_DDR   GPIOB //GPIOC
#define RCC_SPINDLE_DIRECTION_PORT  RCC_APB2Periph_GPIOB     //RCC_APB2Periph_GPIOC
#define SPINDLE_DIRECTION_PORT      GPIOB //GPIOC
#define SPINDLE_DIRECTION_BIT       2     //
#define SetSpindleDirectionBit()    GPIO_WriteBit(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, Bit_SET)
#define ResetSpindleDirectionBit()  GPIO_WriteBit(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, Bit_RESET)
#endif
#define SetSpindleEnablebit()       GPIO_WriteBit(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, Bit_SET)
#define ResetSpindleEnablebit()     GPIO_WriteBit(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, Bit_RESET)


//example of led blink = GPIO_WriteBit(GPIOC, GPIO_Pin_13, nOnFlag)

  // Define flood and mist coolant enable output pins.
  // a later date if flash and memory space allows.
#define COOLANT_FLOOD_PORT            GPIOB
#define RCC_COOLANT_FLOOD_PORT        RCC_APB2Periph_GPIOB //
#define COOLANT_FLOOD_BIT             3 //GPIO_Pin_3
#define COOLANT_MIST_PORT             GPIOB
#define RCC_COOLANT_MIST_PORT         RCC_APB2Periph_GPIOB
#define COOLANT_MIST_BIT              4
#define SetFloodEnablebit()           GPIO_WriteBit(COOLANT_FLOOD_PORT, 1 << COOLANT_FLOOD_BIT, Bit_SET)
#define ResetFloodEnablebit()         GPIO_WriteBit(COOLANT_FLOOD_PORT, 1 << COOLANT_FLOOD_BIT, Bit_RESET)
#define SetMistEnablebit()            GPIO_WriteBit(COOLANT_MIST_PORT, 1 << COOLANT_MIST_BIT, Bit_SET)
#define ResetMistEnablebit()          GPIO_WriteBit(COOLANT_MIST_PORT, 1 << COOLANT_MIST_BIT, Bit_RESET)


/*
 * Author Paul: Tool Changer definition pins
 */
  // Define tool changer and M6 enable output pins.
#define TOOL_CHANGER_PORT             GPIOA
#define RCC_TOOL_CHANGER_PORT         RCC_APB2Periph_GPIOA //
#define TOOL_CHANGER_BIT              13 //GPIO_Pin_13
#define TOOL_M6_PORT                  GPIOA
#define RCC_TOOL_M6_PORT              RCC_APB2Periph_GPIOA
#define TOOL_M6_BIT                   14
#define SetToolChangerEnablebit()     GPIO_WriteBit(TOOL_CHANGER_PORT, 1 << TOOL_CHANGER_BIT, Bit_SET)
#define ResetToolChangerEnablebit()   GPIO_WriteBit(TOOL_CHANGER_PORT, 1 << TOOL_CHANGER_BIT, Bit_RESET)
#define SetM6Enablebit()              GPIO_WriteBit(TOOL_M6_PORT, 1 << TOOL_M6_BIT, Bit_SET)
#define ResetM6Enablebit()            GPIO_WriteBit(TOOL_M6_PORT, 1 << TOOL_M6_BIT, Bit_RESET)

// Define user-control controls (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PIN_PORT              GPIOB
#define CONTROL_PORT                  GPIOB
#define RCC_CONTROL_PORT              RCC_APB2Periph_GPIOB
#define GPIO_CONTROL_PORT             GPIO_PortSourceGPIOB
#define CONTROL_RESET_BIT             5 //GPIO_Pin_5
#define CONTROL_FEED_HOLD_BIT         6 //GPIO_Pin_6
#define CONTROL_CYCLE_START_BIT       7 //GPIO_Pin_7
#define CONTROL_SAFETY_DOOR_BIT       8 //GPIO_Pin_8
#define CONTROL_MASK                 ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))

  // Define probe switch input pin.
#define PROBE_PORT                    GPIOB // moved to GPIOB from A 6/08/2018
#define RCC_PROBE_PORT                RCC_APB2Periph_GPIOB //moved to GPIO B
#define PROBE_BIT                     0  //GPIO_Pin_0
#define PROBE_MASK                    (1<<PROBE_BIT)

// Define USB/UART switch INPUT PIN
#define SERIALSWITCH_PORT                    GPIOC // moved to GPIOB from A 6/08/2018
#define RCC_SERIALSWITCH_PORT                RCC_APB2Periph_GPIOC //moved to GPIO B
#define SERIALSWITCH_BIT                     GPIO_Pin_14
#define SERIALSWITCH_MASK                    (1<<SERIALSWITCH_BIT)

  // Start of PWM & Stepper Enabled Spindle
//#ifdef VARIABLE_SPINDLE

  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
#define SPINDLE_PWM_FREQUENCY       10000                  // KHz
#define SPINDLE_PWM_DDR	            GPIOB //B
#define SPINDLE_PWM_PORT            GPIOB //B
#define RCC_SPINDLE_PWM_PORT        RCC_APB2Periph_GPIOB //RCC_APB2Periph_GPIOB
#define SPINDLE_PWM_BIT	            GPIO_Pin_9 // PB9 is PWM output, was 8 in conflict with USB port
//#endif // End of VARIABLE_SPINDLE
#define SPINDLE_PWM_MAX_VALUE       (1000000 / SPINDLE_PWM_FREQUENCY)

#ifndef SPINDLE_PWM_MIN_VALUE
#define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
#endif
#define SPINDLE_PWM_OFF_VALUE     0
#define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)

  //  PORT A                         PORT B                    PORT C
  //----------------------------------------------------------------------------------
  //   0      X_STEP_BIT           |  PROBE_BIT               | xxxx
  //   1      Y_STEP_BIT           |  SPINDLE_ENABLE_BIT      | xxxx
  //   2      Z_STEP_BIT           |  SPINDLE_DIRECTION_BIT   | xxxx
  //   3      A_STEP_BIT           |  COOLANT_FLOOD_BIT		  | xxxx
  //   4      B_STEP_BIT           |  COOLANT_MIST_BIT		  | xxxx
  //   5      X_DIRECTION_BIT      |  CONTROL_RESET_BIT		  | xxxx
  //   6      Y_DIRECTION_BIT      |  CONTROL_FEED_HOLD_BIT   | xxxx
  //   7      Z_DIRECTION_BIT      |  CONTROL_CYCLE_START_BIT |	xxxx
  //   8      A_DIRECTION_BIT*     |  CONTROL_SAFETY_DOOR_BIT |	xxxx
  //   9       UART                |  SPINDLE_PWM_BIT  T4/CH4 | xxxx
  //   10      UART                |  X_LIMIT_BIT			  | xxxx
  //   11      USB    T1/CH4       |  Y_LIMIT_BIT			  | xxxx
  //   12      USB                 |  Z_LIMIT_BIT         	  | xxxx
  //   13      swdio ATC M6        |  A_LIMIT_BIT*			  | LEDBLINK (test running app)
  //   14	   swdclk ATC Tn   	   |  B_LIMIT_BIT*			  | UART/USB switch
  //   15      B_DIRECTION_BIT*    |  DC Motor fault     	  | STEPPERS_DISABLE_BIT


/*
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

//#endif
