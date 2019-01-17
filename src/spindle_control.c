/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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
#include "stm32f10x_gpio.h"

#ifdef VARIABLE_SPINDLE
  //static float pwm_gradient; // Pre-calculated value to speed up rpm to PWM conversions.
   float pwm_gradient;
#endif


uint16_t time_base;

//void spindle_init()
void spindle_init(uint8_t pwm_mode) // Added the pwm mode, Paul
{

//#ifdef VARIABLE_SPINDLE
////  pwm_gradient = SPINDLE_PWM_RANGE / (settings.rpm_max - settings.rpm_min);
//  pwm_gradient = (TIM_TimeBaseInitStruct.TIM_Period - 5) / (settings.rpm_max - settings.rpm_min);
//#endif

  /*
   * Author Paul
   * Initialise the Enable (CS) and Direction (CW/CWW) ports for the spindle component
   * These need to be programmed first before the timer. Other way around does not initialise
   * the direction, enable and timer functions...
   */
  GPIO_InitTypeDef GPIO_InitStructureControl;
#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
// configure the spin enable port only
	RCC_APB2PeriphClockCmd(RCC_SPINDLE_ENABLE_PORT, ENABLE);
	GPIO_InitStructureControl.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructureControl.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_InitStructureControl);
    GPIO_InitStructureControl.GPIO_Pin =  1 << SPINDLE_ENABLE_BIT | 1 << SPINDLE_DIRECTION_BIT;
    GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_InitStructureControl);
    ResetSpindleEnablebit();

  #else
 // combination of 2 ports used: enable and direction
	RCC_APB2PeriphClockCmd(RCC_SPINDLE_ENABLE_PORT, ENABLE);
	GPIO_InitStructureControl.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructureControl.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_InitStructureControl);
    GPIO_InitStructureControl.GPIO_Pin =   1 << SPINDLE_ENABLE_BIT;
    GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_InitStructureControl);
    // debug
    GPIO_PinLockConfig(SPINDLE_ENABLE_PORT,SPINDLE_ENABLE_BIT);


	RCC_APB2PeriphClockCmd(RCC_SPINDLE_DIRECTION_PORT, ENABLE);
	GPIO_InitStructureControl.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructureControl.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructureControl.GPIO_Pin =  1 << SPINDLE_DIRECTION_BIT;
	GPIO_Init(SPINDLE_DIRECTION_PORT, &GPIO_InitStructureControl);
	// debug
	GPIO_PinLockConfig(SPINDLE_DIRECTION_PORT,SPINDLE_DIRECTION_BIT);

	ResetSpindleEnablebit();
	ResetSpindleDirectionBit();
#endif

#if defined (STM32F103C8)
/*
 * Author Paul
 */

    RCC_APB2PeriphClockCmd(RCC_SPINDLE_ENABLE_PORT|RCC_SPINDLE_PWM_PORT|RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    //RCC->APB1ENR |= RCC_APB1Periph_TIM4;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	//M3, M4, M5 working with this step enable code
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;

	/*
	 * old logical Arduino Grbl bug, port mode is set to PP_AF for timer while it has to be GPIO_Mode_Out_PP
	 * for the enable and direction pins!!
	 */
//#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
//  //GPIO_InitStructure.GPIO_Pin = 1 << SPINDLE_ENABLE_BIT; // configure the spin enable port
//  GPIO_InitStructure.GPIO_Pin = 1 << SPINDLE_DIRECTION_BIT;//
//#else
//  GPIO_InitStructure.GPIO_Pin = 1 << SPINDLE_DIRECTION_BIT | 1 << SPINDLE_ENABLE_BIT | 1 << SPINDLE_PWM_BIT;
//#endif
//  GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_InitStructure);
//#endif

#ifdef VARIABLE_SPINDLE

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC->APB1ENR |= RCC_APB1Periph_TIM4;

  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Disable);
  TIM_InternalClockConfig(TIM4);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;

  switch (pwm_mode) {
    case 0://60 Hz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 60000 - 1; //default setting medium freq 60Hz. 1000 pwm steps
    	TIM_TimeBaseInitStruct.TIM_Period= 999;
      break;
    case 1://125 Hz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 125000 - 1; //dither mode low freq 125Hz
    	TIM_TimeBaseInitStruct.TIM_Period = 999;
      break;
    case 2://250Hz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 250000 - 1; //smooth high freq 250Hz
    	TIM_TimeBaseInitStruct.TIM_Period = 999;
      break;
    case 3://500Hz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 500000 - 1; //ultra smooth highest freq 500Hz
    	TIM_TimeBaseInitStruct.TIM_Period = 999;
      break;

    case 4://1kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 1000000 - 1; //default setting medium freq 1kHz
    	TIM_TimeBaseInitStruct.TIM_Period = 999;
      break;

    case 5://1.5kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 1500000 - 1; //default setting medium freq 1.5Kc
    	TIM_TimeBaseInitStruct.TIM_Period = 999;
      break;
    case 6://3kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 300000 - 1; //smooth high freq 250Hz
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    case 7://4.5kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 4500000 - 1; //default setting medium freq 4.5kC
    	TIM_TimeBaseInitStruct.TIM_Period = 999;
      break;
    case 8://6kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 600000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    case 9://8kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 800000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
    case 10://10kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 1000000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    case 11://15kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 1500000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    case 12://30kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 3000000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    case 13://45kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 4500000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    case 14://60kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 6000000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    case 15://80kHz
    	TIM_TimeBaseInitStruct.TIM_Prescaler = F_CPU / 7500000 - 1;
    	TIM_TimeBaseInitStruct.TIM_Period = 99;
      break;
    default:
     break;
  }
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  /*
   * author Paul
   */

  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_ARRPreloadConfig(TIM4,ENABLE);
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; // 0 counting up mode
  TIM_OCInitStruct.TIM_Pulse = 0;     // initit speed is 0
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High; // Paul, TIM_OCPolarity_High for SG, Low for Mini Gerbil
  TIM_OC4Init(TIM4, &TIM_OCInitStruct);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_Cmd(TIM4,ENABLE);

  RCC_APB2PeriphClockCmd(RCC_SPINDLE_PWM_PORT, ENABLE); // enable the clock

  GPIO_InitTypeDef gpio_InitStructure;
  GPIO_StructInit(&gpio_InitStructure);
  gpio_InitStructure.GPIO_Pin = SPINDLE_PWM_BIT;
  gpio_InitStructure.GPIO_Speed =  GPIO_Speed_2MHz;
  gpio_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPINDLE_PWM_PORT, &gpio_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  TIM_Cmd(TIM4,ENABLE);
  // Does we want to lock in the PWM pin forever?
  //GPIO_PinLockConfig(SPINDLE_PWM_PORT, SPINDLE_PWM_BIT);

  GPIO_ResetBits(SPINDLE_PWM_PORT, SPINDLE_PWM_BIT);

#ifdef VARIABLE_SPINDLE
//  pwm_gradient = SPINDLE_PWM_RANGE / (settings.rpm_max - settings.rpm_min);
  pwm_gradient = (TIM_TimeBaseInitStruct.TIM_Period - 5) / (settings.rpm_max - settings.rpm_min);
  time_base = TIM_TimeBaseInitStruct.TIM_Period;
#endif

#endif

#endif


  spindle_stop();
}


uint8_t spindle_get_state()
{
  uint8_t pin = 0;
	#ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
#ifdef AVRTARGET
  pin = SPINDLE_ENABLE_PORT;
#endif
#if defined (STM32F103C8)
 //pin = SPINDLE_ENABLE_PORT; //GPIO_ReadInputData(SPINDLE_ENABLE_PORT);
  pin = GPIO_ReadInputData(SPINDLE_ENABLE_PORT);
//  printString("\n\l spindle state = ");
//  printInteger(pin);
#endif
		  // No spindle direction output pin. 
		#ifdef INVERT_SPINDLE_ENABLE_PIN
			  if (bit_isfalse(pin,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
	    #else
	 			if (bit_istrue(pin,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
	    #endif
    #else
#ifdef AVRTARGET
  pin = SPINDLE_DIRECTION_PORT;
    if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) 
#endif
#if defined (STM32F103C8)
      pin = GPIO_ReadInputData(SPINDLE_DIRECTION_PORT);
#endif
     {
        if (pin & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      }
    #endif
	#else
#ifdef AVRTARGET
  pin = SPINDLE_ENABLE_PORT;
#endif
#if defined (STM32F103C8)
  pin = GPIO_ReadInputData(SPINDLE_ENABLE_PORT);
#endif
		#ifdef INVERT_SPINDLE_ENABLE_PIN
		  if (bit_isfalse(pin,(1<<SPINDLE_ENABLE_BIT))) {
		#else
		  if (bit_istrue(pin,(1<<SPINDLE_ENABLE_BIT))) {
		#endif
		  if (pin & (1 << SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
		  else { return(SPINDLE_STATE_CW); }
		}
	#endif
	return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
#ifdef VARIABLE_SPINDLE
	#ifdef AVRTARGET
		SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
	#endif
	#if defined (STM32F103C8)

		//TIM_SetCompare4(TIM4, settings.rpm_max - SPINDLE_PWM_OFF_VALUE);
		TIM_SetCompare4(TIM4, 0);
	#endif


	#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
		  #ifdef INVERT_SPINDLE_ENABLE_PIN
			SetSpindleEnablebit();
		  #else
			ResetSpindleEnablebit();
		  #endif
	//    #endif //Pauls bug fix of original code...
	#else
		#ifdef INVERT_SPINDLE_ENABLE_PIN
		  SetSpindleEnablebit();

		#else
		  ResetSpindleEnablebit();

		#endif
	#endif
#endif
}

/*
 * Author Paul
 */
void differentiate_spindle_speed(SPINDLE_PWM_TYPE set_pwm_value, SPINDLE_PWM_TYPE pwm_value)
{
 if ( set_pwm_value > ((TIM4->CCR4) *1.25)){
	while (set_pwm_value > (TIM4->CCR4) ){
		 TIM_SetCompare4(TIM4, ((TIM4->CCR4)+1));
         if (settings.soft_start >= 300){
        	 delay_ms(300);
         } else {
		 delay_ms(settings.soft_start);
         }
	 }
 } else {
		 TIM_SetCompare4(TIM4, set_pwm_value);
	 }

}

#ifdef VARIABLE_SPINDLE
  // Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
  // and stepper ISR. Keep routine small and efficient.
  void spindle_set_speed(SPINDLE_PWM_TYPE pwm_value)
  {
#ifdef AVRTARGET
		SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
#endif
#if defined (STM32F103C8)
		TIM_SetCompare4(TIM4, pwm_value);

#endif
		#ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
     if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        spindle_stop();
      } else {
				#ifdef AVRTARGET
					SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
				#endif
				#if defined (STM32F103C8)
						TIM_CtrlPWMOutputs(TIM1, ENABLE);
				#endif
				#ifdef INVERT_SPINDLE_ENABLE_PIN
					ResetSpindleEnablebit();
				#else
					SetSpindleEnablebit();
				#endif
		 }
		#else
			if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
			#ifdef AVRTARGET
				SPINDLE_TCCRA_REGISTER &= ~(1 << SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
			#endif
			#if defined (STM32F103C8)

				TIM_SetCompare4(TIM4, pwm_value);

			#endif
			} else {
			#ifdef AVRTARGET
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
			#endif
			#if defined (STM32F103C8)

                 TIM_SetCompare4(TIM4, pwm_value); // 13/08/2018

				#ifdef INVERT_SPINDLE_ENABLE_PIN
					ResetSpindleEnablebit(); // Turn Spindle enable On (0 Volt)
				#else
					SetSpindleEnablebit(); // Turn Spindle enable On (5V)
				#endif
			#endif
			}
		#endif
	}
#endif

SPINDLE_PWM_TYPE spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
	{
		SPINDLE_PWM_TYPE pwm_value;
		rpm *= (0.010f*sys.spindle_speed_ovr); // Scale by spindle speed override value.
	// Calculate PWM register value based on rpm max/min settings and programmed rpm.
		if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
			// No PWM range possible. Set simple on/off spindle control pin state.
			sys.spindle_speed = settings.rpm_max;
			//pwm_value = SPINDLE_PWM_MAX_VALUE;
			pwm_value = time_base;
		}
		else if (rpm <= settings.rpm_min) {
			if (rpm == 0.0f) { // S0 disables spindle
				sys.spindle_speed = 0.0f;
				pwm_value = SPINDLE_PWM_OFF_VALUE;
			}
			else { // Set minimum PWM output
				sys.spindle_speed = settings.rpm_min;
				pwm_value = SPINDLE_PWM_MIN_VALUE;
			}
		}
		else {
			// Compute intermediate PWM value with linear spindle speed model.
			// NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
			sys.spindle_speed = rpm;

			pwm_value = (SPINDLE_PWM_TYPE)floorf((rpm - settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
		}
//		printFloat_RateValue(Command);

		return(pwm_value);
	}


// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
  void spindle_set_state(uint8_t state, float rpm)
#else
  void _spindle_set_state(uint8_t state)
#endif
{
  if (sys.abort) { return; } // Block during abort.
  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
  
    #ifdef VARIABLE_SPINDLE
      sys.spindle_speed = 0.0f;
    #endif
    spindle_stop();
  
  } else {
    #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
      if (state == SPINDLE_ENABLE_CW) {
        ResetSpindleDirectionBit();
	  }
	  else {
      SetSpindleDirectionBit();
      }
    #endif
  
    #ifdef VARIABLE_SPINDLE
      // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
      if (settings.flags & BITFLAG_LASER_MODE) {
        if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0f; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
      }
/*
 * Author Paul; soft start feature
 */

      if (settings.soft_start == 0){
      spindle_set_speed(spindle_compute_pwm_value(rpm));}
      else{
    	  current_pwm = TIM4->CCR4 ;

          differentiate_spindle_speed(spindle_compute_pwm_value(rpm),current_pwm);
      }


		#endif
		#ifdef INVERT_SPINDLE_ENABLE_PIN // Paul, bug fix of orginal code
		  ResetSpindleEnablebit();
		#else
		  SetSpindleEnablebit();
		#endif
    #if (defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && \
        !defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)) || !defined(VARIABLE_SPINDLE)
      // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
      // if the spindle speed value is zero, as its ignored anyhow.
//      #ifdef INVERT_SPINDLE_ENABLE_PIN
//        ResetSpindleEnablebit();
//      #else
//        SetSpindleEnablebit();
//      #endif
    #endif
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
  void spindle_sync(uint8_t state, float rpm)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(uint8_t state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    _spindle_set_state(state);
  }
#endif


