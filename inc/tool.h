/*
  tool_control.h - spindle control methods
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

#ifndef tool_h
#define tool_h

#define TOOL_NO_SYNC     false
#define TOOL_FORCE_SYNC  true

#define TOOL_T_DISABLE   0  // Must be zero
#define TOOL_M6_DISABLE  1  //
#define TOOL_T_ENABLE    bit(1) // tool is being changed
#define TOOL_M6_ENABLE   bit(2) // tool valve is enable


// Initializes tool control pins.
void tool_init();

// Returns current tool output state. Overrides may alter it from programmed state.
uint8_t tool_get_state();

// Immediately disables tool pins.
void tool_stop();
// stepper ISR timing for duration of the toggle Tn pulse
void tool_stop_tn();
// stepper ISR timing for duration of the M6 command
void tool_stop_m6();

// Sets the tool pins according to state specified.
void tool_set_m6_state(uint8_t mode);

void tool_set_t_state(uint8_t mode);

// G-code parser entry-point for setting tool states. Checks for and executes additional conditions.
void tool_t_sync(uint8_t mode);
void tool_m6_sync(uint8_t mode);

#endif
