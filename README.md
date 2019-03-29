# SuperGerbil
STM32 driven ARM Grbl firmware
![GitHub Logo](https://awesome.tech/wp-content/uploads/2018/11/20181030_103220-1-e1547624499622.jpg)

***
_Click the `Release` tab to download pre-compiled `.hex` files or just [click here](https://awesome.tech/downloads/)_
***
For the insights of ARM Grbl, please visit this page [click here] https://awesome.tech/grbl-demystified/

-Github repo grbl-1 is for Coocoox CoIde compiler
-Github repo grbl-2 is for Atollic compiler

Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. This version of Grbl runs on a STM32F10x processor. For the appropriate boards see our website https://awesome.tech/buy-gerbil/

The controller is written in highly optimized C utilizing every clever feature of the STM32-chips to achieve precise timing and asynchronous operation. It is able to maintain up to 100kHz of stable, jitter free control pulses and PWM up to 80kHz. See specifications [click here] https://awesome.tech/new-super-gerbil-cnc/

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and most canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 16 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

* [Licensing](https://github.com/gnea/grbl/wiki/Licensing): Grbl is free software, released under the GPLv3 license.

* For more information and help, check out our **[Wiki pages!](https://github.com/gnea/grbl/wiki)** and (https://awesome.tech) If you find that the information is out-dated, please to help us keep it updated by editing it or notifying our community! Thanks!

* Lead Developer Gnea: Sungeun "Sonny" Jeon, Ph.D. (USA) aka @chamnit

* Lead Developer Awesome.tech Super Gerbil: Paul "Laserboy" de Groot (AU) aka @paulusjacobus

* Built on the wonderful Grbl v0.6 (2011) AND v1.1f firmware written by Simen Svale Skogsrud (Norway).

***

-------------
Grbl is an open-source project and fueled by the free-time of our intrepid administrators and altruistic users. If you'd like to donate to @chamnit, all proceeds will be used to help him fund supporting hardware and testing equipment or support us Awesome.tech by buying our boards. Thank you!

[![Donate](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=CUGXJHXA36BYW)
