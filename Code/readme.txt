2016 - CS308  Group X : Project README TEMPLATE
================================================

Group Info:
------------
+   Pratik Fegade (120050004)
+   Devdeep Ray (120050007)
+   Haren Saga (120050072)

Extension Of
------------

Launchpad-Mame-Control (https://github.com/Rhys79/Launchpad-Mame-Control)

Project Description
-------------------

This is a readme template. It is written using markdown syntax. To know more about markdown you can always refer to [Daring Fireball](http://daringfireball.net/projects/markdown/basics).
You can preview how your markdown looks when rendered [here](http://daringfireball.net/projects/markdown/dingus)

Students are requested to use this format for the sake of uniformity and convenience. Also we can parse these files and then index them for easy searching.

Technologies Used
-------------------

Remove the items that do no apply to your project and keep the remaining ones.

+   Embedded C

Installation Instructions
=========================

The Microcontroller Code folder has the following structure.

└── Microcontroller\ Code
    ├── GloveProject
    │   ├── Mame_pins.c
    │   ├── Mame_pins.h
    │   ├── targetConfigs
    │   │   ├── Tiva\ TM4C123GH6PM.ccxml
    │   │   └── readme.txt
    │   ├── tm4c123gh6pm.cmd
    │   ├── tm4c123gh6pm_startup_ccs.c
    │   ├── uartstdio.c
    │   ├── uartstdio.h
    │   ├── usb_dev_mame.c
    │   ├── usb_mame_structs.c
    │   └── usb_mame_structs.h
    ├── usb-ids.h
    ├── usbhid.h
    ├── usbhidmame.c
    └── usbhidmame.h


The instructions to modify and build the code provided are as follows.
1) GloveProject is a CCS project that one can import in CCS directly.
2) Replace usb-ids.h and usbhid.h in the \ti\TivaWare_C_Series-1.1\usblib with the corresponding files above.
3) Place usbdhidmame.c and usbdhidmame.h from the above into the \ti\TivaWare_C_Series-1.1\usblib\device folder.

Demonstration Video
=========================
The link to the folder with the two videos (the screen cast video and the demo video) is given below:
https://drive.google.com/folderview?id=0B6ROgI8ltLfAUENHODVvek1KSkk&usp=sharing


References
===========

Please give references to importance resources.

+   USB.org - HiD: http://www.usb.org/developers/hidpage/
+   I2C Communication with the TI Tiva TM4C123GXL:
+   https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
+   MPU 9250 Datasheet: https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf
+   MPU 9250 Register Map: https://strawberry-linux.com/pub/RM-MPU-9250A-00.pdf
+   https://github.com/Rhys79/Launchpad-Mame-Control
+   http://gamasutra.com/db_area/images/blog/234638/Games%20Revenue%20Forecast.PNG
+   Optical flex sensors: http://hackaday.com/2011/10/21/building-optical-flex-sensors/
+   Optical flex sensor (US 4542291 A): http://www.google.co.in/patents/US4542291

