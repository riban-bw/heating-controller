heating-controller
==================

Central heating controller code for AVR (Arduino) MCU

This code provides a central heating controller.
It is designed to run on AVR ATMega328 microcontroller, specifically Arduino MiniPro.

Dallas one-wire network of senosrs connected to (Arduino) pin defined by g_nOneWire
Heating pump relay controlled from (Arduino) pin defined by g_nPump
Boiler relay contolled from (Arduino) pin defined by g_nBoiler
DS1307 Real Time Clock I2C buss connected to (Arduino) A4 (SDA) & A5 (SCL)

Libraries used:
   Wire - I2C interface library
       TWI/I2C library for Arduino & Wiring  Copyright (c) 2006 Nicholas Zambetti.  All right reserved. GLPL 2.1
   OneWire - Dallas one wire protocol interface library
       Copyright (c) 2007, Jim Studt  (original old version - many contributors since)
       CRC code Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
   EEPROM - Access to MCU EEPROM
        Copyright (c) 2006 David A. Mellis.  All right reserved. GLPL
   ribanTimer - Timer functions without 32-bit roll-over issues
       Copyright (c) 2014, Brian Walton. All rights reserved. GLPL.

