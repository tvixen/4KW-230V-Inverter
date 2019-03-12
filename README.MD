# 230V-Inverter
With these schematics and PCB's, you can create your own 230V AC 4KW inverter.
Read Readme.pdf  how to do it.

## Implementation of an EGS002 with Ardunio Nano

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/1.jpg?raw=true "Figure")

## Introduction

The aim of this repo is to help the hobbyist or student make rapid progress in implementing an 230V inverter based on a Arduino Nano and with the use of a EGS002 PCB.

Please also note that:

 * It's assumed the reader has a basic understanding of C programming
 * It's assumed the reader has a basic understanding of electronic and how to solder
 * If you plan on making an inverter please read the safety section

## Why EGS002

First, the driver pcb. This is a known solution with EGS002.  And why EGS002 ? Because this PCB is dirt cheap on Ebay. Around 5$, 
so there’s no need for buying extra components, when this board contains half of the components we have to use, for this inverter.

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/1.jpg?raw=true "Figure")

### 1.	

Secondly, we need a host for this driver board. My own creation is this small 10x10cm PCB for 2$.

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/2.jpg?raw=true "Figure")

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/3.jpg?raw=true "Figure")

Heres the files so you can make your own PCB:

 * "Inverter Controller with SCR 1.1.zip"
 * "Power Board 3.8.zip"
 * "EMI filter.zip"
 
https://github.com/tvixen/230V-Inverter/tree/master/230V%20Inverter/doc/download


### 2.	

And for this little controller board we need a little power board. Here I made one with 4 or 8 HY4008 FET’s. Again 10x10cm.

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/4.jpg?raw=true "Figure")

Both PCB’s fabricated in China by JLCPCB.

### 3.	

Now a little EMI circuit to sort the spurious frequencies out. 
Made this filter according to the size of your toroid. PCB 10x10cm.  It’s just ferrite with some windings in the right direction.

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/5.jpg?raw=true "Figure") 

### 4.	

A cheap and reliable power supply….from Ebay 1.25$:

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/6.jpg?raw=true "Figure") 

Or one with a little more features 4$:

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/7.jpg?raw=true "Figure") 


### 5.	

The cpu for programming purposes.  Startup, shutdown, Fan control, volt , amp, watt aso: 1.99$.

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/8.jpg?raw=true "Figure") 
 

## Midifying the EGS002 

And now for the modification on EGS002. With this modification the inverter can handle current peaks. So hopefully, no more blown FET’s.

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/9.jpg?raw=true "Figure")  

With this mod, the SPWM enable pin, can enable and disable the sinewave out. (Same as the Temp method, but the right way to do it).  

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/10.jpg?raw=true "Figure")  

1. Remove the 8 components with red a cross.
2. Lift pin 7 on the cpu.
3. Mount 3 short wires, where there’s a red line.
4. Move R10 to position C30 (10k/103)


## Connections

This is how the PCB's has to be connected inside a metal case.

![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/11.jpg?raw=true "Figure") 

If the I2S is used, a second Arduino with a display connected, can be used to show the information running around in the litte Nano.
Here is my example of the colour display 240x320 I use in my old "Piller UPS" cabinet.
![Figure what](https://github.com/tvixen/230V-Inverter/blob/master/230V%20Inverter/doc/display.jpg?raw=true "Figure")  


## Brief Theory
### Basic PWM

Pulse width modulation’s (PWM) main use is to control the voltage supplied to electric circuits, it does this by rapidly switching a load on and off. Another way of thinking of it is to consider it as a method for a digital system to output an analogue signal. The figure below shows an example of a PWM signal.

![Figure 1-1](https://github.com/Terbytes/Arduino-Atmel-sPWM/blob/master/im/basicPWM_3.png?raw=true "Figure 1.1")

There are two properties to a PWM signal, the frequency which is determined by the period of the signal and the duty cycle which is determined by the high-time of the signal. The signal in figure above has a period of 250μS which means it switches at 4KHz. The duty-cycle is the percent high time in each period, in the last figure the duty-cycle is 60% because of the high-time and period of 150μS and 250μS respectively. It is the duty-cycle that determines average output voltage. In figure above the duty-cycle of 60% with 5V switching voltage results in 3V average output as shown by the red line. After filtering the output a stable analogue output can be achieved. 

### Typical microcontroller PWM implementation

This section describes how micro-controllers use timers/counters to implement a PWM signal, this description relies heavily on the figure below. Here the blue line represents a counter that resets after 16000, this gives the period of the PWM and also the switching frequency (fs), if this micro has a clock source of 16MHz, then fs will be 16×10^6/16×10^3 = 1KHz.

The two other values of 11200 and 800 shown as the gree and red line, they represent compare output values of the micro. When the counter reaches this values the micro can change the output of some pins. In this case two pins are set to HIGH when the counter resets and then Set low when the counter reaches each of the output compare values. This determines the high time and therefore duty-cycle of the PWM. It's important to understand how a PWM is typcialy set up in a micro, as the explanation of the code in the next section will not make sense otherwise.

![Figure what](https://github.com/Terbytes/Arduino-Atmel-sPWM/blob/master/im/sawtooth_counter_1.png?raw=true "Figure")

### Sinusoidal PWM

A sinusoidal PWM (sPWM) signal can be constructed by dynamically changing the duty- cycle. The result is short pulses at the zero-crossings and long pulses at the wave peaks. This can be seen in the figure below.

![Figure what](https://github.com/Terbytes/Arduino-Atmel-sPWM/blob/master/im/PWMsin_2.png?raw=true "Figure")

The figure shows negative pulses which is not possible on most micro-controllers. Instead normally this is implemented with two pins, one pulsing the positive half of the sin wave and the second pulsing the negative half, this is how it is implemented in this repo as having the signal split across two pins makes sense if the signal is going to be used to control a H-bridge.

## Safety

This section is to briefly discuss safety in regards to making an inverter that steps up to  mains voltage whether that be 110 or 230. First of all I don't encourage it, I'd prefer that you didn't and I take no responsibilty for your actions. Remember 30mA can be leathal, mains voltage deserves respect.

If you still choose to do so, take basic precautionary steps like: Invest in some terminals and make sure that any high voltage part of the circuit is not touchable; don't modify it while it's power up; Don't do it alone.
Here is an example of connections in a metal box. 
 
