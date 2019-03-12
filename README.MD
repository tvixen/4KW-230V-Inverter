# 230V-Inverter
With these schematics and PCB's, you can create your own 230V AC 4KW inverter.
Read Readme.pdf  how to do it.

Here’s my small low cost 4KW inverter. 
Under the “230V Inverter\Advanced” tab, you will find a “how to” with PCB’s and schematics. 
1.	A controller 
2.	A power stage
3.	A EMI ac filter
4.	Low cost Power supply

First, the driver pcb. This is a known solution with EGS002.  And why EGS002 ? Because this PCB is dirt cheap on Ebay. Around 5$, so there’s no need for buying extra components, when this board contains half of the components we have to use, for this inverter.


Here’s my small low cost 4KW inverter. 
Under the “230V Inverter\Advanced” tab, you will find a “how to” with PCB’s and schematics. 
1.	A controller 
2.	A power stage
3.	A EMI ac filter
4.	Low cost Power supply

First, the driver pcb. This is a known solution with EGS002.  And why EGS002 ? Because this PCB is dirt cheap on Ebay. Around 5$, so there’s no need for buying extra components, when this board contains half of the components we have to use, for this inverter.
https://github.com/tvixen/230V-Inverter/tree/master/230V%20Inverter/doc/1.jpg 
1.	Secondly, we need a host for this driver board. My own creation is this small 10x10cm PCB for 2$.
https://github.com/tvixen/230V-Inverter/tree/master/230V%20Inverter/doc/2.jpg 

 

2.	And for this little controller board we need a little power board. Here I made one with 4 or 8 HY4008 FET’s. Again 10x10cm.
https://github.com/tvixen/230V-Inverter/tree/master/230V%20Inverter/doc/3.jpg 

Both PCB’s fabricated in China by JLCPCB.





3.	Now a little EMI circuit to sort the spurious frequencies out. 
Made this filter according to the size of your toroid. PCB 10x10cm.  It’s just ferrite with some windings in the right direction.
 

4.	A cheap and reliable power supply….from Ebay 1.25$:
 
Or one with a little more features 4$:
 


5.	The cpu for programming purposes.  Startup, shutdown, Fan control, volt , amp, watt aso: 1.99$.
 
 
And now for the modification on EGS002. With this modification the inverter can handle current peaks. So hopefully, no more blown FET’s.
 

With this mod, the SPWM enable pin, can enable and disable the sinewave out. (Same as the Temp method, but the right way to do it).  
 
1. Remove the 8 components with red a cross.
2. Lift pin 7 on the cpu.
3. Mount 3 short wires, where there’s a red line.
4. Move R10 to position C30 (10k/103)

Here is an example of connections in a metal box. 
 
