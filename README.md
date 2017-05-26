Extremely Lost Cost LCR Meter

1 Overview

The goal of this project is design a system capable of measuring resistance, inductance (and ESR), and capacitance. A design goal of this project is to limit the total cost of the daughterboard and components added to the TM4C123GXL evaluation board to $2.50 in 10k quantities.

The project shall provide a complete user interface through the virtual COM port on the evaluation board.

2 Hardware Description

Microcontroller:

An ARM M4F core (TM4C123GH6PMI microcontroller) is required.

Power LED:

A power LED must be connected through a current-limiting resister to indicate the daughterboard has power.

Serial interface:

If using the EK-TM4C123GXL evaluation board, then the UART0 tx/rx pair is routed to the ICDI that provides a virtual COM port through a USB endpoint.

LCR measurement interface:

A circuit is provided that will interface with the microcontroller and allow the user to test an L, C, or R value. The output of this circuit can be connected to the analog comparator and analog-to-digital converter inputs.

3.3V supply:

The circuit is powered completely from the 3.3V regulator output on the evaluation board.

DUT connection;

Two connectors, made of wire loop to save cost, are required to allow the DUT to be connected.

Test points:

Test points shall be added for the ground reference and comparator output at minimum.
 
3 Suggested Parts List			
				
	Part			Quantity
			
	2N3904 NPN transistor	5
			
	2N3906 PNP transistor	2
			
	33ohm, 1/2W resistor	1
			
	3.3kohm, 1/4W resistor	7
			
	10kohm 1/4W resistor	7
			
	100kohm, 1/4W resistor	1
			
	1N5819 Schottky diode (flybacks)	4
			
	1uF capacitor (integrator)	1
			
	47uF capacitor (power supply)	1
			
	2x10 double-row header, unshrouded	2
			
	Wire (22-24 AWG solid wire, 3+ colors)	1
			
	PC board (approx 4.5x6”)	1
				
	ST-7565R based graphics LCD and parts			Optional
				
	Tools, safety glasses, …			1 each
				


4 Software Description

A virtual COM power using a 115200 baud, 8N1 protocol with no hardware handshaking shall be provided with support to the following commands.

Debug:

If “reset” is received, the hardware shall reset.

If “voltage” is received, the hardware shall return the voltage across DUT2-DUT1. The voltage is limited to 0 to 3.3V.

LCR commands:

If “resistor” is received, return the capacitance of the DUT.

If “capacitance” is received, return the capacitance of the DUT.

If “inductance” is received, return the inductance of the DUT.

If “esr” is received, return the ESR of the inductor under test.

If “auto” is received, return the value of the DUT that is most predominant (i.e. an inductor with 1ohm ESR and 10mH inductance will return the inductance and ESR values, a 100kohm resistor will return the resistance, and a 10uF capacitor will return the capacitance.
