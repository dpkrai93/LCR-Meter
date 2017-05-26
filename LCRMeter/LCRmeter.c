//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include <string.h>
#include <strings.h>
#include "tm4c123gh6pm.h"
#include <math.h>


#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

#define PA_2         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))      //INTEGRATE
#define PA_3         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))      //LOWSIDE_R
#define PA_4         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))      //MEAS_C
#define PA_5         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))      //MEAS_LR
#define PE_4         (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))      //HIGHSIDE_R

#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))

typedef enum state{output_DC,output_Sine,output_Sweep,output_voltage} State;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void waitMicrosecond(uint32_t us)
{
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
	__asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}



// Initialize Hardware
void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD;

    //Control Signal Configuration
    GPIO_PORTE_DIR_R = 0x10;
    GPIO_PORTE_DR2R_R = 0x10;
    GPIO_PORTE_DEN_R = 0x10;
    GPIO_PORTA_DIR_R = 0x3C;
    GPIO_PORTA_DR2R_R = 0x3C;
    GPIO_PORTA_DEN_R |= 0x3C;

    //PortF Configuration
    GPIO_PORTF_DIR_R |= 0x08;
    GPIO_PORTF_DR2R_R |= 0x08;
    GPIO_PORTF_DEN_R |= 0x08;

    //Unlock PIN F0
    GPIO_PORTF_LOCK_R |= 0x4C4F434B;
    GPIO_PORTF_CR_R |= 0x01;
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF0_C0O;
    GPIO_PORTF_AFSEL_R |= 0x01;
    GPIO_PORTF_DEN_R |= 0x01;
    GPIO_PORTF_DIR_R |= 0x01;
    GPIO_PORTD_AFSEL_R |= 0x40;
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD6_M;
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
    GPIO_PORTD_DEN_R |= 0x40;

    //PortA UART0 Pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    GPIO_PORTA_DEN_R |= 3;
    GPIO_PORTA_AFSEL_R |= 3;
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;
    UART0_CC_R = UART_CC_CS_SYSCLK;
    UART0_IBRD_R = 21;
    UART0_FBRD_R = 45;
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

    // Configure AN0 as an analog input
    SYSCTL_RCGCADC_R |= 1;
    GPIO_PORTE_AFSEL_R |= 0x08;
    GPIO_PORTE_DEN_R &= ~0x08;
    GPIO_PORTE_AMSEL_R |= 0x08;
    ADC0_CC_R = ADC_CC_CS_SYSPLL;
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;
    ADC0_SSMUX3_R = 0;
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;

    //Initialization of Comparator
    SYSCTL_RCGCACMP_R = SYSCTL_RCGCACMP_R0;
    GPIO_PORTC_AFSEL_R = 0xC0;
    GPIO_PORTC_DEN_R &= ~0xC0;
    GPIO_PORTC_AMSEL_R |= 0xC0;
    GPIO_PORTC_DIR_R &= ~0xC0;
    COMP_ACREFCTL_R |= COMP_ACREFCTL_EN | COMP_ACREFCTL_VREF_M;
    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_CINV;
    waitMicrosecond(10);

    //Set Timer Mode
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER5_CFG_R = 4;
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;
    WTIMER5_TAV_R = 0;
    //WTIMER5_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);
}

int16_t readAdc()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}



char stringg[81]={0};
char str1[81]={0};
char str2[81]={0};
char str3[10],str4[20],str5[20],str6[20],str7[20];
char field_type[3];
char index1[81];
char index2[81];
int field_count=0;
int field_index[3]={0};
int freq,freq3=0;
uint32_t time = 0;
uint32_t phase=0,delta_phase;
uint32_t in,val;
uint32_t index3,value;
uint32_t table[4096],l,k,i;
uint16_t volt1;
char mode;
float pi=3.1415,amp=0,amp1=0,duty=0;
float volt,instantVolt,iirVolt;
float alpha = 0.99;
int firstUpdate = true;
float freq_temp=0,raw,instant_adc=0,freq1=0,freq2=0;
char vtg_temp[10],fre_temp[10];
uint8_t count=2;
char other[10],temp_other[10],function[10];
float res,cap,ind;

char step2(char *stringg)
	{
			int count=0;
			char c;

			for(i=0;i<81;i++)
			{
				stringg[i]=NULL;
				str1[i]=NULL;
				str2[i]=NULL;
			}
			while(1)

			{

				 c = getcUart0();

					if(c==0x08)
					{
							if (count==0)
							{
								continue;
							}
							else
							{
								count--;
								continue;
							}
					}
					if(c==0x0D)
					{
						stringg[count]='\0';
						break;
					}

					else if(c >= 0x20)
					{

						stringg[count]=c;
						count++;

							if (count==80)
							{
								stringg[count++]='\0';
								break;
							}
							else
							{
								continue;
							}

					}

				}
			//putsUart0(stringg);
			return 0;

	}



	void step3(char stringg[])
	{

		int a=0,b=0;
		field_count=0;
		char str2[81]={0};


		char index1[81]={0};
		char index2[81]={20};





		for(i=0; i<strlen(stringg); i++)
			{
				if (stringg[i]>=0x41 && stringg[i]<=0x5A || stringg[i]>=0x61 && stringg[i]<=0x7A ||stringg[i]==0x5F)
				{
					str1[i]=stringg[i];
					str2[i]='c';
				}

				else if ((stringg[i]>=0x30 && stringg[i]<=0x39)|| stringg[i]==0x2E ||stringg[i]==0x2B ||stringg[i]==0x2D ||stringg[i]==0x5F )
				{
					str1[i]=stringg[i];
					str2[i]='n';
				}

				else
					{
					str1[i]='\0';
					str2[i]='\0';
					}
			}

		for(i=0; i<80; i++)
			{


					index1[b]=str2[i];
					index2[b]=str2[i+1];

					if(index1[b]!=index2[b] && index2[b]!='\0')
					{
							field_index[a]=i+1;
							field_type[a]=index2[b];
							(field_count)++;
							a++;

					}

					b++;
			}

	//	putsUart0(str1);
	//	putsUart0(str2);


	}

void wait5Isr()
{
	time=WTIMER5_TAV_R;                        // read counter input
	WTIMER5_TAV_R=0;                           // zero counter for next edge
	time /= 40;                                  // scale to us units
	sprintf(str4, "%u", time);
	putsUart0("\r\n");
	putsUart0(str4);
	putsUart0("\r\n");
	WTIMER5_ICR_R=TIMER_ICR_CAECINT;           // clear interrupt flag
}
void inductance()
{
	int firstUpdate=true;
	uint16_t raw;
	float instantVolt,iirVolt,Meas_Esr,ind;
	float alpha=0.99;
	char volt_val[20],Esr_val[20],induct[20];

		PE_4=0;
		PA_2=0;
		PA_4=0;  //measC
		PA_3=1;  //low
		PA_5=1;  //measLR
		WTIMER5_TAV_R=0;
		waitMicrosecond(300000);
		raw=readAdc();
		instantVolt=(((raw/4096.0)*3.3)-0.02-0.05);
		if(firstUpdate)
		{
			iirVolt=instantVolt;
			firstUpdate=false;
		}
		else
		{
			iirVolt=iirVolt*alpha+instantVolt*(1-alpha);
		}

		Meas_Esr=((33*(3.3+0.034)/iirVolt)-33)-2;

		if(time<10)
		{
			ind=((time*(33+Meas_Esr))/1.299)-10;
		}
		else
		{
			ind=((time*(33+Meas_Esr))/1.299)-320;
		}
		waitMicrosecond(3000);
		sprintf(volt_val, "Voltage: %f", instantVolt);
		putsUart0(volt_val);
		putsUart0(" Volts\n\r");
		sprintf(Esr_val, "ESR: %7f\n\r", Meas_Esr);
		putsUart0(Esr_val);
		sprintf(induct, "L: %7f", ind);
		putsUart0(induct);
		putsUart0(" uH\n\r");
		PA_3=0; //low
		PA_5=0; //measLR
		waitMicrosecond(7000000);
}

void Auto()
{
	uint16_t raw;
	float instantVolt;
	PA_5=0; //measLR
	PA_4=1; //measC
	PA_3=1; //low
	waitMicrosecond(3000);
	PA_3=0; //low
	PE_4=1; //high
	waitMicrosecond(5000000);
	raw=readAdc();
	instantVolt=(((raw/4096.0)*3.3)-0.02-0.05);
	if(instantVolt>=2.2)
	{
		putsUart0("Capacitor Detected.\n\r");
		char Measc[10];
		uint32_t i;
		float cap;
		PA_5=0; //measLR
		PA_4=1; //measC
		for(i=0;i<=5;i++)
		{
			PA_3=1; //low
			waitMicrosecond(3000);
			PA_3=0; //low
			WTIMER5_TAV_R=0;
			PE_4=1; //high
			cap=time/(1.299*100000);
			waitMicrosecond(2500000);
			PE_4=0; //high
			waitMicrosecond(3000);
		}
		sprintf(Measc, "Capacitance: %f", cap);
		putsUart0(Measc);
		putsUart0(" uF\n\r");
	}
	else
	{
		PA_4=0; //measC
		PA_3=1; //low
		PA_5=1; //measLR
		waitMicrosecond(5000000);
		raw=readAdc();
		instantVolt=(((raw/4096.0)*3.3)-0.02-0.05);
		PA_3=0; //low
		PA_5=0; //measLR
		if(instantVolt>=2.4)
		{
			putsUart0("Inductor Detected.\n\r");
			int firstUpdate=true;
			uint16_t raw;
			float instantVolt, iirVolt;
			float alpha = 0.99,Meas_Esr,ind;
			char volt_val[20],Esr_val[20],induct[20];

			PA_4=0; //measC
			PA_3=1; //low
			PA_5=1; //measLR
			WTIMER5_TAV_R=0;
			waitMicrosecond(300000);
			raw=readAdc();
			instantVolt=(((raw/4096.0)*3.3)-0.02-0.05);
			if(firstUpdate)
			{
				iirVolt=instantVolt;
				firstUpdate=false;
			}
			else
			{
				iirVolt=iirVolt*alpha+instantVolt*(1-alpha);
			}
			Meas_Esr=((33*(3.3+0.034)/iirVolt)-33)-2;
			if(time<10)
			{
				ind=((time*(33+Meas_Esr))/1.299)-10;
			}
			else
			{
				ind=((time*(33+Meas_Esr))/1.299)-320;
			}
			waitMicrosecond(3000);

			sprintf(volt_val, "Voltage: %f", instantVolt);
			putsUart0(volt_val);
			putsUart0("Volts\n\r");
			sprintf(Esr_val, "ESR: %7f\n\r", Meas_Esr);
			putsUart0(Esr_val);
			sprintf(induct, "L: %7f", ind);
			putsUart0(induct);
			putsUart0(" uH\n\r");
			PA_3=0; //low
			PA_5=0; //measLR
			waitMicrosecond(7000000);

		}
		else
		{
			putsUart0("Resistor Detected.\n\r");
			char res_val[10];
			uint32_t res,i;
			PA_2=0; //int
			PA_3=0; //low
			PA_4=0; //measR
			PA_5=0; //measLR
			PE_4=0; //high
			PA_2=1; //int
			for(i=0;i<=5;i++)
			{
				PA_3=1; //low
				waitMicrosecond(1010);
				PA_3=0; //low
				WTIMER5_TAV_R=0;
				PA_5=1; //measLR
				res=time/1.299;
				waitMicrosecond(500000);
				PA_5=0; //measLR
				waitMicrosecond(1010);

			 }
			sprintf(res_val, "Resistance: %7lu", res);
			putsUart0(res_val);
			putsUart0(" Ohms\n\r");
		}
	}

}




float getNumber(int field_number)
			{
			 	float number=0;
				number = atof(&stringg[field_index[field_number-1]]);
				return number;
			}




_Bool iscommand(char *match_string, int min_arg)
		{

			if ((strcasecmp(&str1[0], match_string)==0) && (field_count >= min_arg))
				return 1;
			else
				return 0;
		}

void otherstring(int field_number)
			{
				for(i=0;i<10;i++)
				{
					other[i]=0;
				}
				strcpy(other,(&str1[field_index[field_number-1]]));
			}



void step4()
{


	if(iscommand("reset",0))
	{
		__asm("    .global _c_int00\n"
			  "    b.w     _c_int00");				//ResetIsr
	}

	else	if(iscommand("voltage",0))
	{
		PA_5=0;
		PA_4=1;
		waitMicrosecond(500000);
		volt1=readAdc();
		instantVolt=(10.45/2)*(volt1+0.5)/4096;

		iirVolt= instantVolt;

		sprintf(str3, "%3.1f", iirVolt);
		putsUart0(str3);
		iirVolt=0;
	}
	else if(iscommand("io",2))
	{
		otherstring(1);
		strcpy(temp_other,other);
		otherstring(2);
		strcpy(function,other);
		//char MEAS_LR[]="MEAS_LR";
		if(!(strcasecmp("MEAS_LR",temp_other)))
		{
			if(!(strcasecmp("ON",function)))
			{
				PA_4=0;
				PA_5=1;
			}
			else
			{
				PA_5=0;
			}
		}

		if(!(strcasecmp("MEAS_C",temp_other)))
		{
			if(!(strcasecmp("ON",function)))
			{
				PA_5=0;
				PA_4=1;
			}
			else
			{
				PA_4=0;
			}
		}
		if(!(strcasecmp("LOWSIDE_R",temp_other)))
		{
			if(!(strcasecmp("ON",function)))
			{
				PA_3=1;
			}
			else
			{
				PA_3=0;
			}
		}
		if(!(strcasecmp("HIGHSIDE_R",temp_other)))
		{
			if(!(strcasecmp("ON",function)))
			{
				PE_4=1;
			}
			else
			{
				PE_4=0;
			}
		}
		if(!(strcasecmp("Integrate",temp_other)))
		{
			if(!(strcasecmp("ON",function)))
			{
				PA_2=1;
			}
			else
			{
				PA_2=0;
			}
		}
	}
	else if(iscommand("test",0))
	{
		PA_2=1;
		PA_3=1; //low
		PE_4=0; //high
		waitMicrosecond(3000000);
		PA_3=0;
		WTIMER5_TAV_R = 0;
		PE_4=1;
		WTIMER5_CTL_R |= TIMER_CTL_TAEN;
	}
	else if(iscommand("resistance",0))
	{
		PA_2=1;	//int
		PA_3=1; //low
		PE_4=0; //high
		waitMicrosecond(1000000);
		WTIMER5_TAV_R = 0;
		PA_3=0; //low
		PA_5=1;	//measLR
		WTIMER5_CTL_R |= TIMER_CTL_TAEN;
		waitMicrosecond(2000000);
		res=time/1.299;
		sprintf(str5,"%0.2f",res);
	    putsUart0("\r\nResistance: ");
	    putsUart0(str5);
	    putsUart0("  Ohm\r\n");

	}
	else if(iscommand("capacitance",0))
	{
		uint32_t d=100000;
		PA_2=0;	//int
		PA_4=1;	//measC
		PA_3=1;	//low
		PE_4=0; //high
		waitMicrosecond(2000000);
		WTIMER5_TAV_R = 0;
		PA_3=0;	//low
		PE_4=1; //high
		WTIMER5_CTL_R |= TIMER_CTL_TAEN;
		waitMicrosecond(3000000);
		cap=(time/(d*1.29));
		sprintf(str6,"%0.2f",cap);
		putsUart0("\r\nCapacitance: ");
		putsUart0(str6);
		putsUart0("  microF\r\n");

	}
	else if(iscommand("Inductance",0))
	{
		inductance();
	}
	else if(iscommand("Auto",0))
	{
		Auto();
	}

	else
	{
			putsUart0("\r\nEnter the correct string\r\n");
	}


}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();

	// Display greeting
    putsUart0("\r\nEnter the character\r\n");
    GREEN_LED ^= 1;
    waitMicrosecond(500000);

    while(1)
    {

    step2(stringg);
    step3(stringg);
    step4();

    }
}
