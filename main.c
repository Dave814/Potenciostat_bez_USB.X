/*
 * USB Potentiostat/galvanostat firmware
 *
 * This code makes use of Signal 11's M-Stack USB stack to implement 
 * communication through raw USB bulk transfers. Commands are received
 * as ASCII strings on EP1 OUT. They are then interpreted and executed;
 * they either change the state of output pins, or cause data to be read
 * from / written to the MCP3550 (ADC) or DAC1220 (DAC) using a software
 * SPI implementation. The resulting data, or an "OK" message, is sent as
 * a reply on EP1 IN. The USB service is interrupt-driven.
 *
 * Thomas Dobbelaere
 * CoCooN research group
 * 2016-06-07
 */

//#include "usb\include\usb.h"
#include <xc.h>
#include <string.h>
//#include "usb_config.h"
//#include "usb\include\usb_ch9.h"
#include "spi\spi_software.h"
#include "heflash\Flash.h"
#include "heflash\HEFlash.h"
#include "eusart.h"

// PIC16F1459 configuration bit settings:
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF     // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection Bit (NO CPU system divide)
#pragma config USBLSCLK = 48MHz // USB Low Speed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define _XTAL_FREQ 48000000     // 48 MHz CPU clock frequency

#define MODE_SW_PIN LATAbits.LATA5
#define MODE_SW_TRIS TRISAbits.TRISA5
#define CELL_ON_PIN LATAbits.LATA4
#define CELL_ON_TRIS TRISAbits.TRISA4
#define CELL_ON  1
#define CELL_OFF 0
#define POTENTIOSTATIC  0
#define GALVANOSTATIC  1
#define INPUT  1
#define OUTPUT 0
#define RANGE1_PIN LATCbits.LATC4
#define RANGE1_TRIS TRISCbits.TRISC4
#define RANGE2_PIN LATCbits.LATC5
#define RANGE2_TRIS TRISCbits.TRISC5
#define RANGE3_PIN LATCbits.LATC6
#define RANGE3_TRIS TRISCbits.TRISC6

//static const uint8_t* received_data;
static uint8_t received_data_length;
//static uint8_t* transmit_data;
static uint8_t transmit_data_length;
static uint8_t heflashbuffer[FLASH_ROWSIZE];
static uint8_t uart_command[20];

void InitializeIO()
{
	OSCCONbits.IRCF = 0b1111; // 0b1111 = 16MHz HFINTOSC postscaler
	ANSELA = 0x00; // digital I/O on PORTA
	ANSELB = 0x00; // digital I/O on PORTB
	ANSELC = 0x00; // digital I/O on PORTC
	APFCON = 0x00; // no funny alternate pins
	MODE_SW_TRIS = OUTPUT;
	MODE_SW_PIN = POTENTIOSTATIC; // initialize mode to potentiostatic
	CELL_ON_TRIS = OUTPUT;
	CELL_ON_PIN = CELL_OFF; // initialize cell to off position
	RANGE1_TRIS = OUTPUT;
	RANGE2_TRIS = OUTPUT;
	RANGE3_TRIS = OUTPUT;
	RANGE1_PIN = 1; // initialize range to range 1
	RANGE2_PIN = 0;
	RANGE3_PIN = 0;
	InitializeSPI();
	__delay_ms(25); // power-up delay - necessary for DAC1220
	DAC1220_Reset();
	__delay_ms(25);
	DAC1220_Init();
	HEFLASH_readBlock(heflashbuffer, 2, FLASH_ROWSIZE); // get dac calibration
	DAC1220_Write3Bytes(8, heflashbuffer[0], heflashbuffer[1], heflashbuffer[2]); // apply dac calibration
	DAC1220_Write3Bytes(12, heflashbuffer[3], heflashbuffer[4], heflashbuffer[5]); 
    
    EUSART_Initialize();
    const uint8_t *startReply = "Ready!\n";
    USART_Send(startReply,strlen(startReply));
}


void command_unknown()
{
    const uint8_t *reply = "?\n";
    //strcpy(transmit_data, reply);
    transmit_data_length = strlen(reply);
    USART_Send(reply,transmit_data_length);
}

void send_OK()
{
	const uint8_t* reply = "OK\n";
    transmit_data_length = strlen(reply);
    USART_Send(reply,transmit_data_length);
}

void send_WAIT()
{
    const uint8_t* reply = "WAIT\n";
    transmit_data_length = strlen(reply);
    USART_Send(reply,transmit_data_length);
}

void command_cell_on()
{
	CELL_ON_PIN = CELL_ON;
	send_OK();
}

void command_cell_off()
{
	CELL_ON_PIN = CELL_OFF;
	send_OK();
}

void command_mode_potentiostatic()
{
	MODE_SW_PIN = POTENTIOSTATIC;
	send_OK();
}

void command_mode_galvanostatic()
{
	MODE_SW_PIN = GALVANOSTATIC;
	send_OK();
}

void command_range1()
{
    RANGE1_PIN = 1;
    __delay_ms(10); // make the new relay setting before breaking the old one
    RANGE2_PIN = 0;
    RANGE3_PIN = 0;
	send_OK();
}

void command_range2()
{
    RANGE2_PIN = 1;
    __delay_ms(10); // make the new relay setting before breaking the old one
    RANGE1_PIN = 0;
    RANGE3_PIN = 0;
	send_OK();
}

void command_range3()
{
    RANGE3_PIN = 1;
    __delay_ms(10); // make the new relay setting before breaking the old one
    RANGE1_PIN = 0;
    RANGE2_PIN = 0;
	send_OK();
}

void command_set_dac(const uint8_t* dac_data)
{
	DAC1220_Write3Bytes(0, dac_data[0], dac_data[1], dac_data[2]);
	send_OK();
}

void command_calibrate_dac()
{
	DAC1220_SelfCal();
	__delay_ms(500); // wait until calibration is finished
	uint8_t data[6];
	DAC1220_Read3Bytes(8, data, data+1, data+2); // get calibration data
	DAC1220_Read3Bytes(12, data+3, data+4, data+5);
	HEFLASH_writeBlock(2, data, 6); // save calibration data to HEFLASH
	send_OK();
}

void command_read_adc() // sends 2x3bytes to UART <MSB1,middle1,LSB1;MSB2,middle2,LSB2>
                        // bytes 0-2 respresents voltage ADC
                        // bytes 3-5 represents current ADC
{
	uint8_t adc_data[6];
	if(MCP3550_Read(adc_data))
	{      
        USART_Send_6bytes(adc_data);
	}
	else
	{
        send_WAIT();
	}
}

void command_read_offset()
{
	HEFLASH_readBlock(heflashbuffer, 1, FLASH_ROWSIZE);
	USART_Send_6bytes(heflashbuffer);
}

void command_save_offset(const uint8_t* offset_data)
{
	HEFLASH_writeBlock(1, offset_data, 6);
	send_OK();
}

void command_read_shuntcalibration()
{
	HEFLASH_readBlock(heflashbuffer, 3, FLASH_ROWSIZE);
    USART_Send_6bytes(heflashbuffer);
}

void command_save_shuntcalibration(const uint8_t* shuntcalibration_data)
{
	HEFLASH_writeBlock(3, shuntcalibration_data, 6);
	send_OK();
}

void command_read_dac_cal()
{
	HEFLASH_readBlock(heflashbuffer, 2, FLASH_ROWSIZE);
	USART_Send_6bytes(heflashbuffer);
}

void command_set_dac_cal(const uint8_t* dac_cal_data)
{
	HEFLASH_writeBlock(2, dac_cal_data, 6);
	DAC1220_Write3Bytes(8, dac_cal_data[0], dac_cal_data[1], dac_cal_data[2]);
	DAC1220_Write3Bytes(12, dac_cal_data[3], dac_cal_data[4], dac_cal_data[5]);
	send_OK();
}

void interpret_command() {
    if (received_data_length == 7 && strncmp(uart_command,"CELL ON",7) == 0)
        command_cell_on();
    else if (received_data_length == 8 && strncmp(uart_command,"CELL OFF",8) == 0)
        command_cell_off();
    else if (received_data_length == 14 && strncmp(uart_command,"POTENTIOSTATIC",14) == 0)
        command_mode_potentiostatic();
    else if (received_data_length == 13 && strncmp(uart_command,"GALVANOSTATIC",13) == 0)
        command_mode_galvanostatic();
    else if (received_data_length == 7 && strncmp(uart_command,"RANGE 1",7) == 0)
        command_range1();
    else if (received_data_length == 7 && strncmp(uart_command,"RANGE 2",7) == 0)
        command_range2();
    else if (received_data_length == 7 && strncmp(uart_command,"RANGE 3",7) == 0)
        command_range3();
    else if (received_data_length == 10 && strncmp(uart_command,"DACSET ",7) == 0)
	command_set_dac(uart_command+7);
    else if (received_data_length == 6 && strncmp(uart_command,"DACCAL",6) == 0)
	command_calibrate_dac();
    else if (received_data_length == 7 && strncmp(uart_command,"ADCREAD",7) == 0)
	command_read_adc();
    else if (received_data_length == 10 && strncmp(uart_command,"OFFSETREAD",10) == 0)
	command_read_offset();
    else if (received_data_length == 17 && strncmp(uart_command,"OFFSETSAVE ",11) == 0)
	command_save_offset(uart_command+11);
    else if (received_data_length == 9 && strncmp(uart_command,"DACCALGET",9) == 0)
	command_read_dac_cal();
    else if (received_data_length == 16 && strncmp(uart_command,"DACCALSET ",10) == 0)
	command_set_dac_cal(uart_command+10);
    else if (received_data_length == 12 && strncmp(uart_command,"SHUNTCALREAD",12) == 0)
	command_read_shuntcalibration();
    else if (received_data_length == 19 && strncmp(uart_command,"SHUNTCALSAVE ",13) == 0)
	command_save_shuntcalibration(uart_command+13);
    
    else
        command_unknown();
}

int main(void)
{
	InitializeIO();
    

	// Enable Active clock-tuning from the USB
	ACTCONbits.ACTSRC = 1; // 1=USB
	ACTCONbits.ACTEN = 1;

	// Configure interrupts
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;

	// Initialize USB
	//usb_init();

	while (1)
	{
		/*if (usb_is_configured() && usb_out_endpoint_has_data(1)) // wait for data received from host
		{
			if (!usb_in_endpoint_halted(1))
			{
				while (usb_in_endpoint_busy(1)) // wait for EP1 IN to become free
					;
				received_data_length = usb_get_out_buffer(1, &received_data); // get memory location and length of received data
				transmit_data = usb_get_in_buffer(1); // get memory location of data to transmit
				interpret_command(); // this reads received_data and sets transmit_data and transmit_data_length
				usb_send_in_buffer(1, transmit_data_length); // send the data back
			}
			usb_arm_out_endpoint(1);
		}*/
        
        if(EUSART_is_rx_ready()){
            uint8_t i=0;
            uint8_t c;
            c=EUSART_Read();
            while((c!='\r') && (c!='\n')){
                uart_command[i]=c;
                i++;
                c=EUSART_Read();
            }
            received_data_length=i;
            interpret_command();
        }
   
	}

	return 0;
}

void interrupt isr()
{
	//usb_service();
}
