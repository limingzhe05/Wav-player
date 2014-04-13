#include <msp430.h> 
#include <stdint.h>
#include "pff2a/src/diskio.h"
#include "pff2a/src/pff.h"

int spi_selected() {
	return (P1OUT & BIT4) != 0;
}

void dly100u() {
	__delay_cycles(95);
}

void spi_select() {
	P1OUT &= ~BIT4;
}

void spi_deselect() {
	P1OUT |= BIT4;
}

BYTE spi_send(uint8_t v) {
	UCB0TXBUF = v; // dummy
	while(UCB0STAT & UCBUSY);
	return UCB0RXBUF;
}

uint8_t spi_receive() {
	UCB0TXBUF = 0xFF; // dummy
	while(UCB0STAT & UCBUSY);
	return UCB0RXBUF;
}

void spi_init() {
	P1DIR |= BIT4; // outputs
	P1SEL = BIT5 | BIT6 | BIT7; // primary peripheral function
	P1SEL2 = BIT5 | BIT6 | BIT7; // primary peripheral function

	// P1REN = BIT6; // pull enabled
	// P1OUT |= BIT6;

	P1OUT = BIT4; // pull up, active low

	/* Disable USCI */
    UCB0CTL1 |= UCSWRST;

    UCB0CTL0 =
    		UCCKPH | // idle low, capture on first high
			BIT5 | // MSB first
			//BIT4 | // 8 bit
			BIT3 | // Master
			// BIT2 |
			// BIT1 | // 3 pin SPI, active low CS
			BIT0 // UCSYNC
			;

	UCB0CTL1 |= BIT7 |
			// BIT6 | // SMCLK
			0;

	IE2 &= ~UCB0RXIE;
	IE2 &= ~UCB0TXIE;

    /* Bit Rate Control Register 0 */
    UCB0BR0 = 10;

    /* Bit Rate Control Register 1 */
    UCB0BR1 = 0;

    /* Enable USCI */
    UCB0CTL1 &= ~UCSWRST;
#if 0
    spi_deselect();

	// at least 74 cycles
	char i;
	for(i = 0; i < 10; i++) {
		UCB0TXBUF = 0xFF; // MOSI high
		while(UCB0STAT & UCBUSY);
	}

	__delay_cycles(20000);
#endif
}

void spi_ping() {
	volatile uint8_t r1;

	// deselected!
	spi_receive();

	spi_select();

	spi_receive();

	spi_send(0);
	spi_send(0);
	spi_send(0);
	spi_send(0);
	spi_send(0);
	spi_send(0x95);

	int i;
	for (i = 0; i < 10; i++) {
		r1 = spi_receive();
		if (r1 & 0x80)
			break;
	}
	spi_deselect();
}

FATFS fatfs;
uint8_t buf[10];

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    spi_init();

    attach_cs_pin (spi_select, spi_deselect, spi_selected);
    attach_dly100u (dly100u);
    attach_SPIdriver(spi_receive, spi_send);

    // for tuning delay func
#if 0
    while(1) {
    	spi_select();
    	dly100u();
    	spi_deselect();
    	dly100u();
    }
#endif

    disk_initialize();

    volatile FRESULT res;

    if (res = pf_mount(&fatfs)) {
    	__no_operation();
    }

    if (res = pf_open("1.wav")) {
    	__no_operation();
    }

    WORD num_read;

    if (res = pf_read(buf, 10, &num_read)) {
    	__no_operation();
    }

	return 0;
}
