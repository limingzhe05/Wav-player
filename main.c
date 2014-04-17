/**
 *  _____ ___  ___   ___   
 * |_   _/ _ \|   \ / _ \
 *   | || (_) | |) | (_) | 
 *   |_| \___/|___/ \___/
 *
 * TODO:
 * Verify SD bandwidth
 * gå ikke over sporet...
 * pwm driver
 * scheduler
 * universal audio converter
 *
 * https://ccrma.stanford.edu/courses/422/projects/WaveFormat/
 */

#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>
#include "pff2a/src/diskio.h"
#include "pff2a/src/pff.h"

#include "wav.h"

#define SWAP_UINT16(x) _swap_bytes(x)
//#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
#define SWAP_UINT32(x) (_swap_bytes(x & 0xFFFF) << 16) || (_swap_bytes(x >> 16))

wav_fmt fmt;

uint8_t bytes_per_sample;

bool wav_active;

FATFS fatfs;

volatile FRESULT res;
WORD num_read;

// double buffer
uint8_t wav_buf[128];
uint8_t *wav_buf_idx;
uint8_t wav_buf_count;

uint8_t pwm_queue[128];
uint8_t pwm_queue_first, pwm_queue_count;

typedef enum {
  E_READ,
  E_OPEN,
  E_HEADER,
  E_PARSE,
  E_MOUNT
} E_ERRORS;

void led_on() {
  P1OUT |= BIT0;
}

void led_off() {
  P1OUT &= ~BIT0;
}

void led_init(void) {
  P1DIR |= BIT0;
  led_off();
}

void dly100u() {
  __delay_cycles(1529);
}

void panic(E_ERRORS err) {
  uint16_t c, i;

  _DINT();
  led_init();

  while(1) {
    for(c = 0; c < err; c++) {
      // .3s
      for (i = 0; i < 3000; i++)
        dly100u();

      led_on();

      // .7s
      for (i = 0; i < 7000; i++)
        dly100u();

      led_off();
    }

    // 2s
    for (i = 0; i < 20000; i++)
      dly100u();
  }
}

int spi_selected() {
  return (P1OUT & BIT4) != 0;
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
#define SPI_BPS 4e6

  UCB0BR0 = 16e6/SPI_BPS;

  /* Bit Rate Control Register 1 */
  UCB0BR1 = 0;

  /* Enable USCI */
  UCB0CTL1 &= ~UCSWRST;
}

bool pwm_queue_full() {
  return pwm_queue_count == sizeof(pwm_queue);
}

bool pwm_queue_empty() {
  return pwm_queue_count == 0;
}

void pwm_queue_enqueue(uint8_t v) {
  pwm_queue[pwm_queue_first] = v;

  pwm_queue_count++;
  pwm_queue_first++;
  if (pwm_queue_first == sizeof(pwm_queue)) {
    pwm_queue_first = 0;
  }
}

uint8_t pwm_queue_dequeue() {
  int16_t loc = pwm_queue_first - pwm_queue_count;
  if (loc < 0) {
    loc += sizeof(pwm_queue);
  }
  pwm_queue_count--;
  return pwm_queue[loc];
}

void pwm_queue_init() {
  pwm_queue_count = 0;
  pwm_queue_first = 0;
}

// DEBUG
uint32_t bytes_played;

void pwm_start() {
  P2DIR |= BIT1;
  P2SEL |= BIT1;
  P2SEL2 &= ~BIT1;

  TA1CTL = TASSEL_2 | MC_1;
  TA1CCTL0 = 0;
  TA1CCTL1 = OUTMOD_3;
  TA1CCR0 = 255;
  TA1CCR1 = 256;

  bytes_played = 0;
}

void pwm_irq() {
  if(pwm_queue_empty()) {
    // disable irq - we are finished.
    TACCTL0 &= ~CCIE;
    wav_active = false;
  } else {
    static uint8_t a = 0;
#if 1
    a = pwm_queue_dequeue();
#else
    a++;
    a &= 0xFF;
#endif
    // play a
    TA1CCR1 = a;

    bytes_played++;
  }
}

void fill_pwm_queue() {
  uint8_t i;
  int32_t a = 0;

  _DINT();

  // Fill wav buffer
  if (wav_buf_idx >= wav_buf + wav_buf_count) {
	  led_on();
    // load from SD. Enable interrupt during this operation.
    _EINT();

#if 1
    if (res = pf_read(wav_buf, sizeof(wav_buf), &num_read)) {
      panic(E_READ);
    }
#else
    num_read = sizeof(wav_buf);
#endif

    wav_buf_count = num_read;

    wav_buf_idx = wav_buf;

    led_off();
    _DINT();
  }

  // Decode
  for (i = 0; i < fmt.NumChannels; i++) {
    if (fmt.BitsPerSample == 8) {
      a += (int16_t) (*wav_buf_idx)-128;
      wav_buf_idx++;
    } else {
      a += *((uint16_t *) wav_buf_idx);
      wav_buf_idx += 2;
    }
  }
  a /= (fmt.NumChannels * (fmt.BitsPerSample/8));
  pwm_queue_enqueue(a);

  _EINT();
}
/*
 * main.c
 */

void init_sample_timer(uint16_t freq) {
  __disable_interrupt();

  TACCTL0 = CCIE;
  TACTL = TASSEL_2 + MC_1; // Set the timer A to SMCLCK, Continuous
  // Clear the timer and enable timer interrupt

  // 44.1kHz
#if 1
  freq = 16000;
#endif

  TACCR0 = 16E6 / freq;

  __enable_interrupt();
}

void init_play(char *fname) {
  // play
  wav_active = true;

  if (res = pf_open(fname)) {
    panic(E_OPEN);
  }

  // read header
  if (res = pf_read(wav_buf, 44, &num_read)) {
    panic(E_HEADER);
  }

  if (!wav_parse(wav_buf, &fmt)) {
    panic(E_PARSE);
  }

  bytes_per_sample = (fmt.BitsPerSample / 8) * fmt.NumChannels;
  pwm_queue_init();
  wav_buf_count = 0;
  wav_buf_idx = wav_buf;

  while(!pwm_queue_full()) {
    fill_pwm_queue();
  }

  pwm_start();

  init_sample_timer(fmt.SampleRate);
}

bool play_loop(void) {
  if(wav_active) {
    // fill as much as possible to pcm_queue
    while (wav_buf_count > 0 &&
           !pwm_queue_full()) {
      fill_pwm_queue();
    }
    return true;
  } else {
    return false;
  }
}

uint32_t i;

int main(void) {
  WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

  BCSCTL2 = SELM_0 | DIVM_0 | DIVS_0;
  if (CALBC1_16MHZ != 0xFF) {
    /* Adjust this accordingly to your VCC rise time */
    __delay_cycles(100000);

    /* Follow recommended flow. First, clear all DCOx and MODx bits. Then
     * apply new RSELx values. Finally, apply new DCOx and MODx bit values.
     */
    DCOCTL = 0x00;
    BCSCTL1 = CALBC1_16MHZ;     /* Set DCO to 16MHz */
    DCOCTL = CALDCO_16MHZ;
  }
  BCSCTL1 |= XT2OFF | DIVA_0;
  BCSCTL3 = XT2S_0 | LFXT1S_0 | XCAP_1;
	
  led_init();

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

  if (res = pf_mount(&fatfs)) {
    panic(E_MOUNT);
  }

#if 0
  // test that sample sound can be loaded within time.
  led_on();

  if (res = pf_open("1.wav")) {
    __no_operation();
  }

  if (res = pf_read(wav_buf, 10, &num_read)) {
    __no_operation();
  }

  for(i = 0; i < 262460; i += 128) {
    if (res = pf_read(wav_buf, 128, &num_read)) {
      __no_operation();
    }
    if (num_read < 128) {
      __no_operation();
      break;
    }
  }

  led_off();
#endif

  while(true) {
	  P1DIR &= ~BIT3;

	  init_play("1.wav");

	  while(play_loop()) {
		;
		if (P1IN & BIT3 == 0) {
			// PWM Will automatically stop
			while(P1IN & BIT3 == 0) {
				// Wait for key release
				__no_operation();
			}
			break;
		}
	  }

	  while(P1IN & BIT3 == 1) {
		  // Wait for keypress
		  __no_operation();
	  }
		while(P1IN & BIT3 == 0) {
			// Wait for key release
			__no_operation();
		}
  }

  return 0;
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_B (void)
{
  //P1OUT ^= (BIT0);
  pwm_irq();
}
