/*
    A program for the MSP430FG4618 on the Experimenter's board to accept I2C information from the touchpad,
    display it numerically on the LCD, and send it to a host PC for graphical display.


    Steve Underwood
*/

#include <stdint.h>
#include <msp430xG46x.h>

#define FALSE 0
#define TRUE (!FALSE)

//First blank, and the hex codes, which all display quite
//well on a 7-segment display.
#define CHAR_SPACE          0
#define CHAR_ALL            (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g|SEG_h)
#define CHAR_0              (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_1              (SEG_b|SEG_c)
#define CHAR_2              (SEG_a|SEG_b|SEG_d|SEG_e|SEG_g)
#define CHAR_3              (SEG_a|SEG_b|SEG_c|SEG_d|SEG_g)
#define CHAR_4              (SEG_b|SEG_c|SEG_f|SEG_g)
#define CHAR_5              (SEG_a|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_6              (SEG_a|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7              (SEG_a|SEG_b|SEG_c)
#define CHAR_8              (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_9              (SEG_a|SEG_b|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_A              (SEG_a|SEG_b|SEG_c|SEG_e|SEG_f|SEG_g)
#define CHAR_B              (SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_C              (SEG_a|SEG_d|SEG_e|SEG_f)
#define CHAR_D              (SEG_b|SEG_c|SEG_d|SEG_e|SEG_g)
#define CHAR_E              (SEG_a|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_F              (SEG_a|SEG_e|SEG_f|SEG_g)
#define CHAR_MINUS          (SEG_g)

#define SEG_a       0x01
#define SEG_b       0x02
#define SEG_c       0x04
#define SEG_d       0x08
#define SEG_e       0x40
#define SEG_f       0x10
#define SEG_g       0x20
#define SEG_h       0x80

const uint8_t lcd_digit_table[18] =
{
    CHAR_0,
    CHAR_1,
    CHAR_2,
    CHAR_3,
    CHAR_4,
    CHAR_5,
    CHAR_6,
    CHAR_7,
    CHAR_8,
    CHAR_9,
    CHAR_A,
    CHAR_B,
    CHAR_C,
    CHAR_D,
    CHAR_E,
    CHAR_F,
    CHAR_MINUS,
    CHAR_SPACE
};

void init_lcd(void)
{
    int i;

    /* Basic timer setup */
    /* Set ticker to 32768/(256*256) */
    BTCTL = BT_fCLK2_DIV128 | BT_fCLK2_ACLK_DIV256;

    for (i = 0;  i < 20;  i++)
        LCDMEM[i] = 0;

    /* Turn on the COM0-COM3 and R03-R33 pins */
    P5SEL |= (BIT4 | BIT3 | BIT2);
    P5DIR |= (BIT4 | BIT3 | BIT2);

    /* LCD-A controller setup */
    LCDACTL = LCDFREQ_128 | LCD4MUX | LCDSON | LCDON;
    LCDAPCTL0 = LCDS0 | LCDS4 | LCDS8 | LCDS12 | LCDS16 | LCDS20 | LCDS24;
    LCDAPCTL1 = 0;
    LCDAVCTL0 = LCDCPEN;
    LCDAVCTL1 = 1 << 1;
}

void LCDchar(int ch, int pos)
{
    /* Put a segment pattern at a specified position on the LCD display */
    LCDMEM[9 - pos] = ch;
}

void LCDdigit(uint16_t val, int pos)
{
    LCDchar(lcd_digit_table[val], pos);
}

void LCDdec(uint16_t val, int pos)
{
    int i;

    for (i = 2;  i >= 0;  i--)
    {
        LCDchar(lcd_digit_table[val%10], pos + i);
        val /= 10;
    }
}

void configure_i2c_usci0(void)
{
    /* Configure USCI0B as an I2C port */
    P3SEL |= (BIT2 | BIT1);
    UCB0CTL1 |= UCSWRST;                    /* Enable SW reset */

    UCB0CTL0 = UCMODE_3 | UCSYNC;           /* I2C Slave, synchronous mode */
    UCB0I2COA = 0x48;                       /* Own Address */
    UCB0CTL1 &= ~UCSWRST;                   /* Clear SW reset, resume operation */
    UCB0I2CIE |= (UCSTPIE | UCSTTIE);       /* Enable STT and STP interrupt */
    IE2 |= UCB0RXIE;                        /* Enable RX interrupt */
}

uint8_t xxx = 0;

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               /* Stop watchdog */

    configure_i2c_usci0();
    init_lcd();

    _EINT();
    for (;;)
    {
        /* Normal operation */
        LPM0;
        LCDdec(xxx, 3);
    }
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    xxx =
    UCA0TXBUF = UCB0RXBUF;
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
    UCB0STAT &= ~(UCSTPIFG | UCSTTIFG);
    LPM0_EXIT;
}
