
#include  <stdint.h>
#include  <msp430x20x3.h>

#define FALSE 0
#define TRUE (!FALSE)

void usi_i2c_init(void);
void send_to_host(int data);

unsigned int timer_count;
uint8_t host_data = 0;

unsigned int measure_key_capacitance(void)
{
    int sum;
                                /* Right now, all keys should be driven low, forming a "ground" area */
    P1OUT |= BIT1;              /* Take the active key high to charge the pad */
    _NOP(); _NOP(); _NOP();     /* Allow a short delay for the hard pull high to really charge the pad */
    P1IES |= BIT1;              /* Configure a port interrupt as the pin discharges */
    P1IE |= BIT1;
    P1DIR &= ~BIT1;
    timer_count = TAR;          /* Take a snapshot of the timer... */
    LPM0;                       /* ...and wait for the discharge to cause an interrupt.  */
    P1IE &= ~BIT1;              /* Disable active key interrupts */
    P1OUT &= ~BIT1;             /* Discharge the key */
    P1DIR |= BIT1;              /* switch active key to output low to save power */
    sum = timer_count;
    P1OUT |= BIT0;
    _NOP(); _NOP(); _NOP();
    P1IES &= ~BIT1;             /* Configure a port interrupt as the pin charges */
    P1IE |= BIT1;
    P1DIR &= ~BIT1;
    timer_count = TAR;          /* Take a snapshot of the timer... */
    LPM0;                       /* ...and wait for the discharge to cause an interrupt. */
    P1IE &= ~BIT1;              /* Disable active key interrupt */
    P1OUT &= ~(BIT1 | BIT0);    /* Return the keys to the "ground" state */
    P1DIR |= BIT1;
    sum += timer_count;
    return sum;                 /* The sum od the two readings is our answer */
}

int base_capacitance = 0;
long int filtered = 0;
int measured;
int margin;
int Charge;

long int scan_key(void)
{
    measured = measure_key_capacitance();
    margin = measured - base_capacitance;
    filtered += (margin - (filtered >> 4));
    return filtered;
}

void main(void)
{
    int i;

    WDTCTL = WDTPW | WDTHOLD;             /* Stop the watchdog */

    /* Use the VLOCLK oscillator. This is important. If we leave the unused 32kHz
       oscillator running, its pins will not be free for sensing. */
    BCSCTL3 = LFXT1S_2;
    BCSCTL1 = CALBC1_16MHZ;               /* Set the DCO speed */
    DCOCTL = CALDCO_16MHZ;

    /* Initialise all the I/O pins */
    P1OUT = 0;
    P1DIR = 0xFF;
    P1SEL = 0;
    P1REN = 0;
    P1OUT = 0;
    P1DIR = 0xFF;
    P1SEL = 0;
    P1REN = 0;

   /* Prepare for I2C communication */
    usi_i2c_init();

    _EINT();

    base_capacitance = 0;
    filtered = 0;

    /* Set the capacitive sense pin to output, low level, low going interupts */
    /* We need to keep all switches, except the one we are currently sensing,
       driven low so they form the "ground" against which the active pad and the
       finger form the variable capacitance. The end pads in the row can be
       expected to give a weaker response, because they are not surrounded by
       grounded keys on both sides, as the other keys are. */
    P1OUT &= ~BIT1;
    P1DIR |= BIT1;
    P1IES |= BIT1;
    /* Drive Timer A from the SMCLK, in continuous mode */
    TACTL = TASSEL_2 | MC_2 | ID_3;

    /* Scan the keys quite a few times, to allow plenty of time for the
       MCLK and board conditions to stablise */
    for (i = 0;  i < 1000;  i++)
        scan_key();

    /* Now we can use the current filtered key response as the base response.
       The shift allows for the filter gain. */
    base_capacitance = filtered >> 4;
    filtered = 0;

    while (TRUE)
    {
        scan_key();
        Charge = filtered >> 4;
        if (Charge < 0) Charge = 0;
        else if (Charge > 255) Charge = 255;
        if(Charge>200) P1OUT|=BIT0; else P1OUT&=~BIT0;
        send_to_host(Charge);
        _NOP();
        for (i = 0;  i < 1000;  i++) _NOP(); // pause
    }
}

/* ----------------------------- KEY HANDLING INTERRUPTS ---------------------------- */

#pragma vector=PORT1_VECTOR
__interrupt void port_2_interrupt(void)
{
    P1IFG = 0;                                /* Clear interrupt flag */
    timer_count = TAR - timer_count;          /* Record the discharge time */
    LPM3_EXIT;                                /* Exit from low power 3 or 0 */
}

#pragma vector=TIMERA0_VECTOR
__interrupt void timera0_interrupt(void)
{
    LPM3_EXIT;
}

#pragma vector=TIMERA1_VECTOR
__interrupt void timera1_interrupt(void)
{
    switch (TAIV)
    {
    case 2:
        /* TACCR1 */
        LPM3_EXIT;
        break;
    }
}

/* ------------------------------- COMMUNICATIONS ---------------------- */

const uint8_t SLV_Addr = 0x48;                      /* I2C slave address is 0x48 */
int8_t I2C_state = -2;                               /* I2C state tracking */

void send_to_host(int data)
{
    host_data = data;
    I2C_state = 0;
    USICTL1 |= USIIFG;                              /* Set flag and start communication */
    /* Wait until the I/O operation has completed */
    do
        LPM0;
    while (I2C_state >= 0);
}

void usi_i2c_init(void)
{
    P1OUT |= (BIT1 | BIT0);                         /* P1.6 & P1.7 are for I2C */
    P1REN |= (BIT1 | BIT0);                         /* Enable P1.6 & P1.7 pullups */

    USICTL0 = USIPE6 | USIPE7 | USIMST | USISWRST;  /* Port & USI mode setup */
    USICTL1 = USII2C | USIIE;                       /* Enable I2C mode & USI interrupt */
    USICKCTL = USIDIV_7 | USISSEL_2 | USICKPL;      /* Setup USI clock: SCL = SMCLK/128 (~125kHz) */
    USICNT |= USIIFGCC;                             /* Disable automatic clear control */
    USICTL0 &= ~USISWRST;                           /* Enable USI */
    USICTL1 &= ~USIIFG;                             /* Clear pending flag */
}

#pragma vector = USI_VECTOR
__interrupt void USI_TXRX(void)
{
    switch (__even_in_range(I2C_state, 10))
    {
    case 0:
         /* Generate start condition & send address to slave */
        USISRL = 0x00;                      /* Generate Start Condition... */
        USICTL0 |= (USIGE | USIOE);
        USICTL0 &= ~USIGE;
        USISRL = SLV_Addr << 1;             /* ... and transmit address, R/W = 0 */
        USICNT = (USICNT & 0xE0) + 0x08;    /* Bit counter = 8, TX Address */
        I2C_state = 2;                      /* Go to next state: receive address (N)Ack */
        break;
    case 2:
        /* Receive sddress Ack/Nack bit */
        USICTL0 &= ~USIOE;                  /* SDA = input */
        USICNT |= 0x01;                     /* Bit counter = 1, receive (N)Ack bit */
        I2C_state = 4;                      /* Go to next state: check (N)Ack */
        break;
    case 4:
        /* Process sddress Ack/Nack & handle data TX */
        USICTL0 |= USIOE;                   /* SDA = output */
        if (USISRL & 0x01)
        {
            /* Nack received. Send stop... */
            USISRL = 0x00;
            USICNT |=  0x01;                /* Bit counter = 1, SCL high, SDA low */
            I2C_state = 10;                 /* Go to next state: generate Stop */
        }
        else
        {
            /* Ack received, TX data to slave... */
            USISRL = host_data;             /* Load data byte */
            USICNT |=  0x08;                /* Bit counter = 8, start TX */
            I2C_state = 6;                  /* Go to next state: receive data (N)Ack */
        }
        break;
    case 6:
        /* Receive Data Ack/Nack bit */
        USICTL0 &= ~USIOE;                  /* SDA = input */
        USICNT |= 0x01;                     /* Bit counter = 1, receive (N)Ack bit */
        I2C_state = 8;                      /* Go to next state: check (N)Ack */
        break;
    case 8:
        /* Process Data Ack/Nack & send Stop */
        USICTL0 |= USIOE;
        if (USISRL & 0x01)
        {
            /* Nack received... */
            /* TODO: Handle the NACK */
        }
        else
        {
            /* Ack received */
        }
        /* Send stop... */
        USISRL = 0x00;
        USICNT |=  0x01;                    /* Bit counter = 1, SCL high, SDA low */
        I2C_state = 10;                     /* Go to next state: generate stop */
        break;
    case 10:
        /* Generate Stop Condition */
        USISRL = 0x0FF;                     /* USISRL = 1 to release SDA */
        USICTL0 |= USIGE;                   /* Transparent latch enabled */
        USICTL0 &= ~(USIGE | USIOE);        /* Latch/SDA output disabled */
        I2C_state = -2;                     /* Reset state machine for next transmission */
        LPM0_EXIT;                          /* Exit active for next transfer */
        break;
    }
    USICTL1 &= ~USIIFG;                     /* Clear pending flag */
}

