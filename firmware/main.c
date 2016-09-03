#include <msp430.h> 
/*
 * main.c
 */
//------------------------------------------------------------------------------
// Hardware-related definitions
//------------------------------------------------------------------------------
#define UART_TXD   0x02                     // TXD on P1.1 (Timer0_A.OUT0)
#define UART_RXD   0x04                     // RXD on P1.2 (Timer0_A.CCI1A)

//------------------------------------------------------------------------------
// Conditions for 9600 Baud SW UART, SMCLK = 1MHz
//------------------------------------------------------------------------------
#define UART_TBIT_DIV_2     (1000000 / (9600 * 2))
#define UART_TBIT           (1000000 / 9600)

//------------------------------------------------------------------------------
// Global variables used for full-duplex UART communication
//------------------------------------------------------------------------------
unsigned int txData;    // UART internal variable for TX
unsigned char rxBuffer; // Received UART character  
unsigned int temp;
unsigned int tmin;
unsigned int tmax;
void TimerA_UART_init(void);
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);
void setPins(void);
void clock_set(void);
void itoa(long unsigned int value, char* result, int base);

void main()
{
 
    WDTCTL = WDTPW|WDTHOLD; //Stop watchdog timer
    if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
    {											
      while(1);                               // do not load, trap CPU!!	
    }
    char p[24];
    setPins();
    clock_set();
    TimerA_UART_init();
    ADC10CTL0 |= ADC10ON;//ADC setup
    ADC10CTL1 |= INCH_0|ADC10SSEL_1|CONSEQ_1;  // set channel, ad source ACLK, sequence of channels
    ADC10AE0  |= BIT0;//|BIT1;
    ADC10CTL0 |= ENC|ADC10SC; //|ADC10IE;
    __enable_interrupt();
    for (;;)
    {
        // Wait for incoming character
        __bis_SR_register(LPM0_bits);
        //temp = (temp*35)/100; 
        itoa(temp,p,10);
        TimerA_UART_print(p);
        TimerA_UART_print("\r\n");
        //set_led(temp);
    }
}

void setPins(void)
{
    P1OUT &= ~0xFF;
    P1SEL = UART_TXD + UART_RXD;            // Timer function for TXD/RXD pins
    P1DIR &= ~0xFF;
    P1DIR |= 0x42;                            // Set P1.2 and P1.6 to output direction
    P1OUT &= ~0x40;
    // default temperature values
    tmax = 25;
    tmin = 20; 
}

void clock_set(void)
{
    BCSCTL2 &= ~(DIVS_3);			// SMCLK = DCO = 1MHz
    DCOCTL = 0;                             // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                  // Set DCOCLK to 1MHz
    DCOCTL = CALDCO_1MHZ;
}

//------------------------------------------------------------------------------
// Function configures Timer_A for full-duplex UART operation
//------------------------------------------------------------------------------
void TimerA_UART_init(void)
{
    //timer 0 setup
    TA0CCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TA0CCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TA0CTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
    /*TA1CCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TA1CCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TA1CTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
    */
    //timer 1 setup
    TA1CCTL0 = CCIE;                             // CCR0 interrupt enabled
    TA1CTL = TASSEL_2 + MC_1 + ID_3;           // SMCLK/8, upmode  
    TA1CCR0 =  10000;                           // 12.5 Hz   
}
//------------------------------------------------------------------------------
// Outputs one byte using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_tx(unsigned char byte)
{
    while (TACCTL0 & CCIE);                 // Ensure last char got TX'd
    TA0CCR0 = TAR;                           // Current state of TA counter
    TA0CCR0 += UART_TBIT;                    // One bit time till first bit
    TA0CCTL0 = OUTMOD0 + CCIE;               // Set TXD on EQU0, Int
    txData = byte;                          // Load global variable
    txData |= 0x100;                        // Add mark stop bit to TXData
    txData <<= 1;                           // Add space start bit
}

//------------------------------------------------------------------------------
// Prints a string over using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_print(char *string)
{
    while (*string) {
        TimerA_UART_tx(*string++);
    }
}
void itoa(long unsigned int value, char* result, int base)
    {
      // check that the base if valid
      if (base < 2 || base > 36) { *result = '\0';}

      char* ptr = result, *ptr1 = result, tmp_char;
      int tmp_value;

      do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
      } while ( value );

      // Apply negative sign
      if (tmp_value < 0) *ptr++ = '-';
      *ptr-- = '\0';
      while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
      }

    }
//------------------------------------------------------------------------------
// Timer_A UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static unsigned char txBitCnt = 10;

    TA0CCR0 += UART_TBIT;                    // Add Offset to CCRx
    if (txBitCnt == 0) {                    // All bits TXed?
        TA0CCTL0 &= ~CCIE;                   // All bits TXed, disable interrupt
        txBitCnt = 10;                      // Re-load bit counter
    }
    else {
        if (txData & 0x01) {
          TA0CCTL0 &= ~OUTMOD2;              // TX Mark '1'
        }
        else {
          TA0CCTL0 |= OUTMOD2;               // TX Space '0'
        }
        txData >>= 1;
        txBitCnt--;
    }
}      

//------------------------------------------------------------------------------
// Timer_A UART - Receive Interrupt Handler
//------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static unsigned char rxBitCnt = 8;
    static unsigned char rxData = 0;
    //ADC10CTL0 |= ENC|ADC10SC;
    //temp = (35*ADC10MEM)/100;
    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) { // Use calculated branching
        case TA0IV_TACCR1:                        // TACCR1 CCIFG - UART RX
            TACCR1 += UART_TBIT;                 // Add Offset to CCRx
            if (TACCTL1 & CAP) {                 // Capture mode = start bit edge
                TACCTL1 &= ~CAP;                 // Switch capture to compare mode
                TACCR1 += UART_TBIT_DIV_2;       // Point CCRx to middle of D0
            }
            else {
                rxData >>= 1;
                if (TACCTL1 & SCCI) {            // Get bit waiting in receive latch
                    rxData |= 0x80;
                }
                rxBitCnt--;
                if (rxBitCnt == 0) {             // All bits RXed?
                    rxBuffer = rxData;           // Store in global variable
                    rxBitCnt = 8;                // Re-load bit counter
                    TACCTL1 |= CAP;              // Switch compare to capture mode
                    __bic_SR_register_on_exit(LPM0_bits);  // Clear LPM0 bits from 0(SR)
                }
            }
            break;
    }
}

// timer 1 interrup routine to change P1.6 output status
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    ADC10CTL0 |= ENC|ADC10SC;
    temp = (35*ADC10MEM)/100;
    // changes p1.6 status according to temp value
    if(temp > tmax)
    {
        P1OUT |= 0x40;    
        
    }
    else if(temp <= tmin){
        P1OUT &= ~0x40;
    }
    else 
    {
        temp = temp;
    }
     //TA1CCTL0 &= ~CCIE; // clear flag 
}
