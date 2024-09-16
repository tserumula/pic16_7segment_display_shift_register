
// PIC16F1459 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = CLKDIV6 // CPU System Clock Selection Bit (CPU system clock divided by 6)
#pragma config USBLSCLK = 48MHz // USB Low Speed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multiplier Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 4000000 //4MHz

// 74HC595 Connections

#define SHIFT_CLK PORTCbits.RC3   // Shift Register clock (SRCLK)
#define SHIFT_LATCH PORTCbits.RC4   // Latch Pin Clock (RCLK)

#define SHIFT0_DATA PORTAbits.RA5   // Data pin (SER)
#define SHIFT1_DATA PORTAbits.RA4   // Data pin (SER)
#define SHIFT2_DATA PORTCbits.RC5   // Data pin (SER)

#define PUSH_BUTTON PORTBbits.RB5 //The middle button 

//_____ Global variables ______

uint8_t batteryLevel = 0; //battery level
uint8_t batteryDigit0; // first digit of battery level
uint8_t batteryDigit1; // second digit
uint8_t batteryDigit2; // third digit

uint8_t segPattern0; // 7-segment pattern for displaying first digit
uint8_t segPattern1; // 7-segment pattern for displaying second digit
uint8_t segPattern2;// 7-segment pattern for displaying third digit

uint8_t turnOnDisplay = 0; // When = 1, we turn on the display


//Function below is for the Bit pattern for common-cathode 7 segment display
uint8_t get7SegmentPattern(uint8_t digit) {
    uint8_t pattern;
    switch (digit) {
        case 0: pattern = 0b00111111; break; // 0: a, b, c, d, e, f
        case 1: pattern = 0b00000110; break; // 1: b, c
        case 2: pattern = 0b01011011; break; // 2: a, b, d, e, g
        case 3: pattern = 0b01001111; break; // 3: a, b, c, d, g
        case 4: pattern = 0b01100110; break; // 4: b, c, f, g
        case 5: pattern = 0b01101101; break; // 5: a, c, d, f, g
        case 6: pattern = 0b01111101; break; // 6: a, c, d, e, f, g
        case 7: pattern = 0b00000111; break; // 7: a, b, c
        case 8: pattern = 0b01111111; break; // 8: a, b, c, d, e, f, g
        case 9: pattern = 0b01101111; break; // 9: a, b, c, d, f, g
        default: pattern = 0b00000000; break; // Turn off all segments if invalid digit
    }
    
    return pattern;
}

// Shift register function for data, clock, and latch signals
void displayDigits( uint8_t digit0, uint8_t digit1, uint8_t digit2 ){
    int i;
    for ( i=0 ; i<8 ; i++ ){

        SHIFT0_DATA = (digit0 >> i) & (0x01);
        SHIFT1_DATA = (digit1 >> i) & (0x01);
        SHIFT2_DATA = (digit2 >> i) & (0x01);
        
        //Pulse the Shift register clock pin 
        SHIFT_CLK = 1; 
        __delay_us(500);
        SHIFT_CLK = 0;
        __delay_us(500);
    }
    
    //All bits shifted in. Now we pulse the Latch pin
    SHIFT_LATCH = 1;
    __delay_us(500);
    SHIFT_LATCH = 0;
    
}

// Function to set the external interrupt for our push button
void setButtonInterrupt(){
    // Enable Interrupt-on-Change on RB5
    IOCBN = 0x0; // Disable Interrupt on Change for PORTB FALLING EDGE REGISTER
    IOCBPbits.IOCBP5 = 1; // Enable Interrupt on Change for PORTB RISING EDGE REGISTER
    
    INTCONbits.INTF = 0; // Clear external Interrupt flag
            
    // Enable global and peripheral interrupts
    INTCONbits.IOCIE = 1;  // Enable IOC interrupt
    INTCONbits.PEIE = 0;   // Disable peripheral interrupts
    INTCONbits.GIE = 1;    // Enable global interrupts
}

// our Interrupt service routine
void __interrupt() ISR(void) {
    
    if ( INTCONbits.IOCIF  ) { //Interrupt-on-change flag
        
        if( IOCBFbits.IOCBF5 ){
            // Clear the flag
            IOCBFbits.IOCBF5 = 0;
            
            turnOnDisplay = 1;
        }
        
        INTCONbits.INTF = 0; // Clear external Interrupt flag
        INTCONbits.IOCIF = 0; //Clear flag
    }
}

// ____________ Program Start _________________

void main(void) {
    
    //Set main Clock (Internal oscillator block))
    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 1;
    
    //Set internal clock frequency to 4MHz
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF3 = 1;

    __delay_ms(500);
    
    //Set RA5, A4, C5, C4, C3 as output pins
    
    TRISAbits.TRISA5 = 0;
    TRISAbits.TRISA4 = 0;
    
    TRISCbits.TRISC5 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC3 = 0;
    
    TRISBbits.TRISB7 = 0;
    
    LATAbits.LATA5 = 0;
    LATAbits.LATA4 = 0;
    
    LATCbits.LATC5 = 0;
    LATCbits.LATC4 = 0;
    LATCbits.LATC3 = 0;
    
    LATBbits.LATB7 = 0;
    
    //Set RB5 as input
    TRISBbits.TRISB5 = 1;
    RCSTAbits.SPEN = 0; //serial port disabled
    PIE1bits.TXIE = 0; //Eusart Transmit interrupt off
    
    //Disable ADC operation on port A, port B and Port C
    ADCON0 = 0x0;
    ANSELA = 0x0;
    ANSELB = 0x0;
    ANSELC = 0x0;
    
    //Setup button interrupt
    setButtonInterrupt();
    
    while(1){
      
      LATBbits.LATB7 = 1; //connected LED pin (for testing)
        
      batteryDigit0 = batteryLevel / 100; //get first digit
      batteryDigit1 = batteryLevel / 10; // get second digit
      batteryDigit2 = batteryLevel % 10; // third
      
      if( batteryDigit0 == 0 ){
        //We turn off the first digit display if it is == 0
        segPattern0 = get7SegmentPattern(11); // Will return 0000000
      
      }else{
        segPattern0 = get7SegmentPattern( batteryDigit0 ); // 7 segment bit pattern
      
      }
      
      segPattern1 = get7SegmentPattern( batteryDigit1 ); // 7 segment bit pattern
      segPattern2 = get7SegmentPattern( batteryDigit2 ); // 7 segment bit pattern
      
      if( turnOnDisplay == 1 ){     
        
          displayDigits( segPattern0, segPattern1, segPattern2 );
           __delay_ms(10000); //10 seconds
           
           turnOnDisplay = 0;
           
      }else{
          
        displayDigits( 0x0, 0x0, 0x0 ); //keep displays off   
         __delay_ms(2000); // 2 seconds
      
      }
      
    }
    
    return;
}
