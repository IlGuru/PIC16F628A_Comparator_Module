/*
 * File:   main.c
 * Author: Stefano
 *
 * Created on 9 settembre 2015, 15.21
 */


// PIC16F628A Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

#define _XTAL_FREQ  4000000

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

void main(void) {
    
    unsigned char v_ref = 0;
    
    //  -----------------------------------------------------------------------------------------
    //  Configuro il modulo comparatore:
    
    /*
     * Comparator interrupts should be disabled
     * during a Comparator mode change,
     * otherwise a false interrupt may occur.
     */

    //  Disabilito gli interrupt
    INTCONbits.GIE  = 0;        /*  GIE: Global Interrupt Enable bit
                                 *      1 = Enables all un-masked interrupts    
                                 *      0 = Disables all interrupts
                                 */
    
    //  Disabilito gli interrupt relativi alle periferiche
    INTCONbits.PEIE = 0;        /*  PEIE: Peripheral Interrupt Enable bit 
                                 *      1 = Enables all un-masked peripheral interrupts
                                 *      0 = Disables all peripheral interrupts
                                 */
    
    //  Disabilito gli interrupt relativi al comparatore
    PIE1bits.CMIE   = 0;        //  CMIE: Comparator Interrupt Enable bit
    PIR1bits.CMIF   = 0;        /*  TheCMIF bit, PIR1<6>, is the comparator interrupt flag.
                                 *  CMIF: Comparator Interrupt Flag bit
                                 *      1 = Comparator output has changed
                                 *      0 = Comparator output has not changed
                                 */
            
    //  Modalità di funzionamento del modulo comparatore:
    CMCONbits.CM    = 0b010;    //  CM<2:0> = 010 Four Inputs Multiplexed to Two Comparators
    CMCONbits.CIS   = 0;        /*  CIS: Comparator Input Switch bit
                                    When CM<2:0>: = 001
                                        Then:
                                            1 = C1 VIN- connects to RA3
                                            0 = C1 VIN- connects to RA0
                                    When CM<2:0> = 010
                                        Then:
                                            1 = C1 VIN- connects to RA3
                                                C2 VIN- connects to RA2
                                            0 = C1 VIN- connects to RA0
                                                C2 VIN- connects to RA1
                                 */
    CMCONbits.C1INV = 0;        /*  C1INV: Comparator 1 Output Inversion bit
                                        1 = C1 Output inverted
                                        0 = C1 Output not inverted
                                 */
    CMCONbits.C2INV = 0;        /*  C2INV: Comparator 2 Output Inversion bit
                                        1 = C2 Output inverted
                                        0 = C2 Output not inverted
                                 */

    //  -----------------------------------------------------------------------------------------

    //  -----------------------------------------------------------------------------------------
    //  Configuro il modulo Voltage reference:

    VRCONbits.VREN  = 0;        /*  0 = VREF circuit powered down, no IDD drain
                                    1 = VREF circuit powered on
                                */
    VRCONbits.VROE  = 1;        /*  0 = VREF is disconnected from RA2 pin
                                    1 = VREF is output on RA2 pin
                                */
    VRCONbits.VRR   = 0;        /*  0 = High range
                                    1 = Low range
                                */
    VRCONbits.VR    = 0;        /*  When VRR = 0: VREF = 1/4 * VDD + (VR<3:0>/ 32) * VDD
                                    When VRR = 1: VREF = (VR<3:0>/ 24) * VDD
                                */
    
    //  -----------------------------------------------------------------------------------------

    TRISA           = 0b00111111;   //  RA6-RA7 pin di output.
    PORTA           = 0b00000000;   //  Inizializzo PORTA

    TRISB           = 0b00000000;   //  RA6-RA7 pin di output.
    PORTB           = 0b00000000;   //  Inizializzo PORTA
    
    VRCONbits.VREN  = 1;            //  Attivo il modulo Voltage Reference
    
    while(1) {

        for ( v_ref = 0; v_ref < 16; v_ref++ ) {    

            //  Genero una rampa di 16 livelli di VRef
            VRCONbits.VR  = v_ref;

            //  La VRef è presente sul pin AN2        
            
            PORTAbits.RA6 = CMCONbits.C1OUT;    //  Se AN0 > VRef -> RA6=1, altrimenti 0
            PORTAbits.RA7 = CMCONbits.C2OUT;    //  Se AN1 > VRef -> RA7=1, altrimenti 0

            __delay_us(500);
        }

    }
    
    return;
}
