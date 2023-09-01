
/* UNIVERSIDAD DEL VALLE DE GUATEMALA
 * DEPARTAMENTO DE INGENIERIA ELCTRONICA & MECATRONICA
 * CURSO: ELECTRONICA DIGITAL 2
 * PROYECTO No.1 RED DE SENSORES
 * 
 * File:   MAIN_SL2_LIGHT.c
 * Author: DANIEL GONZALEZ
 *
 * Created on August 10, 2023, 10:20 PM
 */

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// CONFIG1
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#pragma config FOSC  = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE  = OFF      // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP    = OFF      // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD   = OFF      // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO  = OFF      // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP   = OFF      // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// CONFIG2
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT   = OFF      // Flash Program Memory Self Write Enable bits (Write protection off)

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// LIBRERIAS
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <pic16f887.h>
#include "I2C.h"
#include "ADC_LIB.h"
#include <xc.h>

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// DIRECTIVAS DEL COPILADOR
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#define _XTAL_FREQ 8000000

//______________________________________________________________________________
// DECLARACION DE VARIABLES
//______________________________________________________________________________

uint8_t z;
uint8_t dato;
uint8_t V;
uint8_t d_reciv;

//______________________________________________________________________________
// PROTOTIPOS DE FUNCIONES
//______________________________________________________________________________

void setup(void);

//______________________________________________________________________________
// FUNCION DE INTERRUPCIONES
//______________________________________________________________________________
void __interrupt() isr(void){
    
//----------------------------- INTERRUPCION I2C -------------------------------     
    if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;      // Clear the overflow flag
            SSPCONbits.WCOL = 0;       // Clear the collision bit
            SSPCONbits.CKP = 1;        // Enables SCL (Clock)
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            z = SSPBUF;                // Lectura del SSBUF para limpiar el buffer y la bandera BF
            PIR1bits.SSPIF = 0;        // Limpia bandera de interrupción recepción/transmisión SSP
            SSPCONbits.CKP = 1;        // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);    // Esperar a que la recepción se complete
            d_reciv = SSPBUF;          // Guardar en el PORTD el valor del buffer de recepción
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;                // Lectura del SSBUF para limpiar el buffer y la bandera BF
            BF = 0;
            SSPBUF = V;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }

//----------------------------- INTERRUPCION ADC -------------------------------     
    
    if(ADIF == 1){                     // REVISAMOS LA BANDERA DEL ADC.   
        if(ADCON0bits.CHS == 0){       // REVISAMOS SI ESTA EN EL CANAL 0. 
             V = adc_read();}          // ASIGNAMOS EL VALOR ADC A UNA VARIABLE.              
    }        
}

//______________________________________________________________________________
// FUNCION PRINCIPAL (MAIN & LOOP)
//______________________________________________________________________________

void main(void) {
    
    setup();
    
//_____________________________ LOOP INFINITO __________________________________    
 
    while(1){
       
//---------------------- REVISION DE CANTIDAD DE LUZ ---------------------------   
        
        if(V > 120){                   // REVISAMOS LA CANTIDAD DE LUZ ES BAJA.  
            PORTB = 0x01;              // ENCENDEMOS LA LUZ.     
        }
        else{
            PORTB = 0x00;              // APGAMOS LA LUZ.
        }
        
//------------------------------- REVISION ADC ---------------------------------   
        
        if(ADCON0bits.GO == 0){        // REVISAMOS SI YA TERMINO LA CONVERSION.  
            __delay_us(50);
            ADCON0bits .GO = 1;        // ENCENDEMOS LA CONVERSION NUEVAMENTE.     
        }
    }
    return;
}

//______________________________________________________________________________
// FUNCION DE CONFIGURACION
//______________________________________________________________________________

void setup(void){

//------------------- CONFIGURACION DE ENTRADAS Y SALIDAS ----------------------
    
    ANSEL  = 0x01;            // SIN ENTRADAS ANALOGICAS, SOLO DIGITALES.   
    ANSELH = 0x01;        

    TRISA = 0x01;             // DECLARAMOS EL PORTA COMO SALIDAS (LCD).
    TRISB = 0x00;             // DECLARAMOS EL PORTD COMO SALIDAS.
    
    PORTA = 0x00;             // LIMPIAMOS LOS PUERTOS.
    PORTB = 0x00;    
    // PORTC = 0X00;             //Limpiamos las banderas del puerto C 
    
//-------------------------- CONFIGURACION DE ADC ------------------------------
    
    adc_init(2,0,0);          // (FOSC/32), VDD, VSS. 
    adc_start(0);             // CHANEL 0, ENCENDEMOS ADC, INICIAMOS CONVERSION. 
    
//-------------------------- CONFIGURACION DEL I2C -----------------------------

    I2C_Slave_Init(0x50);   
    
//---------------------- CONFIGURACION DE INTERRUPCIONES -----------------------
      
    INTCONbits.GIE  = 1;      // HABILITAMOS LA INTERRUPCION GLOBALES.
    PIE1bits.RCIE   = 1;
    PIR1bits.ADIF   = 0;      // LIMPIAMOS LA BANDERA DEL ADC.  
}
//______________________________________________________________________________
//______________________________________________________________________________
