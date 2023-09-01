
/* UNIVERSIDAD DEL VALLE DE GUATEMALA
 * DEPARTAMENTO DE INGENIERIA ELCTRONICA & MECATRONICA
 * CURSO: ELECTRONICA DIGITAL 2
 * PROYECTO No.1 RED DE SENSORES
 * 
 * File:   MAIN_TEMP.c
 * Author: DANIEL GONZALEZ 
 *
 * Created on August 10, 2023, 10:10 PM
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

#include <xc.h>
#include <stdint.h>
#include "LCD.h"                // AGREGAMOS LA LIBRERIRA DE LA LCD.
#include "I2C.h"                // AGREGAMOS LA LIBRERIRA I2C.
#include "UASART_LIB.h"         // AGREGAMOS LA LIBRERIRA UART.

#include <stdbool.h>
#include <string.h>

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// DIRECTIVAS DEL COPILADOR
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#define _XTAL_FREQ 8000000      /* DEFINIMOS LA FRECUNCIA DE RELOJ, 
                                   PARA PODER USAR LOS DELAY.*/
#define RXD2 16
#define TXD2 17
//______________________________________________________________________________
// DECLARACION DE VARIABLES
//______________________________________________________________________________

uint8_t TEMP_H;
uint8_t TEMP_L;
uint8_t SIGNO;

uint8_t PX =0;

uint8_t  UNID;
uint8_t  DECE;
uint8_t  CENT;

uint8_t POT =0;
uint8_t PD1 = 0x39;
uint8_t PD2 = 0x39;
uint8_t PD3 = 0x39;

//______________________________________________________________________________
// PROTOTIPOS DE FUNCIONES
//______________________________________________________________________________

void SETUP(void);
void DECIMAL(uint8_t V, uint8_t SELEC );

//______________________________________________________________________________
// FUNCION DE INTERRUPCIONES
//______________________________________________________________________________

//______________________________________________________________________________
// FUNCION PRINCIPAL (MAIN & LOOP)
//______________________________________________________________________________

void main(void) {
    
    SETUP();
               
//_____________________________ LOOP INFINITO __________________________________    
    
    while(1){ 
        
//---------------------- ENVIO Y RECEPCION DE DATOS I2C ------------------------    
        

        
//  DATOS SENSOR DE LUZ:
        
        I2C_Master_Start();           // INICIAR COMUNICACION.
        I2C_Master_Write(0x50);       // DIRECCION DEL ESCLAVO, WRITE (0).
        I2C_Master_Write(0);          // ENVIAMOS UN DATO.
        I2C_Master_Stop();            // DETENMEOS LA COMUNICACION.
        __delay_ms(200);
       
        I2C_Master_Start();           // INICIAR COMUNICACION.
        I2C_Master_Write(0x51);       // DIRECCION DEL ESCLAVO, READ (1).
        POT = I2C_Master_Read(0);     // LEEEMOS EL DATO Y LO GUARDAMOS, AK(0).
        I2C_Master_Stop();            // DETENMEOS LA COMUNICACION.
        __delay_ms(200);
        
//  DATOS SENSOR DE PROXIMIDAD:
        
        I2C_Master_Start();           // INICIAR COMUNICACION.
        I2C_Master_Write(0x60);       // DIRECCION DEL ESCLAVO, WRITE (0).
        I2C_Master_Write(0);          // ENVIAMOS UN DATO.
        I2C_Master_Stop();            // DETENMEOS LA COMUNICACION.
        __delay_ms(200);
       
        I2C_Master_Start();           // INICIAR COMUNICACION.
        I2C_Master_Write(0x61);       // DIRECCION DEL ESCLAVO, READ (1).
        PX = I2C_Master_Read(0);     // LEEEMOS EL DATO Y LO GUARDAMOS, AK(0).
        I2C_Master_Stop();            // DETENMEOS LA COMUNICACION.
        __delay_ms(200);
        
//  DATOS SENSOR DE TEMPERATURA:

        I2C_Master_Start();             // INICIAR COMUNICACION.
        I2C_Master_Write(0b10010000);   // DIRECCION DEL ESCLAVO, WRITE 0.
        I2C_Master_Write(0x00);         // SELECCIONAMOS EL REGISTRO.
        I2C_Master_RepeatedStart();     // REPETIMOS EL INICIO DE COMUNICACION.
        I2C_Master_Write(0b10010001);   // DIRECCION DEL ESCLAVO, READ 1.
        TEMP_H = I2C_Master_Read(0);    // LEEEMOS EL REGISTRO Y LO GUARDAMOS.
        TEMP_L = I2C_Master_Read(0);    // LEEEMOS EL REGISTRO Y LO GUARDAMOS.
        I2C_Master_Stop();              // DETENMEOS LA COMUNICACION.
        __delay_ms(200);
        
        SIGNO = 43;                     // SIGNO POSITIVO (+).
        
//-------------------------- TEMPERATURA NEGATIVA ------------------------------
        
        if (TEMP_H>>7 == 1){         // VERIFICAMOS SI ES UN VALOR NEGATIVO.
            TEMP_H = ~TEMP_H +1;     // TRANSFOMACION DE COMPLEMTO A2 A BINARIO.
            SIGNO = 45;              // SIGNO NEGATIVO (-).
        }
        
             
//________________________ ESCRITURA EN PANTALLA LCD ___________________________         
       
//----------------------------- DATOS SENOSR 1 ---------------------------------         
        
        DECIMAL(TEMP_H,0);           // SEPARAMOS EL VALOR EN DIGITOS.
        Lcd_Set_Cursor(1,1);         // SELECIONAMOS LA POSICION (1,1) DEL LCD. 
        Lcd_Write_String("TM");      // MOSTRAMOS EL NOMBRE DEL SENSOR "S1".   
        Lcd_Set_Cursor(2,1);         // SELECIONAMOS LA POSICION (2,1) DEL LCD.
        Lcd_Write_Char(SIGNO);       // MOSTRAMOS EL SIGNO DEL VALOR. 
        Lcd_Write_Char(DECE);        // PRIMER DIGITO DE LA TEMPERATUA. 
        Lcd_Write_Char(UNID);        // SEGUNDO DIGITO DE LA TEMPERATURA.
        Lcd_Write_Char(223);         // "º"
        Lcd_Write_Char(67);          // "C"        

//----------------------------- DATOS SENOSR 2 ---------------------------------         
        
        DECIMAL(POT,1);              // SEPARAMOS EL VALOR EN DIGITOS.
        Lcd_Set_Cursor(1,7);         // SELECIONAMOS LA POSICION (1,7) DEL LCD. 
        Lcd_Write_String("F1");      // MOSTRAMOS EL NOMBRE DEL SENSOR "S2".   
        Lcd_Set_Cursor(2,7);         // SELECIONAMOS LA POSICION (2,1) DEL LCD.
        Lcd_Write_Char(PD1);         // MOSTRAMOS EL SIGNO DEL VALOR. 
        Lcd_Write_Char(PD2);         // PRIMER DIGITO DE LA TEMPERATUA. 
        Lcd_Write_Char(PD3);         // SEGUNDO DIGITO DE LA TEMPERATURA.

//----------------------------- DATOS SENOSR 3 ---------------------------------         
        
        Lcd_Set_Cursor(1,11);        // SELECIONAMOS LA POSICION (1,11) DEL LCD. 
        Lcd_Write_String("I1");     // MOSTRAMOS EL NOMBRE DEL SENSOR "S3".   
        Lcd_Set_Cursor(2,11);        // SELECIONAMOS LA POSICION (2,1) DEL LCD.
        Lcd_Write_Char(PORTEbits.RE0 + 0x30);
        if(PORTEbits.RE0 == 1){         // REVISAMOS HAY MOVIMIENTO DENTRO. 
            Lcd_Write_String("..""");} // MOSTRAMOS EN LA LCD "ALERTA".      
        
        else{
            Lcd_Write_String("OK");}  // MOSTRAMOS EN LA LCD "NORMAL". 
          
          
        
        
//------------------- DESPLIEGUE DE VALORES EN LA CONSOLA ----------------------    
    
        //__delay_ms(1000); 
        //printf(0x39);
        UART_WRITE_CHAR(0x39);

      
        
   
        
void UART_Init() {
    TRISC6 = 0; // Configura TX como salida
    TRISC7 = 1; // Configura RX como entrada
    
    SPBRG = 25; // Calcula el valor de SPBRG para una velocidad de 9600 bps (con Fosc = 8MHz)
    
    TXSTAbits.TXEN = 1; // Habilita el módulo de transmisión
    TXSTAbits.SYNC = 0; // Modo asíncrono
    TXSTAbits.BRGH = 1; // Alta velocidad de transmisión
    
    RCSTAbits.SPEN = 1; // Habilita el puerto serial
    RCSTAbits.CREN = 1; // Habilita la recepción continua
}

void UART_Write_Char(char data) {
    while(!TXIF); // Espera hasta que el buffer de transmisión esté vacío
    TXREG = data; // Envía el dato
}

char UART_Read_Char() {
    while(!RCIF); // Espera hasta que se reciba un dato
    return RCREG; // Retorna el dato recibido
}

void main() {
    OSCCON = 0x72; // Configura el oscilador interno a 8MHz

    UART_Init(); // Inicializa el UART
    
    while (1) {
        // Tu lógica y procesamiento aquí
        
        char dataToSend = 'POT'; // Datos a enviar
        char dataToSend = 'PX';
       // char dataToSend = 'TEMP_H';
        
        
        UART_Write_Char(dataToSend);
        
        __delay_ms(1000); // Espera antes de enviar el próximo dato
    }
}   
        
        /*
         * 
    

//------------------ DESPLIEGUE DE VALORES SERIALES ----------------------------
void UART_Init() {
    TRISC6 = 0; // Configura TX como salida
    TRISC7 = 1; // Configura RX como entrada
    
    SPBRG = 25; // Calcula el valor de SPBRG para una velocidad de 9600 bps (con Fosc = 8MHz)
    
    TXSTAbits.TXEN = 1; // Habilita el módulo de transmisión
    TXSTAbits.SYNC = 0; // Modo asíncrono
    TXSTAbits.BRGH = 1; // Alta velocidad de transmisión
    
    RCSTAbits.SPEN = 1; // Habilita el puerto serial
    RCSTAbits.CREN = 1; // Habilita la recepción continua
}

void UART_Write_Char(char data) {
    while(!TXIF); // Espera hasta que el buffer de transmisión esté vacío
    TXREG = data; // Envía el dato
}

void main() {
    OSCCON = 0x72; // Configura el oscilador interno a 8MHz

    UART_Init(); // Inicializa el UART
    
    while (1) {
        
        char dataToSend = 'S1'; // Datos a enviar
        char dataToSend = 'SIGNO'; 
        char dataToSend = 'DECE'; 
        char dataToSend = 'UNI'; 
        
        UART_Write_Char(dataToSend);
        
        __delay_ms(1000); // Espera antes de enviar el próximo dato
    }
}

*/
        

//-------------------------- ACTIVASION MOTOR DC -------------------------------    
        if(TEMP_H >= 27){              // REVISAMOS LA CANTIDAD DE LUZ ES BAJA.  
            PORTA = 0xFF;}             // ENCENDEMOS EL VENTILADOR.     
        else{
            PORTA = 0x00;}             // APGAMOS EL VENTILADOR.
    }
    return;
}
//______________________________________________________________________________
// FUNCION DE SEPARACION DE DIGITOS Y CONVERSION ASCII.
//______________________________________________________________________________

void DECIMAL(uint8_t V, uint8_t SELEC ){

//-------------------- DECLARACION DE VARIABLES LOCALES ------------------------
    
    uint16_t VOLT;

//--------------------------- SEPARCION DE DIGITOS -----------------------------
   
    VOLT = (uint16_t)(V*1);                  // RESOLUCION DE 1.00.
    CENT = VOLT/100;                         // SEPARAMOS EL PRIMER DIGITO.
    DECE = (VOLT - CENT*100)/10;             // SEPARAMOS EL SEGUNDO DIGITO.
    UNID = (VOLT - CENT*100 - DECE*10)/1;    // SEPARAMOS EL TERCER DIGITO.

//--------------- CONVERSION ASCII & SELECCION DE POTENCIOMETRO ----------------
    if(SELEC == 0){
        CENT = CENT + 0x30;
        DECE = DECE + 0x30;
        UNID = UNID + 0x30;}   
    
    if(SELEC == 1){
        PD1 = CENT + 0x30;
        PD2 = DECE + 0x30;
        PD3 = UNID + 0x30;}      
}               

//______________________________________________________________________________
// FUNCION DE CONFIGURACION
//______________________________________________________________________________
void SETUP(void){
    
//------------------- CONFIGURACION DE ENTRADAS Y SALIDAS ----------------------   
    ANSEL  = 0x00;             // SIN ENTRADAS ANALOGICAS, SOLO DIGITALES.   
    ANSELH = 0x00;        

    TRISA = 0b00000000;        // DECLARAMOS EL PORTA COMO SALIDAS (MOTOR).
    TRISB = 0x00;              // DECLARAMOS EL PORTB COMO SALIDAS (LCD).
    TRISD = 0x00;              // DECLARAMOS EL PORTD COMO SALIDAS.
    TRISC = 0b10000000;        // Declaramos el PORTC.bit7 como entrada.
    TRISE = 0b111;
    
    PORTA = 0x00;              // LIMPIAMOS LOS PUERTOS.
    PORTD = 0x00;  
    PORTB = 0x00;
    PORTE = 0x00;

//-------------------------- CONFIGURACION DE LCD ------------------------------
    unsigned int a;
    Lcd_Init();    

//--------------------------- CONFIGURACION DE TX & RX ------------------------- 
    UART_TX_CONFIG(9600);    // Configuracion y activacion del TX.
    UART_RX_CONFIG(9600);    // Configuracion y activacion del RX.
    
    
//-------------------------- CONFIGURACION DEL I2C -----------------------------   
    I2C_Master_Init(100000);   // INICICALIZAMOS COMUNICACION I2C.    
}
//______________________________________________________________________________
//______________________________________________________________________________



