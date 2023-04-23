/*
 * File:   Principal.c
 * Author: schwe
 *
 * Created on 21 de abril de 2023, 09:01 AM
 */

//CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT                        //Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF                                   //Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF                                  //Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF                                  //RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                                     //Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                                    //Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF                                  //Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF                                   //Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF                                  //Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                                    //Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V                               //Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                                    //Flash Program Memory Self Write Enable bits (Write protection off)

//Librerías
#include <xc.h>

//Definición de variables
#define _XTAL_FREQ 4000000
#define dirEEPROM 0x04                                      //Direccion de EEPROM
uint8_t pot;
int antiR;

//Prototipos
void setup(void);
void main(void);
uint8_t leerEEPROM (uint8_t direccion);
void escribirEEPROM (uint8_t direccion, uint8_t data);

//Interrupcion
void __interrupt() isr(void) {
    if (INTCONbits.RBIF){
        PORTB = PORTB;
        PORTEbits.RE0 = 1;                                  //Encender indicador de WAKE
    }
    INTCONbits.RBIF = 0;                                    //Resetear flags
    return;
}

//Setup General
void setup(void){
    //Oscilador
    OSCCON = 0B01100000;                                    //Oscilador a 4Mhz
    
    //Interrupciones
    INTCON = 0B10001000;                                    //Int globales, PORTB activadas
        
    //ADC
    ADCON0 = 0B01001101;                                    //Fosc/8 (2us), AnCH3, ADC encendido 
    ADCON1 = 0B00000000;                                    //Voltajes de referencia y formato a la izquierda
    ANSELH = 0;
    ANSEL = 0B00001000;                                     //Bit A3 como analogico
    
    //PORTB
    OPTION_REG = 0B00000000;
    TRISB = 0B00000111;
    WPUB = 0B00000111;
    IOCB = 0B00000010;
    
    //Entradas y salidas
    TRISA = 0B00001000;                                     //Entradas
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    
    //Valores iniciales de variables y puertos
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    return;
}

//Loop
void main(void) {
    setup();
    while(1){
        if (ADCON0bits.GO == 0){                            //Si ADC inició
            pot = ADRESH;
            PORTC = pot;
            __delay_ms(1);
            ADCON0bits.GO = 1;                              //Volver a iniciar
        }
        PORTD = leerEEPROM(dirEEPROM);                      //Leer valores de la EEPROM
        if (!PORTBbits.RB0){                                //Si se apacho boton SLEEP
            PORTEbits.RE0 = 0;                              //Apagar indicador de WAKE
            SLEEP();
        }
        if (!PORTBbits.RB2){                                //Antirrebote
            antiR = 1;
        }
        if (PORTBbits.RB2 && antiR == 1){                   //Si el boton esta apagado y el antirrebote encendido
            escribirEEPROM (dirEEPROM, pot);                //Escribir en la EEPROM
            antiR = 0;                                      //Apagar antirrebote
        }
    }
}

//Funciones
uint8_t leerEEPROM (uint8_t direccion){
    EEADR = direccion;                                      //Guardar direccion
    EECON1bits.EEPGD = 0;                                   //Ir a la direccion
    EECON1bits.RD = 1;                                      //Iniciar lectura
    return EEDATA;                                          //Regresar valor leido
}

void escribirEEPROM (uint8_t direccion, uint8_t data){
    if (!EECON1bits.WR){                                    //Si ya termino de escribir
        EEADR = direccion;                                  //Guardar direccion
        EEDATA = data;                                      //Guardar data
        EECON1bits.EEPGD = 0;                               //Ir a la direccion
        EECON1bits.WREN = 1;                                //Habilitar escritura
        INTCONbits.GIE = 0;                                 //Deshabilitar interrupciones
        EECON2 = 0x55;
        EECON2 = 0xAA;
        EECON1bits.WR = 1;                                  //Iniciar escritura
        INTCONbits.GIE = 1;                                 //Habilitar interrupciones
        EECON1bits.WREN = 0;                                //Deshabilitar escritura
    }
    return;
}