/* ########################################################## */
/* Communications.h                                           */
/* Created : 30/04/2020 18:35:00                              */
/* Author : Jean_Louis Druilhe                                */
/* mail : jean-louis.druilhe@univ-tlse3.fr                    */
/* ########################################################## */

#ifndef COMUART_H_
#define COMUART_H_      1

#include                <avr/pgmspace.h>    /* pour utiliser PROGMEM */
#include                <avr/interrupt.h>
#include                <avr/io.h>          /* entrées-sorties fonction du type printf */
#include                <string.h>          /* traitement des chaînes de caractères */
#include                <stdint.h>
#include                <stdio.h>
#include                <stdlib.h>          /* nécessaire pour la fonction atof */
#include                "Arduino.h"         /* continet la classe Serial */


/* constantes */
#define                 DefaultConfUART       "uart1_4_8N1"     // Commande : 'uart'<d_d_dAd> (uart1_4_8N1)  19200 bauds
#define                 NoneParityText        "None"
#define                 EvenParityText        "Even"
#define                 OddParityText         "Odd"
//#define                 EvenParity            'E'
//#define                 OddParity             'O'
//#define                 NoneParity            'N'
#define                 CharSize6bit          6
#define                 CharSize7bit          7
#define                 CharSize8bit          8
#define                 CharSize9bit          9
#define                 enable_UART           true
#define                 disable_UART          false
#define                 Nbr_Car_LCD           20

/* drapeaux */
#define                 flag_ISR_TxD0         0             /* drapeau_UART */
#define                 flag_ISR_TxD1         1             /* drapeau_UART */
#define                 flag_ISR_TxD2         2             /* drapeau_UART */
#define                 flag_ISR_TxD3         3             /* drapeau_UART */
#define                 flag_ISR_RxD0         4             /* drapeau_UART */
#define                 flag_ISR_RxD1         5             /* drapeau_UART */
#define                 flag_ISR_RxD2         6             /* drapeau_UART */
#define                 flag_ISR_RxD3         7             /* drapeau_UART */
#define                 flag_ISR_UDRE0        8             /* drapeau_UART */
#define                 flag_ISR_UDRE1        9             /* drapeau_UART */
#define                 flag_ISR_UDRE2        10            /* drapeau_UART */
#define                 flag_ISR_UDRE3        11            /* drapeau_UART */
#define                 ASCII_Received        3             /* drapeau_A */
#define                 Space                 0x20          /* " " */
#define                 Null                  '\0'          // 0 ou '\0'
#define                 LF                    0x0A          // '\n'
#define                 CR                    0x0D          // '\r'
#define                 F_cpu                 16000000UL

#define                 MEGA2560Board
//#define                 UNO328Board

/* Types prédéfinis et énuméartions */
typedef enum UART_Port {        // pour définir le port de communication à solliciter
  UART0 = 0,                    // entier de type int soit 16 bits
  UART1,
  UART2,
  UART3
} UART_Port_t;

typedef enum CharSize {
  CharSize6bits = 6,
  CharSize7bits,
  CharSize8bits,
  CharSize9bits
} CharSize_t;

typedef enum ParityChoice {
  EvenParity = 0,               // 'E'
  OddParity,                    // 'O'
  NoneParity                    // 'N' entier de 16 bits de type int (multiplication checked)
} Parity_t;                     // MyParityTest = 2; if (MyParityTest == NoneParity) Serial.println(F("Test d'égalité vérifié"));

typedef enum NbrStopBits {
  OneStopBit = 1,
  TwoStopBit
} StopBit_t;

/* prototypage des fonctions de la bibliothèque */
void UART_StatusWrite(UART_Port_t, bool);
void Init_UART(UART_Port_t, uint32_t, CharSize_t, Parity_t, StopBit_t);
void uarthelp();
void separateurComUART(uint8_t, char);
uint8_t ConvUint32ToDecASCIIChar(char *, uint32_t);
void ConfigUART(String);
void UARTReading(String);
uint32_t SearchCloseValue(uint32_t);




#endif /* COMMUNICATIONS_H_ */

/* ######################################################################################################## */
// END of file
