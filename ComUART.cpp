/* ########################################################## */
/* API for communications using UART and other functions      */
/* Communications.c                                           */
/* Created : 23/09/2014 10:03:27                              */
/* Author : Jean_Louis Druilhe                                */
/* mail : jean-louis.druilhe@univ-tlse3.fr                    */
/* ########################################################## */
/* 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include                  "ComUART.h"
#include                  "Fonctions.h"

/*************************** Variables externes utilisées par les procédures d'interruption ***************************/                       
extern volatile uint8_t       compt1, compt2, compt3, compt4, compt5, cmpt_100us, cmpt_5ms; // variables utilisées par les ISR des compteurs
extern volatile uint16_t      cmpt1, cmpt2, cmpt3, cmpt4, cmpt5;

/*************************** Local variables used by interrupts subroutines ***************************/         
volatile uint16_t       drapeau_UART;                   // Utilisé par les procédures d'interruption liées à l'UART
volatile uint8_t        ReceivedChar;

// Local variables specific to this module
uint8_t                 Indice_carac;
uint8_t                 Byte_Read, UART_Input;    /* octet utilisé pour la lecture et pour l'écriture d'un seul octet */
uint16_t                UBRR_value;
uint8_t                 UBRR_Low, UBRR_High;
const uint32_t          BaudRates[] PROGMEM = {115200UL, 57600UL, 38400UL, 28800UL, 19200UL, 14400UL, 9600UL, 4800UL, 2400UL, 1200UL};
const uint8_t           DelayBetween2Chars[] PROGMEM = {1, 2, 2, 3, 3, 4, 4, 5, 5, 6};    // factor multiplied by 100 µs
uint8_t                 UART_number, BaudRate_index, Nbr_bits, StopBits;
uint8_t                 Delay2Chars;                  // Contention window
uint32_t                BaudRate, BaudRateCorrected;
char                    ParityChar;
char                    TabAsciiUnsignedInteger[20];  // 2^64 = 18 446 744 073 709 551 616
uint8_t                 ScratchComUART_8bits;
uint16_t                ScratchComUART_16bits;
uint32_t                ScratchComUART_32bits;
uint32_t                ResDivision_UART;             // 0 to 4,294,967,295
uint8_t                 ResModulo_UART;
char                    ComAsciiArray[20];
uint8_t                 FlagReader;
uint8_t                 BuffIn[100];                  // global array as buffer for received frames
uint8_t                 BuffOut[100];                 // global array as buffer for transmission frames
uint16_t                TestFlags16bits;
const char              *ptrConstChar;

// Control commands for oxymeter
const char              CdeVersion[] PROGMEM = "VER";                       // cde_VER 1




/* ##################### Vecteurs d'interruptions ##################### */
/********************************************************************************************/
/* Programmes d'interrution des 3 UARTs. UART1, UART2 et UART3.                             */
/********************************************************************************************/
/* Lien pour identifier les procédures d'interruptions officiellement reconnues par le compilateur AVR Studio */
// https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__avr__interrupts.html
/* ------------------------------------- UART1 ------------------------------------- */
/********************************************************************************************/
/* interruption liée à la réception d'un seul caractère dans le buffer UDR1 de l'UART1.     */
/* Principe : ce drapeau vaut 1 quand le buffer contient une donnée valide et il est remis
/* à zéro quand */
/********************************************************************************************/
ISR(USART1_RX_vect) {
  ReceivedChar = UDR1;
  drapeau_UART |= (1<<flag_ISR_RxD1);
}
/********************************************************************************************/
/* interruption liée à la transmission d'un seul caractère dans le buffer de l'UART1.       */
/* Principe : ce drapeau vaut 1 quand le buffer est vide.                                   */
/********************************************************************************************/
ISR(USART1_TX_vect) {
  drapeau_UART |= (1<<flag_ISR_TxD1);
}
/********************************************************************************************/
/* interruption liée à la non présence d'un octet dans le buffer d'émission UDR1 et qui sera*/
/* remis à 0 quand on y écrit un octet. Ce vecteur d'interruption est toujours appelé quand */
/* UDR1 est vide et n'est pas remis à 0 quand la fonction d'interruption est appelée au     */
/* contraire du drapeau TXCn. A éviter étant démuni d'une remise à 0 matérielle automatique.*/
/********************************************************************************************/
ISR(USART1_UDRE_vect) {
  drapeau_UART |= (1<<flag_ISR_UDRE1);
}
/* ------------------------------------- UART2 ------------------------------------- */
/********************************************************************************************/
/* interruption liée à la réception d'un seul caractère dans le buffer UDR2 de l'UART2.     */
/********************************************************************************************/
#ifdef MEGA2560Board
ISR(USART2_RX_vect) {
  ReceivedChar = UDR2;
  drapeau_UART |= (1<<flag_ISR_RxD2);
}
#endif
/********************************************************************************************/
/* interruption liée à la transmission d'un seul caractère dans le buffer de l'UART2.       */
/* Principe : ce drapeau vaut 1 quand la donnée du registre UDR2 a été sérialisée.          */
/********************************************************************************************/
#ifdef MEGA2560Board
ISR(USART2_TX_vect) {
  drapeau_UART |= (1<<flag_ISR_TxD2);
}
#endif
/********************************************************************************************/
/* interruption liée à la non présence d'un octet dans le buffer d'émission UDR1 et qui sera*/
/* remis à 0 quand on y écrit un octet. Ce vecteur d'interruption est toujours appelé quand */
/* UDR1 est vide et n'est pas remis à 0 quand la fonction d'interruption est appelée au     */
/* contraire du drapeau TXCn.                                                               */
/********************************************************************************************/
#ifdef MEGA2560Board
ISR(USART2_UDRE_vect) {
  drapeau_UART |= (1<<flag_ISR_UDRE2);
}
#endif
/* ------------------------------------- UART3 ------------------------------------- */
/********************************************************************************************/
/* interruption liée à la réception d'un seul caractère dans le buffer UDR3 de l'UART3.     */
/* Principe : ce drapeau vaut 1 quand le buffer contient une donnée valide.                 */
/********************************************************************************************/
#ifdef MEGA2560Board
ISR(USART3_RX_vect) {
  ReceivedChar = UDR3;
  drapeau_UART |= (1<<flag_ISR_RxD3);
}
#endif
/********************************************************************************************/
/* interruption liée à la transmission d'un seul caractère dans le buffer de l'UART3.       */
/* Principe : ce drapeau vaut 1 quand la donnée du registre UDR3 a été sérialisée.          */
/********************************************************************************************/
#ifdef MEGA2560Board
ISR(USART3_TX_vect) {
  drapeau_UART |= (1<<flag_ISR_TxD3);
}
#endif
/********************************************************************************************/
/* interruption liée à la transmission  d'un seul caractère dans le buffer de l'UART3.      */
/* Principe : ce drapeau vaut 1 quand le buffer est vide.                                   */
/********************************************************************************************/
#ifdef MEGA2560Board
ISR(USART3_UDRE_vect) {
  drapeau_UART |= (1<<flag_ISR_UDRE3);
}
#endif
/* ##################### Fin des vecteurs d'interruptions ##################### */
/****************************************************************************************************/
/* Fonction pour activer 1 UART parmi 3. L'UART0 ne devra pas être configuré car il est utilisé par */
/* l'IDE Arduino.                                                                                   */
/****************************************************************************************************/
void UART_StatusWrite(UART_Port_t UART_selected, bool State) {
  switch (UART_selected) {
    case UART1:
      if (State == enable_UART) UCSR1B |= ((1<<TXEN1)|(1<<RXEN1));
      else UCSR1B &= ~((1<<TXEN1)|(1<<RXEN1));
      break;
    case UART2:
      if (State == enable_UART) UCSR2B |= ((1<<TXEN2)|(1<<RXEN2));
      else UCSR2B &= ~((1<<TXEN2)|(1<<RXEN2));
      break;
    case UART3:
      if (State == enable_UART) UCSR3B |= ((1<<TXEN3)|(1<<RXEN3));
      else UCSR3B &= ~((1<<TXEN3)|(1<<RXEN3));
      break;
    default:
      break;
  }
}
/****************************************************************************************************/
/* Function to initiate one UART port among 4. Regardless the rate transfert selected this function */
/* calculates the content of UBRRH and UBRRL registers to size the prescaler. We can configure UART1*/
/* UART2 and UART3 without UART0 which is reserved by the boot program of Arduino.                  */
/* Default Communication using 19200, 8 bits with no parity and 1 stop bit.                         */
/* The same UART ressource do not allow to obtain an emission and reception stream with different   */
/* rate transferts. But for each of them, it will be possible to apply 3 different baud rates.      */
/****************************************************************************************************/
void Init_UART(UART_Port_t UART_selected, uint32_t Baudrate, CharSize_t LengthChar, Parity_t Parity, StopBit_t StopBit) {
  char DisplayedText[6];
  uint8_t k = 0;
  do {
    ScratchComUART_32bits = pgm_read_dword(&BaudRates[k++]);
  } while(ScratchComUART_32bits != Baudrate);
  Delay2Chars = pgm_read_byte(&DelayBetween2Chars[k - 1]);      // useful only for the Polling_RS232 function
  //Serial.print(F("coefficient between 2 chars: ")); Serial.print(Delay2Chars, DEC);
  UBRR_value = (uint16_t)(F_cpu / (16UL * Baudrate)) - 1;
  Serial.print(F("\n[info] UART"));
  Serial.print(UART_selected, DEC);
  ScratchComUART_8bits = ConvUint32ToDecASCIIChar(&TabAsciiUnsignedInteger[0], Baudrate);
  Serial.print(F("\n[info] Baudrate : "));
  for (k = 0; k < ScratchComUART_8bits; k++) Serial.print(TabAsciiUnsignedInteger[k]);
  Serial.print(F(" baud"));
  UBRR_High = (uint8_t)(UBRR_value>>8);     // UBRR_value non modifié
  UBRR_Low = (uint8_t)UBRR_value;           // ne conserve que la partie basse du mot de 16 bits
  Serial.print(F("\n[info] Value of USART Baud Rate Register (UBRRxH) : 0x")); Serial.print(UBRR_High, HEX);
  Serial.print(F("\n[info] Value of USART Baud Rate Register (UBRRxL) : 0x")); Serial.print(UBRR_Low, HEX);
  switch (UART_selected) {
    case UART1:
      UBRR1H = UBRR_High;
      UBRR1L = UBRR_Low;
      switch (LengthChar) {
        case CharSize6bits:
          UCSR1B &= ~(1<<UCSZ12);
          UCSR1C &= ~(1<<UCSZ11);
          UCSR1C |= (1<<UCSZ10);
          break;
        case CharSize7bits:
          UCSR1B &= ~(1<<UCSZ12);
          UCSR1C &= ~(1<<UCSZ10);
          UCSR1C |= (1<<UCSZ11);
          break;
        case CharSize8bits:
          UCSR1B &= ~(1<<UCSZ12);
          UCSR1C |= ((1<<UCSZ11)|(1<<UCSZ10));
          break;
        case CharSize9bits:
          UCSR1B |= (1<<UCSZ12);
          UCSR1C |= ((1<<UCSZ11)|(1<<UCSZ10));
          break;
        default:
          break;
      }
      Serial.print(F("\n[info] Character size : ")); Serial.print(LengthChar, DEC);
      switch (Parity) {                                 // char Parity
        case EvenParity:                                // 'E'
          UCSR1C |= (1<<UPM11);
          UCSR1C &= ~(1<<UPM10);
          break;
        case OddParity:                                 // 'O'
          UCSR1C |= ((1<<UPM11)|(1<<UPM10));
          break;
        case NoneParity:                                // 'N'
          UCSR1C &= ~((1<<UPM11)|(1<<UPM10));
          break;
        default:
          break;
      }
      switch (StopBit) {
        case OneStopBit:
          UCSR1C &= ~(1<<USBS1);
          break;
        case TwoStopBit:
          UCSR1C |= (1<<USBS1);
          break;
        default:
          break;  
      }
      break;
    case UART2:
      UBRR2H = UBRR_High;
      UBRR2L = UBRR_Low;
      switch (LengthChar) {
        case CharSize6bits:
          UCSR2B &= ~(1<<UCSZ22);
          UCSR2C &= ~(1<<UCSZ21);
          UCSR2C |= (1<<UCSZ20);
          break;
        case CharSize7bits:
          UCSR2B &= ~(1<<UCSZ22);
          UCSR2C &= ~(1<<UCSZ20);
          UCSR2C |= (1<<UCSZ21);
          break;
        case CharSize8bits:
          UCSR2B &= ~(1<<UCSZ22);
          UCSR2C |= ((1<<UCSZ21)|(1<<UCSZ20));
          break;
        case CharSize9bits:
          UCSR2B |= (1<<UCSZ22);
          UCSR2C |= ((1<<UCSZ21)|(1<<UCSZ20));
          break;
        default:
          break;
      }
      switch (Parity) {
        case EvenParity:                  // 'E'
          UCSR2C |= (1<<UPM21);
          UCSR2C &= ~(1<<UPM20);
          break;
        case OddParity:                   // 'O'
          UCSR2C |= ((1<<UPM21)|(1<<UPM20));
          break;
        case NoneParity:                  //'N'
          UCSR2C &= ~((1<<UPM21)|(1<<UPM20));
          break;
        default:
          break;
      }
      switch (StopBit) {
        case OneStopBit:
          UCSR2C &= ~(1<<USBS2);
          break;
        case TwoStopBit:
          UCSR2C |= (1<<USBS2);
          break;
        default:
          break;  
      }
      break;
    case UART3:
      UBRR3H = UBRR_High;
      UBRR3L = UBRR_Low;
      switch (LengthChar) {
        case CharSize6bits:
          UCSR3B &= ~(1<<UCSZ32);
          UCSR3C &= ~(1<<UCSZ31);
          UCSR3C |= (1<<UCSZ30);
          break;
        case CharSize7bits:
          UCSR3B &= ~(1<<UCSZ32);
          UCSR3C &= ~(1<<UCSZ30);
          UCSR3C |= (1<<UCSZ31);
          break;
        case CharSize8bits:
          UCSR3B &= ~(1<<UCSZ32);
          UCSR3C |= ((1<<UCSZ31)|(1<<UCSZ30));
          break;
        case CharSize9bits:
          UCSR3B |= (1<<UCSZ32);
          UCSR3C |= ((1<<UCSZ31)|(1<<UCSZ30));
          break;
        default:
          break;
      }
      switch (Parity) {
        case EvenParity:                  // 'E'
          UCSR3C |= (1<<UPM31);
          UCSR3C &= ~(1<<UPM30);
          break;
        case OddParity:                   // 'O'
          UCSR3C |= ((1<<UPM31)|(1<<UPM30));
          break;
        case NoneParity:                  //'N'
          UCSR3C &= ~((1<<UPM31)|(1<<UPM30));
          break;
        default:
          break;
      }
      switch (StopBit) {
        case OneStopBit:
          UCSR3C &= ~(1<<USBS3);
          break;
        case TwoStopBit:
          UCSR3C |= (1<<USBS3);
          break;
        default:
          break;  
      }
      break;
    default:
      break;              
  }
}
/****************************************************************************************************/
/* README file to identify the control commands to configure all the UARTs.                         */
/****************************************************************************************************/
void uarthelp() {
  separateurComUART(110, '*');
  Serial.println(F("REMARQUES IMPORTANTES :"));
  Serial.println(F("- Les nombres flottants ne doivent pas comporter plus de 7 décimales."));
  Serial.println(F("- Les caractères < et > ne doivent pas être écrits, ils précisent un paramètre hexadécimal (x), décimal (d) ou alphanumérique (a)."));
  Serial.println(F("- La représentation ASCII des paramètres alphanumériques peut être en majuscule : hexadécimal (X) et alphanumérique (A)."));
  Serial.println(F("- La forme littérale de la commande est précisée entre apostrophes (') qui ne doivent pas être utilisées."));
  separateurComUART(110, '*');
  Serial.println(F("UARTs configurables : UART1, UART2 & UART3"));
  Serial.println(F("Commande : 'uart'<d_d_dAd> (uart1_6_8N1)"));
  Serial.println(F("Commande ASCII comprenant : numéro d'UART, taux de communication, nbr de bits par caractère, parité et nbr de bits de stop\n"));
  Serial.println(F("\t- Numéro de l'UART choisi :\t1 -> UART1,\t2 -> UART2,\t3 -> UART3"));
  Serial.println(F("\t- Baud Rate :\t0 -> 115200,\t1 -> 57600,\t2 -> 38400,\t3 -> 28800,\t4 -> 19200,\n\t\t\t5 -> 14400,\t6 -> 9600,\t7 -> 4800,\t8 -> 2400,\t9 -> 1200"));
  Serial.println(F("\t- Nombre de bits par caractère :\t6 -> 6 bits,\t7 -> 7 bits,\t8 -> 8 bits,\t9 -> 9 bits"));
  Serial.println(F("\t- Parité :\tE (Even) -> paire,\tO (Odd) -> impaire,\tN (None) -> aucune"));
  Serial.println(F("\t- Stop bits :\t1 -> One stop bit,\t2 -> Two stop bits"));
  separateurComUART(110, '-');
  Serial.println(F("To check features of each UART : 'readuart_'<d>"));
  Serial.println(F("\t- d is the number of the selected UART :\t1 -> UART1,\t2 -> UART2,\t3 -> UART3"));
  separateurComUART(110, '_');
  Serial.println(F("CONFIGURATION des interruptions propres aux UARTs :"));
  Serial.println(F("Control command : 'intuart_'<xxxx>"));             // version anglaise : Interrupts Control
  Serial.println(F("Une interruption activée entraine l'activation de l'émetteur et du récepteur"));
  Serial.println(F("\t<xxxx> => x___ : UART number (1 to 3)\n"));
  Serial.println(F("\t<xxxx> => _x__ : RX Complete Interrupt Enable : RX_ISR_enable (1) or RX_ISR_disable (0)\n"));
  Serial.println(F("\t<xxxx> => __x_ : TX Complete Interrupt Enable : TX_ISR_enable (1) or TX_ISR_disable (0)\n"));
  Serial.println(F("\t<xxxx> => ___x : USART Data Register Empty Interrupt Enable : UDRempty_ISR_enable (1) or UDRempty_ISR_disable (0)"));
  separateurComUART(110, '-');
  Serial.println(F("To check interruptions used on each UART : 'readint_'<d>"));
  Serial.println(F("\t- d is the number of the selected UART :\t1 -> UART1,\t2 -> UART2,\t3 -> UART3"));
  separateurComUART(110, '_');
  Serial.println(F("Exchange of frame on the same UART using TxC and RxC pinouts connected together"));
  Serial.println(F("Control command : 'rxtxtied_'<d>"));
  separateurComUART(110, '_');
  Serial.println(F("Polling the I2C bus"));
  Serial.println(F("\t- To read I2C addresses for all devices connected : 'scan' ou 'i2c'"));
  separateurComUART(110, '=');
}
/****************************************************************************************************/
/* Séparateur de ce module.                                                                         */
/****************************************************************************************************/
void separateurComUART(uint8_t nbr_carac, char caract) {
  uint8_t i;
  for (i = 0; i < nbr_carac; i++) Serial.print(caract);
  Serial.println();
}
/****************************************************************************************************/
/* Fonction pour convertir un nombre hexadécimal en son équivalent ASCII représentant la valeur     */
/* décimale. On transmet comme argument l'adresse d'un tableau au paramètre de la fonction qui est  */
/* un pointeur de type char. Ce tableau doit être commun au module et l'entier ne doit pas dépasser */
/* 2^32 = 4,294,967,296 - 1.                                                                        */
/****************************************************************************************************/
uint8_t ConvUint32ToDecASCIIChar(char *ptrTAB, uint32_t valAconvertir) {  // 10 caractères possibles 0 to 4,294,967,295
  uint8_t k, j;
  char *ptrINIT;
  ptrINIT = ptrTAB;     // affectation d'une adresse
  for (k = 0; k < 10; k++) *(ptrTAB++) = Null;  // initialisation du tableau
  j = 9;                                        // Indice du poids faible en décimal dans le tableau de char : de 0 à 4,294,967,295
  ptrTAB = ptrINIT;                             // On conserve l'adresse du premier élément du tableau
  ptrTAB += j * sizeof(char);                   // position du chiffre de poids le plus faible
  do {
    ResModulo_UART = (uint8_t)(valAconvertir % 0x0A);
    *(ptrTAB--) = (char)(ResModulo_UART + 0x30);     // caractère ASCII à afficher
    j--;
    ResDivision_UART = (valAconvertir - ResModulo_UART) / 0x0A;
    valAconvertir = ResDivision_UART;      // nouvelle valeur dont il faut définir le modulo
  } while(ResDivision_UART > 15);          // on considère que si le résultat est entre 0 et 15, on identifie tous les caractères ASCII à afficher
  if (ResDivision_UART >= 1 && ResDivision_UART <= 9) *ptrTAB = (char)(0x30 + ResDivision_UART);
  if (ResDivision_UART >= 10 && ResDivision_UART <= 15) {
    ScratchComUART_8bits = ResDivision_UART - 0x0A;
    *(ptrTAB--) = (char)(0x30 + ScratchComUART_8bits);
    j--;
    *ptrTAB = (char)0x31;
  }
  ptrTAB = ptrINIT;               // premier emplacement du tableau
  ptrINIT += j * sizeof(char);    // on récupère le dernier emplacement du chiffre de poids fort
  for (k = 0; k < 10 - j; k++) *(ptrTAB++) = *(ptrINIT++);   // permet de récupérer un tableau commençant par l'indice 0
  //for (k = 10 - j; k < Nbr_Car_LCD; k++) *(ptrTAB++) = 0x20;
  *(ptrTAB++) = Null;
  return 10 - j;
}
/************************************************************************************************************/
/* Fonction pour configurer l'UART avec une seule commande. Commande : 'uart'<d_d_dAd>                      */
/* Commande : 'uart'<d_d_dAd> (uart1_6_8N1 => UART1, 9600 bauds, 8 bits, None parity, 1 Stop bit)           */
/* BaudRates[] = {115200UL, 57600UL, 38400UL, 28800UL, 19200UL, 14400UL, 9600UL, 4800UL, 2400UL, 1200UL}    */
/************************************************************************************************************/
void ConfigUART(String Cde_lue) {
  UART_Port_t MyUART;
  CharSize_t MyCharSize;
  Parity_t MyParity;
  StopBit_t MyStopBit;
  uint8_t k;
  Cde_lue.toUpperCase();                                // retrieve only uppercase
  Cde_lue = Cde_lue.substring(4);
  Cde_lue.toCharArray(ComAsciiArray, Cde_lue.length() + 1);
  MyUART = (uint8_t)ComAsciiArray[0] - 0x30;            // uart?_6_8N1
  BaudRate_index = (uint8_t)ComAsciiArray[2] - 0x30;    // uart1_?_8N1
  Delay2Chars = pgm_read_byte(&DelayBetween2Chars[BaudRate_index]);     // useful only for the Polling_RS232 function
  MyCharSize = (uint8_t)ComAsciiArray[4] - 0x30;        // uart1_6_?N1
  ParityChar = ComAsciiArray[5];                        // E (Even) -> paire, O (Odd) -> impaire, N (None) -> aucune
  switch (ParityChar) {
    case 'E':
      MyParity = EvenParity;
      break;
    case 'O':
      MyParity = OddParity;
      break;
    case 'N':
      MyParity = NoneParity;
      break;
  }
  MyStopBit = (uint8_t)ComAsciiArray[6] - 0x30;
  if (MyUART >= 1 && MyUART <= 3) {
    if (BaudRate_index >= 0 && BaudRate_index <= 9) {
      if (MyCharSize >= CharSize6bits && MyCharSize <= CharSize9bits) {   
        if (isAlpha(ParityChar)) {
          if (ParityChar == 'O' || ParityChar == 'N' || ParityChar == 'E') {
            if (MyStopBit >= 1 && MyStopBit <= 2) {
              ScratchComUART_32bits = pgm_read_dword(&BaudRates[BaudRate_index]);
              Init_UART(MyUART, ScratchComUART_32bits, MyCharSize, MyParity, MyStopBit);  // configuration des registres mais pas activation des UARTs
              Serial.print(F("L'UART")); Serial.print(MyUART, DEC); Serial.print(F(" a été programmé avec les paramètres suivants :\n"));
              ScratchComUART_8bits = ConvUint32ToDecASCIIChar(&TabAsciiUnsignedInteger[0], ScratchComUART_32bits);
              Serial.print(F("\t- Baud Rate : "));
              for (k = 0; k < ScratchComUART_8bits; k++) Serial.print(TabAsciiUnsignedInteger[k]);
              Serial.print(F("\n\t- Frame length : ")); Serial.print(MyCharSize, DEC); Serial.println(F( "bits"));
              Serial.print(F("\t- Parity : "));
              if (ParityChar == 'E') Serial.println(F("Even (paire)"));
              else if (ParityChar == 'O') Serial.print(F("Odd (impaire)"));
              else Serial.println(F("None (aucune)"));
              Serial.print(F("\t- STOP bits number : ")); Serial.print(MyStopBit, DEC);
            } else {
              Serial.print(F("Nombre de bits de STOP programmés non conforme\n"));
              Serial.print(F("L'indice doit être 1 ou 2\n"));
              return;
            }
          } else {
            Serial.print(F("Erreur de parité qui ne correspond pas à E (e), O (o) ou N (n)\n"));
            return;
          }
        } else {
          Serial.print(F("Erreur de parité : le caractère programmé n'est pas conforme\n"));
          return;
        }
      } else {
        Serial.print(F("Nombre de bits demandé hors échelle\n"));
        Serial.print(F("L'indice doit être compris entre 6 et 9\n"));
        return;
      }
    } else {
      Serial.print(F("Indice du taux de communication non conforme\n"));
      Serial.print(F("L'indice doit être compris entre 0 et 9\n"));
      return;
    }
  } else {
    Serial.print(F("Problème avec le numéro de l'UART\n"));
    Serial.print(F("UARTs autorisés : 1 -> UART1, 2 -> UART2 et 3 -> UART3\n"));
  }
}
/****************************************************************************************************/
/* Function to read state of one UART among 4. The rate transfert selected must be read reading the */    
/* UBRR register. The baud rate will be defined using formulas on page 203. The inventory can be    */
/* made for only one UART port. The control frame is 'readuart_'<d>                                 */
/* We identify the features of one UART only sending one argument which is his rank.                */
/****************************************************************************************************/
void UARTReading(String Cde_lue) {
  uint16_t LocalUBRR;
  UART_Port_t UART_selected;
  CharSize_t CharacterSize;
  Parity_t ParityMode;
  StopBit_t StopBits;
  uint8_t k;
  Cde_lue = Cde_lue.substring(9);
  Cde_lue.toCharArray(ComAsciiArray, Cde_lue.length() + 1);
  UART_selected = (uint8_t)(ComAsciiArray[0]) - 0x30;
  switch(UART_selected) {
    case UART0:
      UBRR_value = UBRR0;
      break;
    case UART1:
      UBRR_value = UBRR1;
      break;
    case UART2:
      UBRR_value = UBRR2;
      break;
    case UART3:
      UBRR_value = UBRR3;
      break;
  }
  LocalUBRR = (UBRR_value + 1) * 16;          // par défaut U2Xn = 0
  BaudRate = (uint32_t)(F_cpu / LocalUBRR);
  BaudRateCorrected = SearchCloseValue(BaudRate);
  ScratchComUART_8bits = 0;
  switch (UART_selected) {
    case UART0:
      FlagReader = (UCSR0B & (1<<UCSZ02));
      if (FlagReader != 0) ScratchComUART_8bits |= (1<<UCSZ02);      // else involve that CharacterSize stay null
      ScratchComUART_8bits <<= 1;
      FlagReader = (UCSR0C & ((1<<UCSZ01)|(1<<UCSZ00)));
      ScratchComUART_8bits |= FlagReader;
      ScratchComUART_8bits >>= 1;
      break;
    case UART1:
      FlagReader = (UCSR1B & (1<<UCSZ12));
      if (FlagReader != 0) ScratchComUART_8bits |= (1<<UCSZ12);      // else involve that CharacterSize stay null
      ScratchComUART_8bits <<= 1;
      FlagReader = (UCSR1C & ((1<<UCSZ11)|(1<<UCSZ10)));
      ScratchComUART_8bits |= FlagReader;
      ScratchComUART_8bits >>= 1;
      break;
    case UART2:
      FlagReader = (UCSR2B & (1<<UCSZ22));
      if (FlagReader != 0) ScratchComUART_8bits |= (1<<UCSZ22);      // else involve that CharacterSize stay null
      ScratchComUART_8bits <<= 1;
      FlagReader = (UCSR2C & ((1<<UCSZ21)|(1<<UCSZ20)));
      ScratchComUART_8bits |= FlagReader;
      ScratchComUART_8bits >>= 1;
      break;
    case UART3:
      FlagReader = (UCSR3B & (1<<UCSZ32));
      if (FlagReader != 0) ScratchComUART_8bits |= (1<<UCSZ32);      // else involve that CharacterSize stay null
      ScratchComUART_8bits <<= 1;
      FlagReader = (UCSR3C & ((1<<UCSZ31)|(1<<UCSZ30)));
      ScratchComUART_8bits |= FlagReader;
      ScratchComUART_8bits >>= 1;
      break;
  }
  switch (ScratchComUART_8bits) {
    case 1:
      CharacterSize = CharSize6bits;
      break;
    case 2:
      CharacterSize = CharSize7bits;
      break;
    case 3:
      CharacterSize = CharSize8bits;
      break;
    case 7:
      CharacterSize = CharSize9bits;
      break;
  }
  switch (UART_selected) {
    case UART0:
      FlagReader = (UCSR0C & ((1<<UPM01)|(1<<UPM00)));
      break;
    case UART1:
      FlagReader = (UCSR1C & ((1<<UPM11)|(1<<UPM10)));
      break;
    case UART2:
      FlagReader = (UCSR2C & ((1<<UPM21)|(1<<UPM20)));
      break;
    case UART3:
      FlagReader = (UCSR3C & ((1<<UPM31)|(1<<UPM30)));
      break;
  }
  FlagReader >>= 4;
  switch (FlagReader) {
    case 0:
      ParityMode = NoneParity;
      break;
    case 2:
      ParityMode = EvenParity;
      break;
    case 3:
      ParityMode = OddParity;
      break;
  }
  switch (UART_selected) {
    case UART0:
      FlagReader = (UCSR0C & (1<<USBS0));
      break;
    case UART1:
      FlagReader = (UCSR1C & (1<<USBS1));
      break;
    case UART2:
      FlagReader = (UCSR2C & (1<<USBS2));
      break;
    case UART3:
      FlagReader = (UCSR3C & (1<<USBS3));
      break;
  }
  FlagReader >>= 3;
  switch (FlagReader) {
    case 0:
      StopBits = OneStopBit;
      break;
    case 1:
      StopBits = TwoStopBit;
      break;
  }
  Serial.print(F("[info] UART features :"));
  Serial.print(F(" UART")); Serial.println(UART_selected, DEC);
  switch (UART_selected) {
    case UART0:
      FlagReader = (UCSR0B & ((1<<TXEN0)|(1<<RXEN0)));
      break;
    case UART1:
      FlagReader = (UCSR1B & ((1<<TXEN1)|(1<<RXEN1)));
      break;
    case UART2:
      FlagReader = (UCSR2B & ((1<<TXEN2)|(1<<RXEN2)));
      break;
    case UART3:
      FlagReader = (UCSR3B & ((1<<TXEN3)|(1<<RXEN3)));
      break;
  }
  if (FlagReader == 0x18) Serial.println(F("Transmitter and Receiver: Enables"));
  else Serial.println(F("Transmitter and Receiver: Disables"));
  Serial.print(F("Real Baudrate : "));
  ScratchComUART_8bits = ConvUint32ToDecASCIIChar(&TabAsciiUnsignedInteger[0], BaudRate);
  for (k = 0; k < ScratchComUART_8bits; k++) Serial.print(TabAsciiUnsignedInteger[k]);
  Serial.print(F("\nStandard Baudrate : "));
  ScratchComUART_8bits = ConvUint32ToDecASCIIChar(&TabAsciiUnsignedInteger[0], BaudRateCorrected);
  for (k = 0; k < ScratchComUART_8bits; k++) Serial.print(TabAsciiUnsignedInteger[k]);
  Serial.print(F("\nCharacter size : ")); Serial.println(CharacterSize, DEC);
  Serial.print(F("Parity : "));
  switch (ParityMode) {
    case NoneParity:
      Serial.println("None");
      break;
    case EvenParity:
      Serial.println("Even");
      break;
    case OddParity:
      Serial.println("Odd");
      break;                     
  }
  Serial.print(F("Stop Bit(s) : ")); Serial.println(StopBits, DEC);
}
/****************************************************************************************************/
/* Function to identify the baud rate the most compatible with standard values.                     */    
/****************************************************************************************************/
uint32_t SearchCloseValue(uint32_t MyBaudRate) {
  uint8_t k = 0;
  int32_t Difference;
  uint16_t Margin;
  do {
    ScratchComUART_32bits = pgm_read_dword(&BaudRates[k++]);
    Difference = ScratchComUART_32bits - MyBaudRate;
  } while (Difference > 0);
  Margin = (uint16_t)((6 * ScratchComUART_32bits) / 100);
  if (Difference < 0 && (Difference * (-1)) < Margin) MyBaudRate = ScratchComUART_32bits;
  else MyBaudRate = pgm_read_dword(&BaudRates[--k]);
  return MyBaudRate;
}
/****************************************************************************************************************/
/* Function to select interrupt for all UARTs for transmission and reception except UART0 which is reserved by  */
/* the boot program. I do not recommand to use the UDREn flag as transmission interupt.                         */
/* TXCIEn : The TXCn flag bit is set when the entire frame in the Transmit Shift Register has been shifted out  */
/* and there are no new data currently present in the transmit buffer (UDRn).                                   */
/* UDRIEn : The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn is    */
/* one, the buffer is empty, and therefore ready to be written.                                                 */
/* RX_Interrupt ->  This flag bit is set when there are unread data in the receive buffer and cleared when the  */
/*                  receive buffer is empty.                                                                    */
/* TX_Interrupt ->  This flag bit is set when the entire frame in the Transmit Shift Register has been shifted  */
/*                  out and there are no new data currently present in the transmit buffer.                     */
/* UDR_Interrupt -> This flag bit is set when the transmit buffer UDRn is empty and ready to receive new data.  */
/* If the flag UDR_Interrupt is used, we have to inhibit the authorization TXCIEn in the UCSRnB register and    */
/* reciprocally, it will necessary to inhibit UDRIEn if the flag TX_Interrupt is used.                          */
/* ------------------------------------------------------------------------------------------------------------ */
/* Interrupts Control is : 'intuart_'<xxxx> An interruption for one channel is applied to receive and transmit  */
/* UART channel : <xxxx> => x___ : UART number (1 to 3)                                                         */
/* Receive Complete Interrupt Enable : <xxxx> => _x__ :  RX_ISR_enable (1) or RX_ISR_disable (0)                */
/* Transmit Complete Interrupt Enable :  <xxxx> => __x_ :  TX_ISR_enable (1) or TX_ISR_disable (0)              */
/* UDRE Interrupt Enable : <xxxx> => ___x :  UDRempty_ISR_enable (1) or UDRempty_ISR_disable (0)                */
/****************************************************************************************************************/
void UART_Interrupts_config(String Cde_lue) {
  UART_Port_t UART_selected;
  UCSRnB_register_t ChannelInterrupts;
  Cde_lue = Cde_lue.substring(8);                                     // 'intuart_'<xxxx>
  Cde_lue.toCharArray(ComAsciiArray, Cde_lue.length() + 1);
  UART_selected = (uint8_t)ComAsciiArray[0] - 0x30;                   // <xxxx> => x___
  ScratchComUART_8bits = (uint8_t)ComAsciiArray[1] - 0x30;            // <xxxx> => _x__
  if (ScratchComUART_8bits == 1) ChannelInterrupts.RXCIEn_bit = true; 
  else ChannelInterrupts.RXCIEn_bit = false;
  ScratchComUART_8bits = (uint8_t)ComAsciiArray[2] - 0x30;            // <xxxx> => __x_
  if (ScratchComUART_8bits == 1) ChannelInterrupts.TXCIEn_bit = true; 
  else ChannelInterrupts.TXCIEn_bit = false;
  ScratchComUART_8bits = (uint8_t)ComAsciiArray[3] - 0x30;            // <xxxx> => ___x
  if (ScratchComUART_8bits == 1) ChannelInterrupts.UDRIEn_bit = true; 
  else ChannelInterrupts.UDRIEn_bit = false;
  
  switch (UART_selected) {
    case UART1:
      if (ChannelInterrupts.RXCIEn_bit == true) UCSR1B |= (1<<RXCIE1);
      else UCSR1B &= ~(1<<RXCIE1);
      if (ChannelInterrupts.TXCIEn_bit == true) UCSR1B |= (1<<TXCIE1);
      else UCSR1B &= ~(1<<TXCIE1);  
      if (ChannelInterrupts.UDRIEn_bit == true) UCSR1B |= (1<<UDRIE1);
      else UCSR1B &= ~(1<<UDRIE1);  
      break;
    case UART2:
      if (ChannelInterrupts.RXCIEn_bit == true) UCSR2B |= (1<<RXCIE2);
      else UCSR2B &= ~(1<<RXCIE2);
      if (ChannelInterrupts.TXCIEn_bit == true) UCSR2B |= (1<<TXCIE2);
      else UCSR2B &= ~(1<<TXCIE2);  
      if (ChannelInterrupts.UDRIEn_bit == true) UCSR2B |= (1<<UDRIE2);
      else UCSR2B &= ~(1<<UDRIE2);  
      break;
    case UART3:
      if (ChannelInterrupts.RXCIEn_bit == true) UCSR3B |= (1<<RXCIE3);
      else UCSR3B &= ~(1<<RXCIE3);
      if (ChannelInterrupts.TXCIEn_bit == true) UCSR3B |= (1<<TXCIE3);
      else UCSR3B &= ~(1<<TXCIE3);  
      if (ChannelInterrupts.UDRIEn_bit == true) UCSR3B |= (1<<UDRIE3);
      else UCSR3B &= ~(1<<UDRIE3);  
      break;
    default:
      break;
  }
}
/****************************************************************************************************/
/* Function to read state of the UART interrupts registers.                                         */    
/* The control frame is 'readint_'<d>                                                               */
/****************************************************************************************************/
void ReadUARTInterrupts(String Cde_lue) {
  UART_Port_t UART_selected;
  UCSRnB_register_t ChannelInterrupts;
  uint8_t k;
  Cde_lue = Cde_lue.substring(8);
  Cde_lue.toCharArray(ComAsciiArray, Cde_lue.length() + 1);
  UART_selected = (uint8_t)ComAsciiArray[0] - 0x30;
  
  switch (UART_selected) {
    case UART1:
      ScratchComUART_8bits = UCSR1B;
      FlagReader = (ScratchComUART_8bits & (1<<RXCIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] RXCIE1 interrupt activated"));
      else Serial.print(F("\n[setup] RXCIE1 interrupt disabled"));
      FlagReader = (ScratchComUART_8bits & (1<<TXCIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] TXCIE1 interrupt activated"));
      else Serial.print(F("\n[setup] TXCIE1 interrupt disabled"));
      FlagReader = (ScratchComUART_8bits & (1<<UDRIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] UDRIE1 interrupt activated"));
      else Serial.print(F("\n[setup] UDRIE1 interrupt disabled"));
      break;
    case UART2:
      ScratchComUART_8bits = UCSR2B;
      FlagReader = (ScratchComUART_8bits & (1<<RXCIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] RXCIE2 interrupt activated"));
      else Serial.print(F("\n[setup] RXCIE2 interrupt disabled"));
      FlagReader = (ScratchComUART_8bits & (1<<TXCIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] TXCIE2 interrupt activated"));
      else Serial.print(F("\n[setup] TXCIE2 interrupt disabled"));
      FlagReader = (ScratchComUART_8bits & (1<<UDRIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] UDRIE2 interrupt activated"));
      else Serial.print(F("\n[setup] UDRIE2 interrupt disabled"));  
      break;
    case UART3:
      ScratchComUART_8bits = UCSR3B;
      FlagReader = (ScratchComUART_8bits & (1<<RXCIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] RXCIE3 interrupt activated"));
      else Serial.print(F("\n[setup] RXCIE3 interrupt disabled"));
      FlagReader = (ScratchComUART_8bits & (1<<TXCIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] TXCIE3 interrupt activated"));
      else Serial.print(F("\n[setup] TXCIE3 interrupt disabled"));
      FlagReader = (ScratchComUART_8bits & (1<<UDRIEn));
      if (FlagReader != 0) Serial.print(F("\n[setup] UDRIE3 interrupt activated"));
      else Serial.print(F("\n[setup] UDRIE3 interrupt disabled"));  
      break;
    default:
      break;
  }
}
/************************************************************************************************************/
/* Function to check the UART port with connections Txn and Rxn tied together.                              */
/* The aim is to receive an ASCII frame from the terminal with asking and displaying content to send and to */
/* receive. The control command is 'rxtxtied_'<d>                                                           */
/************************************************************************************************************/
void UART_TxRx_tieded(String Cde_lue) {
  UART_Port_t UartNumber;
  uint8_t k;
  uint8_t nbr_ReceivedBytes, *ptr_byte;
  Cde_lue = Cde_lue.substring(9);
  Cde_lue.toCharArray(ComAsciiArray, Cde_lue.length() + 1);
  UartNumber = (uint8_t)ComAsciiArray[0] - 0x30;
  Serial.print(F("[uart] You must tied Rx")); Serial.print(UartNumber, DEC); 
  Serial.print(F(" and Tx")); Serial.println(UartNumber, DEC);
  Serial.print(F("[uart] confirm the connection between Tx")); Serial.print(UartNumber, DEC);
  Serial.print(F(" and Rx")); Serial.print(UartNumber, DEC); Serial.print(F(" (Y/N) ?"));
  do  {                                 // polling on UART0
  } while(Serial.available() == 0);     // We wait the answer of the operator Yes or No
  nbr_ReceivedBytes = Serial.readBytesUntil(CR, BuffOut, 100);
  BuffOut[nbr_ReceivedBytes] = Null;
  ptr_byte = BuffOut;
  Serial.print(F("\n[uart] Answer from terminal: ")); Serial.println((char)*ptr_byte);
  
  if (*ptr_byte == 'Y' || *ptr_byte == 'y') {
    Serial.print(F("[uart] You can write a frame..."));
    do  {                                           // polling on UART0
    } while(Serial.available() == 0); // We wait the ASCII text from operator to fill the Buffer Output to transmit message
    nbr_ReceivedBytes = Serial.readBytesUntil(CR, BuffOut, 100);        // Line feed included and Carriage Return excluded
    BuffOut[nbr_ReceivedBytes - 1] = '\0';          // at the emplacement of LF character (0x0A or 10)
    Serial.print(F("\n[uart] Number of bytes being transmitted: ")); Serial.print(nbr_ReceivedBytes - 1, DEC);
    Serial.print(F("\n[uart] Payload sended:\t\t"));
    k = 0;
    do {
      Serial.print((char)BuffOut[k++]);
    } while(BuffOut[k] != '\0');
    for (k = 0; k < nbr_ReceivedBytes - 1; k++) {   // main loop which sends and receives one char
      Send_OnlyOneChar(BuffOut[k], UartNumber);
      BuffIn[k] = Read_OnlyOneChar(UartNumber); 
    }
    BuffIn[nbr_ReceivedBytes - 1] = '\0';
    k = 0;
    Serial.print(F("\n[uart] Payload received:\t"));
    do {
      Serial.print((char)BuffIn[k++]);
    } while(BuffIn[k] != '\0');                
  } else {
    Serial.print(F("\n[uart] No communication awaited on UART"));
    Serial.print(UartNumber, DEC);
  }
}
/********************************************************************************************/
/* To transmit only one character.                                                          */
/* The priority of the reception interrupt involves that the flag is not cleared as long as */
/* the buffer is not read. So when a character is transmitted using the instruction         */
/* "UDR = ..." the first interruption which is rising is the reception interrupt and as     */
/* long as we don't read the buffer, the other interruption with lower priority are not     */
/* activated. It is important to understand the necessity to deactivate the reception       */
/* interrupt when we send only one char. The high priority of the receive interrupt do not  */
/* allow the rising of the transmission interrupt flag even if we read the content of UDR1  */
/* to remove the receive interruption (ScratchComUART_8bits = UDR1;).                       */
/********************************************************************************************/
void Send_OnlyOneChar(uint8_t bytetransmited, UART_Port_t UartNumber) {
  switch (UartNumber) {
    case UART0:
      break;
    case UART1:
      //ScratchComUART_8bits = UDR1;    // normally strength UDRE1 in order to change to one but non necessary here
      UCSR1B &= ~(1<<RXCIE1);         // disabling the reception interrupt which priority is higher than transmission
      UDR1 = bytetransmited;          // before this line the level of UDRE1 is one
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_TxD1));  // interruption when content of transmit register has been completly shifted out
      } while (TestFlags16bits == 0);
      drapeau_UART &= ~(1<<flag_ISR_TxD1);
      UCSR1B |= (1<<RXCIE1);          // reactivate the reception interrupt
      break;
    case UART2:
      //ScratchComUART_8bits = UDR2;
      UCSR2B &= ~(1<<RXCIE2);
      UDR2 = bytetransmited;
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_TxD2));
      } while (TestFlags16bits == 0);
      drapeau_UART &= ~(1<<flag_ISR_TxD2);
      UCSR2B |= (1<<RXCIE2);
      break;
    case UART3:
      //ScratchComUART_8bits = UDR3;
      UCSR3B &= ~(1<<RXCIE3);
      UDR3 = bytetransmited;
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_TxD3));
      } while (TestFlags16bits == 0);
      drapeau_UART &= ~(1<<flag_ISR_TxD3);
      UCSR3B |= (1<<RXCIE3);
      break;
  }
}
/******************************************************************************************************************/
/* Non-blocking function to read only one char. When we used this function, no array is needed.                   */
/******************************************************************************************************************/
uint8_t Read_OnlyOneChar(UART_Port_t UartNumber) {
  cmpt1 = 0;    // (uint16_t) timer set to 5 ms : to avoid the shutdown of the oxymeter
  switch (UartNumber) {
    case UART0:
      return;
    case UART1:
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_RxD1));
        if (cmpt1 >= 10) return;      // After 50 ms of awaiting delay, we get out from this function 
      } while(TestFlags16bits == 0);
      drapeau_UART &= ~(1<<flag_ISR_RxD1);
      return UDR1;                    // the fact reading the buffer clear automatically the flag RXCn
    case UART2:
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_RxD2));
        if (cmpt1 >= 10) return; 
      } while(TestFlags16bits == 0);
      drapeau_UART &= ~(1<<flag_ISR_RxD2);
      return UDR2;
    case UART3:
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_RxD3));
        if (cmpt1 >= 10) return; 
      } while(TestFlags16bits == 0);
      drapeau_UART &= ~(1<<flag_ISR_RxD3);
      return UDR3;
  }
}
/************************************************************************************************************/
/* Function to identify the version of the instrument via the UART1.                                        */
/* The array which contains the control command is BuffOut[]. The command is 'ver_'<d> (d is the channel).  */
/* https://www.nongnu.org/avr-libc/user-manual/group__avr__pgmspace.html#ga5749897c91c479d02054fc02128de482 */
/* Useful function : char *strcpy_P(char *dest, const char *src)                                            */
/* The length of all commands is 3 characters and it's easy to define the total length of the frame.        */
/************************************************************************************************************/
void ControlCdeVer(String Cde_lue) {
  uint8_t ASCIIChannel;
  uint8_t k;
  uint8_t *ptr;
  memset(BuffIn, Null, sizeof(BuffIn));       // global array
  memset(BuffOut, Null, sizeof(BuffOut));     // global array
  UCSR1B |= (1<<TXCIE1);      // transmit interrupt enable (At this point the receive interruption is not activated)
  
  Cde_lue = Cde_lue.substring(4);
  Cde_lue.toCharArray(ComAsciiArray, Cde_lue.length() + 1);
  ASCIIChannel = (uint8_t)ComAsciiArray[0];
  ptrConstChar = &CdeVersion[0];            // "VER"
  strcpy_P(BuffOut, ptrConstChar);
  k = NbrCharInArray(&BuffOut[0]);          // define the location just after the control command
  BuffOut[k++] = 0x20;
  BuffOut[k++] = ASCIIChannel;
  BuffOut[k++] = CR;
  BuffOut[k++] = '\0';                      // here we get: VER 1<CR>'\0'
  ptr = &BuffOut[0];
  Serial.print(F("\n[uart] control command sent: "));   // no any interaction with reception so have to be made before transmission
  do {
    Serial.print((char)*(ptr++));
  } while(*ptr != '\0');
  Serial.print((char)LF);                   // Normally we get a new line
  Send_Frame(UART1);
  UCSR1B |= (1<<RXCIE1);
  PollingUART_CRend(UART1);               // predictable events and blocking function
  //ScratchComUART_8bits = Polling_RS232(UART1, 200);                // non blocking function and life duration of 200 x 5 ms with no recieved chars
    
  ptr = &BuffIn[0];
  Serial.print(F("\n[uart] answer received: "));
  do {
    Serial.print((char)*(ptr++));
  } while(*ptr != '\0');
  UCSR1B &= ~(1<<RXCIE1);
  UCSR1B &= ~(1<<TXCIE1);
  //Serial.print(F("\n[uart] characters number: ")); Serial.print(ScratchComUART_8bits, DEC);
}
/******************************************************************************************************************/
/* Function to transmit a control command which have been loaded in BuffOut array.                                */
/******************************************************************************************************************/
void Send_Frame(UART_Port_t num_UART) {
  uint8_t *ptrTxD;
  ptrTxD = &BuffOut[0];
  
  switch (num_UART) {
    case UART0:
      return;
    case UART1:
      do {
        UDR1 = *(ptrTxD++);
        do {
          TestFlags16bits = (drapeau_UART & (1<<flag_ISR_TxD1));
        } while(TestFlags16bits == 0);
        drapeau_UART &= ~(1<<flag_ISR_TxD1);
      } while(*ptrTxD != '\0');
      break;
    case UART2:
      do {
        UDR2 = *(ptrTxD++);
        do {
          TestFlags16bits = (drapeau_UART & (1<<flag_ISR_TxD2));
        } while(TestFlags16bits == 0);
        drapeau_UART &= ~(1<<flag_ISR_TxD2);
      } while(*ptrTxD != '\0');
      break;
    case UART3:
      do {
        UDR3 = *(ptrTxD++);
        do {
          TestFlags16bits = (drapeau_UART & (1<<flag_ISR_TxD3));
        } while(TestFlags16bits == 0);
        drapeau_UART &= ~(1<<flag_ISR_TxD3);
      } while(*ptrTxD != '\0');
      break;
  }
}
/************************************************************************************************************/
/* This function is used just after sending a control command. The life duration of this function has to be */
/* limited by a time delay (NbrCWTimex5ms). This variable must be completed by user. An other time delay    */
/* which depends of the baud rate applied and defined by functions Init_UART() and ConfigUART(). This delay */
/* which is a multiple of 100 µs is extract from the array named DelayBetween2Chars. The name of this       */
/* variable is Delay2Chars and is automaticaly defined when we call functions Init_UART() and ConfigUART(). */
/* This function return the number of received characters. If a character is received, the delay between    */
/* two characters is reset and the function will stop whith the last character of the frame as long as the  */
/* delay between two charcacters do not reach the value defined by DelayBetween2Chars. So when this         */
/* function is called, the complete life duration depends of NbrCWTimex5ms multiply by an integer. Between  */
/* two characters, the elapse times according to the different baud rates are:                              */
/* 115200 -> 100 µs, 57600 -> 200 µs, 38400 -> 200 µs, 28800 -> 300 µs, 19200 -> 300 µs, 14400 -> 400 µs    */
/* 9600 -> 400 µs, 4800 -> 500 µs, 2400 -> 500 µs, 1200 -> 600 µs                                           */
/* Normaly this function allows polling frames sent by a device without knowing the response delay and the  */
/* end character of the frame. So this function is adapted to an unpredictable answer from an other device. */
/************************************************************************************************************/
uint8_t Polling_RS232(UART_Port_t NumUart, uint16_t NbrCWTimex5ms) {  // maximum delay if no byte have been received = NbrCWTimex5ms x 5 ms
  cmpt_5ms = 0;                         // This variable will be incremented by timer1 interrupt every 5 ms
  cmpt_100us = 0;                       // defines the time step which is multiply by a coefficient issued from baudrate
  Indice_carac = 0;                     // incremented index which defines the number of characters received
  bool OneAsciiReceived = false;        // At the calling of this function
  uint8_t *ptrRxD;
  ptrRxD = &BuffIn[0];
                                   
  switch(NumUart) {
    case UART1:
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_RxD1));
        if (TestFlags16bits != 0) {     // non blocking
          *(ptrRxD++) = ReceivedChar;
          drapeau_UART &= ~(1<<flag_ISR_RxD1);
          cmpt_100us = 0;               // to reset delay between two ASCII characters received
          cmpt_5ms = 0;
          OneAsciiReceived = true;      // as soon as the first character has been received, this flag is modified
          Indice_carac++;
        } else {
          if (OneAsciiReceived == false) cmpt_100us = 0;
        }
      } while(cmpt_5ms < NbrCWTimex5ms || cmpt_100us < Delay2Chars);  // for a 19200 baudrate, Delay2Chars = 3 that involves 300 µs time delay
      break;
    case UART2:
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_RxD2));
        if (TestFlags16bits != 0) {     // non blocking
          *(ptrRxD++) = ReceivedChar;
          drapeau_UART &= ~(1<<flag_ISR_RxD2);
          cmpt_100us = 0;               // to reset delay between two ASCII characters received
          cmpt_5ms = 0;
          OneAsciiReceived = true;      // as soon as the first character has been received, this flag is modified
          Indice_carac++;
        } else {
          if (OneAsciiReceived == false) cmpt_100us = 0;
        }
      } while(cmpt_5ms < NbrCWTimex5ms || cmpt_100us < Delay2Chars);  // for a 19200 baudrate, Delay2Chars = 3 that involves 300 µs time delay
      break;
    case UART3:
      do {
        TestFlags16bits = (drapeau_UART & (1<<flag_ISR_RxD3));
        if (TestFlags16bits != 0) {     // non blocking
          *(ptrRxD++) = ReceivedChar;
          drapeau_UART &= ~(1<<flag_ISR_RxD3);
          cmpt_100us = 0;               // to reset delay between two ASCII characters received
          cmpt_5ms = 0;
          OneAsciiReceived = true;      // as soon as the first character has been received, this flag is modified
          Indice_carac++;
        } else {
          if (OneAsciiReceived == false) cmpt_100us = 0;
        }
      } while(cmpt_5ms < NbrCWTimex5ms || cmpt_100us < Delay2Chars);  // for a 19200 baudrate, Delay2Chars = 3 that involves 300 µs time delay
      break;
  }
  *ptrRxD = '\0';
  return Indice_carac;    // to inform of the array content which contains only one character
}
/************************************************************************************************************/
/* This receive function is stopped by fetchning a Carriage Return character sent by oxymeter. The array is */
/* terminated by a NULL char ('\0'). This function is absolutely predictable.                               */
/* This function is blocking and needs to receive at least one character to exit from it.                   */
/************************************************************************************************************/
void PollingUART_CRend(UART_Port_t NumUart) {
  uint8_t *ptrRxD;
  ptrRxD = &BuffIn[0];
  
  switch(NumUart) {
    case UART1:
      do {
        while ((drapeau_UART & (1 << flag_ISR_RxD1)) == 0);     // blocking
        *(ptrRxD++) = ReceivedChar;
        drapeau_UART &= ~(1<<flag_ISR_RxD1);
      } while(ReceivedChar != CR);
      break;
    case UART2:
      do {
        while ((drapeau_UART & (1 << flag_ISR_RxD2)) == 0);
        *(ptrRxD++) = ReceivedChar;
        drapeau_UART &= ~(1<<flag_ISR_RxD2);
      } while(ReceivedChar != CR);
      break;
    case UART3:
      do {
        while ((drapeau_UART & (1 << flag_ISR_RxD3)) == 0);
        *(ptrRxD++) = ReceivedChar;
        drapeau_UART &= ~(1<<flag_ISR_RxD3);
      } while(ReceivedChar != CR);
      break;
  }
  *ptrRxD = '\0';
}












/* ######################################################################################## */
/* End of file */
