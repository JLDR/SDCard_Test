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
//#include                  "Fonctions.h"



/*************************** Variables locales utilisées par les procédures d'interruption ***************************/         
volatile uint16_t       drapeau_UART;                   // Utilisé par les procédures d'interruption liées à l'UART

// Variables locales propres au module
uint16_t                Indice_carac;
uint8_t                 Byte_Read, UART_Input;    /* octet utilisé pour la lecture et pour l'écriture d'un seul octet */
uint16_t                UBRR_value;
uint8_t                 UBRR_Low, UBRR_High;
const uint32_t          BaudRates[] PROGMEM = {115200UL, 57600UL, 38400UL, 28800UL, 19200UL, 14400UL, 9600UL, 4800UL, 2400UL, 1200UL};
uint8_t                 UART_number, BaudRate_index, Nbr_bits, StopBits;
uint32_t                BaudRate, BaudRateCorrected;
char                    ParityChar;
char                    TabAsciiUnsignedInteger[20];  // 2^64 = 18 446 744 073 709 551 616
uint8_t                 ScratchComUART_8bits;
uint16_t                ScratchComUART_16bits;
uint32_t                ScratchComUART_32bits;
uint32_t                ResDivision_UART;              // 0 to 4,294,967,295
uint8_t                 ResModulo_UART;
char                    ComAsciiArray[20];
uint8_t                 FlagReader;



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
  uint8_t k;
  UBRR_value = (uint16_t)(F_cpu / (16UL * Baudrate)) - 1;
  Serial.print(F("\n[info] UART"));
  Serial.println(UART_selected, DEC);
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
/* fonction d'aide pour lire le contenu des commandes propres à l'UART.                             */
/****************************************************************************************************/
void uarthelp() {
  separateurComUART(110, '=');
  Serial.println(F("REMARQUES IMPORTANTES :"));
  Serial.println(F("- Les nombres flottants ne doivent pas comporter plus de 7 décimales."));
  Serial.println(F("- Les caractères < et > ne doivent pas être écrits, ils précisent un paramètre hexadécimal (x), décimal (d) ou alphanumérique (a)."));
  Serial.println(F("- La représentation ASCII des paramètres alphanumériques peut être en majuscule : hexadécimal (X) et alphanumérique (A)."));
  Serial.println(F("- La forme littérale de la commande est précisée entre apostrophes (') qui ne doivent pas être utilisées."));
  separateurComUART(110, '_');
  Serial.println(F("UARTs configurables : UART1, UART2 & UART3"));
  Serial.println(F("Commande : 'uart'<d_d_dAd> (uart1_6_8N1)"));
  Serial.println(F("Commande ASCII comprenant : numéro d'UART, taux de communication, nbr de bits par caractère, parité et nbr de bits de stop\n"));
  Serial.println(F("\t- Numéro de l'UART choisi :\t1 -> UART1,\t2 -> UART2,\t3 -> UART3\n"));
  Serial.println(F("\t- Baud Rate :\t0 -> 115200,\t1 -> 57600,\t2 -> 38400,\t3 -> 28800,\t4 -> 19200,\n\t\t\t5 -> 14400,\t6 -> 9600,\t7 -> 4800,\t8 -> 2400,\t9 -> 1200\n"));
  Serial.println(F("\t- Nombre de bits par caractère :\t6 -> 6 bits,\t7 -> 7 bits,\t8 -> 8 bits,\t9 -> 9 bits\n"));
  Serial.println(F("\t- Parité :\tE (Even) -> paire,\tO (Odd) -> impaire,\tN (None) -> aucune\n"));
  Serial.println(F("\t- Stop bits :\t1 -> One stop bit,\t2 -> Two stop bits"));
  separateurComUART(110, '=');
  Serial.println(F("CONFIGURATION des interruptions propres aux UARTs :"));
  Serial.println(F("Commande : 'intuart_'<xxxx>"));             // version anglaise : Interrupts Control
  Serial.println(F("Une interruption activée entraine l'activation de l'émetteur et du récepteur"));
  Serial.println(F("\t<xxxx> => x___ : UART number (1 to 3)\n"));
  Serial.println(F("\t<xxxx> => _x__ : RX Complete Interrupt Enable : RX_ISR_enable (0) or RX_ISR_disable (1)\n"));
  Serial.println(F("\t<xxxx> => __x_ : TX Complete Interrupt Enable : TX_ISR_enable (0) or TX_ISR_disable (1)\n"));
  Serial.println(F("\t<xxxx> => ___x : USART Data Register Empty Interrupt Enable : UDRempty_ISR_enable (0) or UDRempty_ISR_disable (1)"));
  separateurComUART(110, '=');
  Serial.println(F("Scrutation des périphériques I2C"));
  Serial.println(F("- Pour lire les adresses I2C courantes des modules connectés : 'scan' ou 'i2c'"));
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
  Serial.println(F("[info] UART features :"));
  Serial.print(F("UART")); Serial.println(UART_selected, DEC);
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
      Serial.println(NoneParityText);         // "None"
      break;
    case EvenParity:
      Serial.println(EvenParityText);         // "Even"
      break;
    case OddParity:
      Serial.println(OddParityText);         // "Odd"
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
/* Function to select interrupt for all UARTs for transmission and reception ecept UART0 whic is reserved by    */
/* the boot program. We do not recommand to use the UDREn flag as transmission interupt.                        */
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
/* reciprocally, it will necessary to inhibit UDRIEn if the flag TX_Interrupt is used.                          */                                                */
/****************************************************************************************************************/
void UART_Interrupts_config(uint8_t UART_selected, bool TxRx_Interrupt, bool UDR_Interrupt) {
  switch (UART_selected) {
    case UART1:
      if (TxRx_Interrupt == enable_UART) {
        UCSR1B |= ((1<<RXCIE1)|(1<<TXCIE1));                // registre d'autorisation d'interruptions de l'UART
        UCSR1B &= ~(1<<UDRIE1);
        cout_Communication << F("\n[setup] Interruptions RXCIE1 et TXCIE1 activées\n");
      } else UCSR1B &= ~((1<<TXCIE1)|(1<<RXCIE1));
      if (UDR_Interrupt == enable_UART) {
        UCSR1B |= (1<<UDRIE1);
        UCSR1B &= ~(1<<TXCIE1);
        cout_Communication << F("\n[setup] Interruptions RXCIE1 et UDRIE1 activées\n");
      } else UCSR1B &= ~(1<<UDRIE1);
      break;
    case UART2:
      if (TxRx_Interrupt == enable_UART) {
        UCSR2B |= ((1<<RXCIE2)|(1<<TXCIE2));
        UCSR2B &= ~(1<<UDRIE2);
      } else UCSR2B &= ~((1<<TXCIE2)|(1<<RXCIE2));
      if (UDR_Interrupt == enable_UART) {
        UCSR2B |= (1<<UDRIE2);
        UCSR2B &= ~(1<<TXCIE2);
      } else UCSR2B &= ~(1<<UDRIE2);
      break;
    case UART3:
      if (TxRx_Interrupt == enable_UART) {
        UCSR3B |= ((1<<RXCIE3)|(1<<TXCIE3));
        UCSR3B &= ~(1<<UDRIE3);
      } else UCSR3B &= ~((1<<TXCIE3)|(1<<RXCIE3));
      if (UDR_Interrupt == enable_UART) {
        UCSR3B |= (1<<UDRIE3);
        UCSR3B &= ~(1<<TXCIE3);
      } else UCSR3B &= ~(1<<UDRIE3);
      break;
    default:
      break;
  }
}






/* ######################################################################################## */
/* End of file */