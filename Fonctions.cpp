
#include    "Fonctions.h"

// Variables externes
extern uint32_t             EntierLong1, EntierLong2, EntierLong3, EntierLong4;  // version 32 bits
extern const uint32_t       *ptr_lng0, *ptr_lng1;
extern const char           *ptrChar1;
extern bool                 test0, test1, test2, test3;
extern uint8_t              Indice0, Indice1, Indice2, Indice3, Indice4, Indice5, Indice6, Indice7;
extern uint16_t             param0, param1, param2, param3;
extern uint8_t              i, j, k, l, m, n, t;

extern char                 TabConversionInteger[20];     // 2^64 = 18 446 744 073 709 551 616
extern char                 TabASCII[60];
extern char                 Scratch_tab[20];

extern char                 computerdata[60];             // permet de stocker les commandes ASCII provenant du PC ou du terminal
extern char                 cinBuf[40];
extern uint8_t              computer_bytes_received;      // nombre d'octets transmis par le PC ou le terminal
extern uint32_t             scratch_32bits;
extern uint16_t             scratch_16bits;
extern uint8_t              scratch_8bits;
extern uint16_t             lecture_masque;
extern String               Cde_received;
extern uint16_t             Val_dec;
extern uint8_t              adresse_scan;
extern String               Commande;
extern char                 *cmd;                             // pointeur de type char utilisé pour les opérations de parsing
extern uint8_t              add_stamp;
extern uint8_t              var0, var1, var2, var3, var4, var5, var6;
extern int                  test[6];
extern uint8_t              pos0, pos1, pos2, pos3, pos4;
extern uint8_t              Nbr_FolderOnRoot;
extern uint8_t              Nbr_FoldersFromRoot[5];
extern char                 FoldersOnRoot[20][NbCrFldName];   // array[LIGNES][COLONNES], nom des répertoires limité à 8 caractères
extern char                 FoldersOnLevel1[20][NbCrFldName];
extern char                 FoldersOnLevel2[20][NbCrFldName];
extern char                 FoldersOnLevel3[20][NbCrFldName];
extern char                 FoldersOnLevel4[20][NbCrFldName];
extern uint8_t              IndexFolders[5][20];
extern uint8_t              Drapeaux_groupe1;
extern uint8_t              Flags_reading;
extern char                 filepath_array[58];           // comprend le chemin d'accès et le nom du fichier à supprimer
extern char                 filename[13];                 // 8 caractères au maximum pour le nom du fichier et 4 pour l'option .xxx + '\0'
extern char                 foldername[45];               // On prévoit 5 niveaux d'arborescence au maximum soit 8 caractères par répertoire + '/'
extern char                 optionfile[4];
extern uint8_t              Foldername_length;
extern uint8_t              Filename_length;
extern String               FilePathFromRoot;
extern String               FolderPathFromRoot;
extern bool                 Root_level;
extern uint8_t              NbrFolder0, NbrFolder1, NbrFolder2, NbrFolder3, NbrFolder4, NbrFolder5;   // 5 niveaux de strates dans l'arborescence
extern String               NomDuFichier;
extern String               NomDuRepertoire;
extern String               OptionFichier;
extern String               FichierOuvert;
extern uint64_t             volumesize;
extern uint32_t             cardSize;
extern bool                 EtatLecture;

// Variables locales propres au module
uint8_t             result_modulo;      
uint32_t            result_division;        // 0 to 4,294,967,295
uint64_t            result_division64;
volatile uint16_t   cmpt1, cmpt2, cmpt3, cmpt4, cmpt5;
volatile uint8_t    compt1, compt2, compt3, compt4, compt5, cmpt_100us, cmpt_5ms;
uint8_t             test_drapeaux;
char                ConvAsciiToUint16_t[5];      // tableau de '0' à "65535"

// Instanciation d'objets
LiquidCrystal_I2C lcd_New(0x27, 20, 4);                 /* déclaration de l'afficheur appelé lcd */
ArduinoOutStream cout(Serial);                          // Create a Serial output stream (ofstream class)
ArduinoInStream cin(Serial, cinBuf, sizeof(cinBuf));    // Create a serial input stream

/* #########################################Interruptions########################################## */
/****************************************************************************************************/
/* Programmes d'interrution des 3 compteurs. Le compteur0 est réservé par plusieurs bibliothèques.  */
/* Timers utilisés : Timer1 (16 bits, 5 ms), Timer2 (8 bits, 100 µs) et Timer5 (16 bits, 500 ms).   */
/****************************************************************************************************/
// Compteur 1 programmé pour des temporisations de 5 ms
ISR(TIMER1_COMPA_vect) {
  compt1++;                     // 8 bits
  cmpt1++;                      // 16 bits
  cmpt_5ms++;                   // 8 bits
}
// Compteur 2 programmé pour des temporisations de 100 µs
ISR(TIMER2_COMPA_vect) {
  compt2++;                     // 8 bits
  cmpt2++;                      // 16 bits
  cmpt_100us++;                 // 8 bits
}
// Compteur 2 programmé pour des temporisations de 10 ms
ISR(TIMER3_COMPA_vect) {
  compt3++;                     // 8 bits
  cmpt3++;                      // 16 bits
}
// Compteur 4 programmé pour des temporisations de 100 ms
ISR(TIMER4_COMPA_vect) {
  compt4++;                     // 8 bits
  cmpt4++;                      // 16 bits
}
// Compteur 5 programmé pour des temporisations longues de 50 ms
ISR(TIMER5_COMPA_vect) {
  compt5++;
  cmpt5++;
}
/* ####################################Fin des interruptions####################################### */
/****************************************************************************************************/
/* Fonction pour initialiser les 6 timers du microcontrôleur ATmega2560. Le programme principal     */
/* doit charger l'entier Ctrl_Flags pour déterminer les compteurs utilisés. Par convention :        */
/* Timer0 (8 bits), temporisation : 1 ms, mode CTC, prescaler = 64 => OCR0A = 250                   */                                       
/* Timer1 (16 bits), temporisation : 5 ms, mode CTC, prescaler = 8 => OCR1A = 0x2710 (10000)        */
/* Timer2 (8 bits), temporisation : 100 µs, mode CTC, prescaler = 8 => OCR2A = 0xC8 (200)           */
/* Timer3 (16 bits), temporisation : 10 ms, mode CTC, prescaler = 64 => OCR3A = 0x09C4 (2500)       */
/* --- Timer3 will not have to be used while some others ressources use it. ---                     */
/* Timer4 (16 bits), temporisation : 100 ms, mode CTC, prescaler = 64 => OCR4A = 0x61A8 (25000)     */
/* --- Timer4 will not have to be used while some others ressources use it. ---                     */
/* Timer5 (16 bits), temporisation : 50 ms, mode CTC, prescaler = 256 => OCR5A = 0x0C35 (3125)      */
/* Timer0 => used by LiquidCrystal_I2C library. The Timer0 will not have to be configured.          */
/* Above all the clock of the timer0 will not have to be stopped and is applicable only for him.    */
/****************************************************************************************************/
void Init_Timers(uint8_t drapeauxTimers, uint8_t time_Timer0, uint16_t time_Timer1, uint8_t time_Timer2, uint16_t time_Timer3, uint16_t time_Timer4, uint16_t time_Timer5) {
  test_drapeaux = (drapeauxTimers & (1 << Timer0_ON)); // le seul qui ne doit pas être modifié car il est utilisé par la bibliothèque LiquidCrystal_I2C.h
  if (test_drapeaux != 0) {
    /* compteur 0 */                        // ne pas être utilisé avec l'afficheur I2C
    TCCR0A = 0;                             // (Par défaut) WGM00, WGM01, COM0B0, COM0B1, COM0A0, COM0A1 = 0
    TCCR0A |= (1 << WGM01);                 // CTC (Mode 2) => Clear Timer on Compare match
    TCCR0B = 0;                             // WGM02 = 0
    TCCR0B |= ((1 << CS00) | (1 << CS01));  // division par 64 => 250 KHz
    OCR0A = time_Timer0;                    // temporisation de 1 ms avec 250 (0xFA)
    TIMSK0 |= (1 << OCIE0A);                // vecteur d'interruption
    TCNT0 = 0;
  } else {
    //TCCR0B &= ~((1<<CS00)|(1<<CS01)|(1<<CS02));         // ne doit pas être bloqué car les méthodes de la bibliothèque de l'afficheur ne l'activent pas
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer1_ON));    // compteur1 16 bits configuré pour 5 ms
  if (test_drapeaux != 0) {
    /* compteur 1 => WGM10=0, WGM11=0, WGM12=1, WGM13=0 => CTC */
    TCCR1A = 0;                             // WGM10 et WGM11 sont à  0
    TCCR1B = (1 << WGM12);                  // Mode 4 du timer1
    TCCR1B |= (1 << CS11);                  // prescaler = 8 et WGM13 = 0 => 2 MHz
    TIMSK1 |= (1 << OCIE1A);                // autorisation d'interruption par comparaison : désigne un type d'interruption
    TCNT1 = 0;
    scratch_16bits = time_Timer1;
    OCR1AH = (uint8_t)(scratch_16bits >> 8);    // 0x2710 = 10000
    OCR1AL = (uint8_t)time_Timer1;              // tempo = 10000 x 1/(16MHz/8) = 5 ms
  } else {
    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); // blocage du compteur
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer2_ON));   // compteur2 8 bits configuré pour 100 µs
  if (test_drapeaux != 0) {
    // compteur 2 (configuration CTC) compteur 8 bits
    TCCR2A |= (1 << WGM21);   // CTC (Mode 2)
    TCCR2B = 0;               // WGM02 = 0, CS20 = 0, CS21 = 0, CS22 = 0
    TCCR2B |= (1 << CS21);    // division par 8 => 2 MHz
    OCR2A = time_Timer2;      // temporisation de 100 µs => valeur décimale 200 (0xC8)
    TIMSK2 |= (1 << OCIE2A);  // vecteur d'interruption
    TCNT2 = 0;
  } else {
    TCCR2B &= ~((1 << CS20) | (1 << CS21) | (1 << CS22));     // blocage du compteur
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer3_ON));        // compteur3 16 bits configuré pour 10 ms
  if (test_drapeaux != 0) {
    TCCR3B |= (1 << WGM32); // WGMn2 doit être le seul drapeau activé (mode 4 => CTC)
    TCCR3A = 0;
    TCCR3B |= ((1 << CS31) | (1 << CS30));                    // division par 64 => 250 Khz
    scratch_16bits = time_Timer3;                             // 2500 à convertir
    OCR3AH = (uint8_t)(scratch_16bits >> 8);                  // 0x09C4 = 2500
    OCR3AL = (uint8_t)time_Timer3;
    TIMSK3 |= (1 << OCIE3A);                // autorisation d'interruption par comparaison
    TCNT3 = 0;                              // initialisation du compteur non nécessaire
  } else {
    //TCCR3B &= ~((1 << CS30) | (1 << CS31) | (1 << CS32)); // blocage du compteur qui sera activé par d'autres ressources
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer4_ON));      // compteur4 16 bits configuré pour 100 ms
  if (test_drapeaux != 0) {
    TCCR4B |= (1 << WGM42);                                 // WGMn2 doit être le seul drapeau activé (mode 4 => CTC)
    //TCCR4A = 0;
    TCCR4B |= ((1 << CS41) | (1 << CS40));                  // division par 64 => 250 Khz
    scratch_16bits = time_Timer4;                           // 25000 à convertir pour 100 ms
    OCR4AH = (uint8_t)(scratch_16bits >> 8);                // 0x61A8 = 25000
    OCR4AL = (uint8_t)time_Timer4;
    TIMSK4 |= (1 << OCIE4A);                                // autorisation d'interruption par comparaison
    TCNT4 = 0;                                              // initialisation du compteur non nécessaire
  } else {
    //TCCR4B &= ~((1 << CS40) | (1 << CS41) | (1 << CS42)); // blocage du compteur à l'initialisation qui sera utilisé par d'autres ressources
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer5_ON));      // compteur5 16 bits configuré pour 50 ms
  if (test_drapeaux != 0) {
    TCCR5B |= (1 << WGM52);                                 // WGMn2 doit être le seul drapeau activé (mode 4 => CTC)
    //TCCR5A = 0;                                             // par défaut
    TCCR5B |= (1 << CS52);                                  // division par 256 => 62,5 Khz
    scratch_16bits = time_Timer5;                           // 3125 à convertir pour 50 ms
    OCR5AH = (uint8_t)(scratch_16bits >> 8);                // 0x0C35 = 3125
    OCR5AL = (uint8_t)time_Timer5;
    TIMSK5 |= (1 << OCIE5A);                                // autorisation d'interruption par comparaison
    TCNT5 = 0;                                              // initialisation du compteur non nécessaire
  } else {
    TCCR5B &= ~((1 << CS50) | (1 << CS51) | (1 << CS52)); // blocage du compteur
  }
}
/****************************************************************************************************/
/* Fonction pour convertir un nombre hexadécimal en son équivalent ASCII représentant la valeur
 * décimale. une sorte de sprintf mais à partir d'une variable long. la conversion se fait des poids 
 * faibles vers les poids forts.
 * principe : modulo par 0x0A => le résultat doit être retranché à la valeur initiale
 * modulo => valeur entière du dernier caractère de 0 à 9
 * soustraction => on réduit la valeur à convertir du résultat produt par la fonction modulo
 * division => fixe le nouveau niveau pour lequel on applique la fonction modulo
 * exemple :  0x358 % A = 6 => 0x358 - 6 = 0x352 => 0x352 / A = 0x55
 *            0x55 % A = 5 => 0x55 - 5 = 0x50 => 0x50 / A = 0x8
 *            Résultat => 0x358 => '8' '5' '6'
 * Le tableau qui réceptionne les valeurs ASCCI des entiers qui sont déduis de la conversion est de
 * dimension 20 et est déclaré dans le programme principal (une des conditions impératives)
 * TabConversionInteger[20];     // 2^64 = 18 446 744 073 709 551 616
 */
/****************************************************************************************************/
uint8_t ConvertUint32ToASCIIChar(char *ptrTAB, uint32_t valAconvertir) {       // 10 caractères possibles 0 to 4,294,967,295
  char *ptrINIT;
  ptrINIT = ptrTAB;     // affectation d'une adresse
  for (l = 0; l < 20; l++) *(ptrTAB++) = Space;     // espace (0x20) initialisation du tableau
  Indice7 = 9;                                      // Indice du poids faible en décimal dans le tableau de char : de 0 à 4,294,967,295
  ptrTAB = ptrINIT;
  ptrTAB += Indice7 * sizeof(char);                 // position du chiffre de poids le plus faible
  do {
    result_modulo = (uint8_t)(valAconvertir % 0x0A);
    *(ptrTAB--) = (char)(result_modulo + 0x30);     // caractère ASCII à afficher
    Indice7--;
    result_division = (valAconvertir - result_modulo) / 0x0A;
    valAconvertir = result_division;                // nouvelle valeur dont il faut définir le modulo
  } while(result_division > 15);    // on considère que si le résultat est entre 0 et 15, on identifie tous les caractères ASCII à afficher
  if (result_division >= 1 && result_division <= 9) *ptrTAB = (char)(0x30 + result_division);
  if (result_division >= 10 && result_division <= 15) {
    scratch_8bits = result_division - 0x0A;
    *(ptrTAB--) = (char)(0x30 + scratch_8bits);
    Indice7--;
    *ptrTAB = (char)0x31;
  }
  ptrTAB = ptrINIT;     // premier emplacement du tableau
  ptrINIT += Indice7 * sizeof(char);    // on récupère le dernier emplacement du chiffre de poids fort
  for (k = 0; k < 10 - Indice7; k++) *(ptrTAB++) = *(ptrINIT++);   // permet de récupérer un tableau commençant par l'indice 0
  for (k = 10 - Indice7; k < Nbr_Car_LCD; k++) *(ptrTAB++) = 0x20;
  return 10 - Indice7;
}
/****************************************************************************************************/
/* fonction de conversion d'un entier très long de type uint64_t vers une tableau de caractères     */
/* ASCII. On transmet à la fonction un pointeur qui cible le tableau de caractères                  */
/* TabConversionInteger[20] limité à 20 valeurs pour représenter le plus grand entier avec 64 bits. */
/* La fonction renvoie le nombre de caractères ASCII disponibles pour représenter l'entier à        */
/* convertir.                                                                                       */
/****************************************************************************************************/
uint8_t ConvertUint64ToASCIIChar(char *ptrTAB64, uint64_t valAconvertir) {
  char *ptrINIT64;
  ptrINIT64 = ptrTAB64;                                 // affectation d'une adresse
  for (l = 0; l < 20; l++) *(ptrTAB64++) = 0x20;        // espace
  Indice7 = 19; // valeur du poids faible en décimal dans le tableau de char : de 0 à 18 446 744 073 709 551 615
  ptrTAB64 = ptrINIT64;
  ptrTAB64 += Indice7 * sizeof(char);                   // position du chiffre de poids le plus faible
  do {
    result_modulo = (uint8_t)(valAconvertir % 0x0A);    // premier caractère = poids faible du nombre décimal
    *(ptrTAB64--) = (char)(result_modulo + 0x30);       // stockage dans le tableau ASCII applicable uniquement pour les fonctions modulo
    Indice7--;
    result_division64 = (valAconvertir - result_modulo) / 0x0A;     // quand le résultat de cette division est inférieur à 0x0A, il y a arrêt de l'itération
    valAconvertir = result_division64;                              // result_division64 est automatiquement un nombre entier soit un multiple de 0x0A
  } while(result_division64 > 15);          // on considère que si le résultat est entre 0 et 15, on identifie tous les caractères ASCII à afficher
  if (result_division64 >= 1 && result_division64 <= 9) *ptrTAB64 = (char)(0x30 + result_division64);
  if (result_division64 >= 10 && result_division64 <= 15) {
    scratch_8bits = result_division64 - 0x0A;
    *(ptrTAB64--) = (char)(0x30 + scratch_8bits);
    Indice7--;
    *ptrTAB64 = (char)0x31;
  }
  // recallage des valeurs de fond de tableau vers le début du tableau qui commence avec l'indice 0
  ptrTAB64 = ptrINIT64;                     // premier emplacement du tableau
  ptrINIT64 += Indice7 * sizeof(char);      // on récupère le dernier emplacement du chiffre de poids fort
  for (k = 0; k < 20 - Indice7; k++) *(ptrTAB64++) = *(ptrINIT64++);    // permet de récupérer un tableau commençant par l'indice 0
  for (k = 20 - Indice7; k < 20; k++) *(ptrTAB64++) = 0x20;
  return 20 - Indice7;
}
/****************************************************************************************************/
/* Fonction optimisée pour la conversion d'un entier 32 bits vers une chaîne de caractères ASCII.   */
/* La fonction renvoie le nombre de caractères ASCII qui pourront être affichés.                    */ 
/****************************************************************************************************/
uint8_t ConvertUint32ToASCIICharBis(char *ptrTAB, uint32_t valAconvertir) {       // 10 caractères possibles 0 to 4,294,967,295
  char *ptrINIT;
  ptrINIT = ptrTAB;     // affectation d'une adresse
  for (l = 0; l < 20; l++) *(ptrTAB++) = Space;     // espace (0x20) initialisation du tableau
  Indice7 = 9;                                      // valeur du poids faible en décimal dans le tableau de char : de 0 à 4,294,967,295
  ptrTAB = ptrINIT;
  ptrTAB += Indice7 * sizeof(char);                 // position du chiffre de poids le plus faible
  do {
    result_modulo = (uint8_t)(valAconvertir % 0x0A);
    *(ptrTAB--) = (char)(result_modulo + 0x30);     // caractère ASCII à afficher
    Indice7--;
    result_division = (valAconvertir - result_modulo) / 0x0A;
    valAconvertir = result_division;                // nouvelle valeur dont il faut définir le modulo
  } while(result_division >= 10);                   // on considère que si le résultat est entre 0 et 10, on identifie tous les caractères ASCII à afficher
  if (result_division >= 1 && result_division <= 9) *ptrTAB = (char)(0x30 + result_division);
  ptrTAB = ptrINIT;     // premier emplacement du tableau
  ptrINIT += Indice7 * sizeof(char);    // on récupère le dernier emplacement du chiffre de poids fort
  for (k = 0; k < 10 - Indice7; k++) *(ptrTAB++) = *(ptrINIT++);   // permet de récupérer un tableau commençant par l'indice 0
  for (k = 10 - Indice7; k < Nbr_Car_LCD; k++) *(ptrTAB++) = 0x20;
  return 10 - Indice7;
}
/****************************************************************************************************/
void Affich_Cmd(char *ptrCmd, uint8_t nbrCar) {
  lcd_New.setCursor(0,3);
  for (j = 0; j < Nbr_Car_LCD; j++) lcd_New.write(Space);   // effacement de la ligne de l'afficheur LCD
  lcd_New.setCursor(0,3);
  cout << F("Commande :");
  for (j = 0; j < nbrCar; j++) {
    lcd_New.write(*ptrCmd);
    cout << *(ptrCmd++);
  }
  cout << endl;
}
/****************************************************************************************************/
/* sprintf ne fonctionne pas avec Arduino. Il faut utiliser char * dtostrf(double __val (entier),   */ 
/* signed char __width, unsigned char __prec, char * __s)                                           */
/* str = String(num); str.toCharArray(cstr,16);                                                     */
/****************************************************************************************************/
void Affich_val_dec(uint16_t valeur_dec, uint8_t colonne, uint8_t ligne) {
  lcd_New.setCursor(colonne, ligne);
  lcd_New.print("DEC = ");
  cout << F("DEC = ");
  sprintf(TabASCII, "%5u", valeur_dec);         // "%5d" => représente toujours des nombres signés donc strictement négatifs
  for (l = 0; l < 5; l++) {
    lcd_New.write(TabASCII[l]);
    cout << TabASCII[l];
  }
  cout << endl;
}
/****************************************************************************************************/
/* fonction qui détecte le nombre entier après une chaine de caractère qui est transmise.           */
/* on transmet l'ensemble de la commande sous forme d'un tableau et Cde contient le nom de la       */
/* commande sous forme de caractères ASCII.                                                         */ 
/****************************************************************************************************/
uint16_t detect_entier(char *ptr_mess, String Cde) {
  for (m = 0; m < Nbr_Car_LCD; m++) TabASCII[m] = Space;   /* on initialise le tableau de 20 caractères avec la valeur 0x20 */
  ptr_mess += Cde.length() * sizeof(char); // pour se situer après dec (dimension du pointeur et non le pointeur lui même)
  //lcd_New.setCursor(0,0);
  //lcd_New.write(*ptr_cmd);
  t = 0;
  do {
    TabASCII[t++] = *(ptr_mess++);    // on récupère la partie après la commande identifiée
  } while(*ptr_mess >= 0x30 && *ptr_mess <= 0x39);
  //return atoi(TabASCII);    // atoi() ne fonctionne pas avec arduino
  return ConvASCIItoInt16(TabASCII);
}
/****************************************************************************************************/
uint16_t Lecture_DEC(char *ptrString, uint8_t Long) {
  for (m = 0; m < Nbr_Car_LCD; m++) TabASCII[m] = Space;   /* on initialise le tableau de 20 caractères avec la valeur 0 */
  ptrString += 3 * sizeof(ptrString);     // pour se situer après dec
  if (*ptrString != Space) {
    for (m = 0; m < Long; m++) TabASCII[m] = *(ptrString++);
    return atoi(TabASCII);      // ne fonctionne pas avce Arduino
  } else return 0;
}
/****************************************************************************************************/
/* On convertit un tableau de char représentant une chaîne décimale en un entier 16 bits.           */
/****************************************************************************************************/
uint16_t ConvASCIItoInt16(char *ptr_tabl1) {
  uint32_t entier_verif;
  uint16_t result_local;
  char *ptr_local;
  ptr_local = ptr_tabl1;
  n = 0;      // doit permettre d'identifier les caractères ASCII interprétables
  do {
    if ((uint8_t)(*ptr_tabl1) < 0x30 || (uint8_t)(*ptr_tabl1) > 0x39) break;    // rupture de la boucle do while
    else {
      cout << *ptr_tabl1;
      ptr_tabl1++;
      n++;    // nombre de caractères interprétables
    }
  } while(1);
  if (n != 0) {
    ptr_local += (n - 1) * sizeof(char);  // on fixe la position de l'unité dans le tableau
    entier_verif = 0UL;
    //lcd_New.setCursor(0,2);
    for (i = 0; i < n; i++) {
      switch(i) {
        case 0:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30));
          break;
        case 1:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 10
          break;
        case 2:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 100
          break;
        case 3:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 1000
          break;
        case 4:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 10000
          break;
        case 5:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 100000
          break;
        default:
          break;
      }
      //entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i)); // nombre d'itérations (ne fonctionne pas et correspond à une vraie énigme)
      //Serial.println(entier_verif);
    }
  }
  if (entier_verif >= 65536) return 0;
  else return (uint16_t)entier_verif;
}
/****************************************************************************************************/
uint8_t Recherche_addI2C(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission();   // If true, endTransmission() sends a stop message after transmission, releasing the I2C bus
}
/****************************************************************************************************/
/* fonction de scrutation du bus I2C pour inventorier l'ensemble des périphériques I2C.             */
/* Le balayage des adresses se fait de 1 à 127 avec un affichage de        */
/*  mémoire de programme. En général la réponse sera inférieure à 256 caractères.                   */
/****************************************************************************************************/
void scani2c() {                                            // Scan for all I2C devices
  Serial.println(F("Starting  I2C scan..."));
  var0 = 0;
  for (add_stamp = 1; add_stamp < 128; add_stamp++) {       // toutes les adresses sont scrutées de 1 à 127
    Wire.beginTransmission(add_stamp);
    if (Wire.endTransmission() == 0) {           // si le périphérique répond présent
      var0++;                                         // informe sur le nombre de périphériques I2C présents sur le bus
      Serial.println(F("------------------"));              // simple ligne de texte
      Serial.print(F("I2C CHANNEL 0x"));                    // store string in flash memory
      Serial.println(add_stamp, HEX);                       // affichage de l'adresse du périphérique
    }
  }
  Serial.println(F("\r\r"));
  Serial.println(F("SCAN COMPLETE"));
  Serial.print(F("nombre de périphériques I2C trouvés : "));
  Serial.println(var0, DEC);
}
/* **************************************************************************************************************************** */
void help() {
  cout << F("=====================================================================================================================") << endl;
  cout << F("- Les caractères < et > ne doivent pas être envoyés, ils précisent le paramètre qui sera lu par la fonction") << endl;
  cout << F("- La commande est précisée entre apostrophe (qui ne doit pas être utilisée) avec le symbole : '") << endl;
  cout << F(".....................................................................................................................") << endl;
  cout << F("REMARQUES IMPORTANTES (SDCard) : \n\tLe nom des répertoires doit être limité à 12 caractères sans utiliser les caractères accentués.\n");
  cout << F("\tLe nom des fichiers doit être limité à 8 caractères sans utiliser les caractères accentués.\n");
  cout << F("\tLes caractères transmis (commande ou chaîne ASCII écrite dans un fichier) ne doivent pas être accentués.\n\n");
  cout << F("COMMANDES :\n");
  cout << F("\t- Pour éditer les caractéristiques de la mémoire micro SDCard utilisée : 'checkSD'") << endl;
  cout << F("\t- Pour vérifier qu'un fichier a déjà été créé en précisant le nom et l'option : 'checkFile_'<NomDuFichier.option>") << endl;
  cout << F("\t- Pour lire le contenu d'un fichier en précisant l'option et son chemin : 'cat_'<Répertoire/NomDuFichier.option>") << endl;
  cout << F("\t- Pour créer un fichier dont on précisera le nom et l'option : 'mkfile_'<NomDuFichier.option>") << endl;
  cout << F("\t\t\t * Remarque : la commande ci-dessus détruit le contenu du fichier s'il existait précédemment\n");
  cout << F("\t\t\t            : la création d'un fichier ne peut pas dépasser 12 caractères avec le format suivant : XXXXXXXX.xxx\n");
  cout << F("\t- Pour identifier le type de carte SDCard et son formatage : 'check'") << endl;
  cout << F("\t- Pour scanner le bus I2C en identifiant les adresses utilisées : 'scan'") << endl;
  cout << F("\t- Pour convertir un entier 16 bits avec l'opérateur unaire ~ : 'tilde_0x'<XXXX>") << endl;
  cout << F("\t- Pour lister le contenu de la SDCard : 'dir' ou 'sd' (à éviter)") << endl;
  cout << F("\t- Pour supprimer un fichier à partir de la racine quand le répertoire est connu : 'del_'<Répertoire>") << endl;
  cout << F("\t- Pour créer un répertoire (make) avec un chemin identifié : 'mkdir_'<Répertoire>") << endl;
  cout << F("\t- Pour supprimer un répertoire (remove) avec un chemin identifié : 'rmdir_'<Répertoire>") << endl;
  cout << F("\t- Pour changer le répertoire courant depuis la racine : 'chdir_'<Répertoire>") << endl;
  cout << F("\t\t\t * Remarques : <Répertoire> est de la forme : REP/REP/REP\n");
  cout << F("\t\t\t             : On ne peut supprimer qu'un niveau d'arborescence à la fois\n");
  cout << F("\t- Pour vérifier si le chemin mentionné est un répertoire ou pas : 'chkdir_'<Répertoire>") << endl;

  cout << F("\t- Conversion d'une chaîne HexASCII en une valeur décimale :") << endl;
  cout << F("\t\t- Pour convertir une valeur hexadécimale 32 bits en son équivalent décimal : 'conv_'<XXXXXXXX>") << endl;
  cout << F("\t\t- Pour convertir une valeur hexadécimale 64 bits en son équivalent décimal : 'conv_'<XXXXXXXXXXXXXXXX>")<< endl;
  cout << F("=====================================================================================================================") << endl;
}
/* **************************************************************************************************************************** */
/* conversion d'un nombre hexadécimal et affichage sous forme binaire avec la commande tilde_0x'<XXXX>                          */
/* **************************************************************************************************************************** */
void operateur_unaire(String Cde_received) {
  lcd_New.setCursor(0,2);
  for (m = 0; m < Nbr_Car_LCD; m++) lcd_New.write(Space);
  lcd_New.setCursor(0,3);
  for (m = 0; m < Nbr_Car_LCD; m++) lcd_New.write(Space);
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  ConvAsciiToUint16_t[0] = TabASCII[8];
  ConvAsciiToUint16_t[1] = TabASCII[9];
  ConvAsciiToUint16_t[2] = TabASCII[10];
  ConvAsciiToUint16_t[3] = TabASCII[11];
  param2 = Convert_HexASCII_to_uint16(&ConvAsciiToUint16_t[0]);
  lcd_New.setCursor(0,2);
  lecture_masque = Masque;          // #define Masque 0x8000
  for (k = 0; k < 16; k++) {
    scratch_16bits = (param2 & lecture_masque);
    if (scratch_16bits == 0) {
      lcd_New.write(0x30);
      cout << "0";
    } else {
      lcd_New.write(0x31);
      cout << "1";
    }
    lecture_masque >>= 1;
  }
  cout << endl;
  param1 = ~param2;
  lcd_New.setCursor(0,3);
  lecture_masque = Masque;
  for (k = 0; k < 16; k++) {
    scratch_16bits = (param1 & lecture_masque);
    if (scratch_16bits == 0) {
      lcd_New.write(0x30);
      cout << "0";
    } else {
      lcd_New.write(0x31);
      cout << "1";
    }
    lecture_masque >>= 1;
  }
  cout << endl;
  sprintf(&TabASCII[0], "%04X", param2);
  cout << F("Valeur 16 bits lue : ") << TabASCII;
  //for (i = 0; i < 4; i++) Serial.print(TabASCII[i]);
}
/****************************************************************************************************/
/* Conversion d'une chaîne ASCII de 4 caractères hexadécimaux au maximum pour restituer un entier   */
/* permettant un calcul numérique (nombre uint16_t). Obtenir un entier de 0 à 65535. On transmet un */
/* tableau de caractères hexASCII pour lequel on vérifiera la pertinence des caractères transmis.   */
/* Dans l'ordre, les caractères de poids fort sont avec l'indice le plus faible dans la tableau.    */
/****************************************************************************************************/
uint16_t Convert_HexASCII_to_uint16(char *ptr_chaine_hexascii) {
  char carac1;
  scratch_16bits = 0;
  for (k = 0; k < 4; k++) {
    carac1 = toupper(*(ptr_chaine_hexascii++));
    if (isxdigit(carac1)) {                   // fonction vérifiée fonctionnelle
      //Serial.print(carac1);
      if (carac1 > '9') scratch_8bits = ((uint8_t)carac1) - 7;  // => qui permet de passer de 0x41 à 0x3A
      else scratch_8bits = (uint8_t)carac1;
      scratch_8bits -= 0x30;                                    // on récupère la valeur décimale                                       
      scratch_16bits |= (uint16_t)scratch_8bits;
      if (k < 3) scratch_16bits <<= 4;                          // décalage de 4 emplacements vers la gauche à chaque itération de la boucle
    } else return 0;                                  
  }
  //Serial.println(scratch_16bits);
  return scratch_16bits;
}
/****************************************************************************************************/
/* Conversion d'une chaîne ASCII de 8 caractères hexadécimaux au maximum pour restituer un entier   */
/* permettant un calcul numérique (nombre uint32_t). Obtenir un entier de 0 à 4 294 967 295. On     */
/* transmet un tableau de caractères hexASCII pour lequel on vérifiera la pertinence des caractères */
/* transmis. Dans l'ordre, les caractères de poids fort sont identifiés avec l'indice le plus       */
/* faible dans le tableau qui est envoyé comme argument.                                            */
/****************************************************************************************************/
uint32_t Convert_HexASCII_to_uint32(char *ptr_chaine_hexascii) {
  char carac1;
  scratch_32bits = 0;
  k = 0;
  do {
    carac1 = toupper(*(ptr_chaine_hexascii++));
    if (isxdigit(carac1)) {                                     // fonction vérifiée fonctionnelle
      if ((uint8_t)carac1 > 0x39) scratch_8bits = ((uint8_t)carac1) - 7;  // => qui permet de passer de 0x41 à 0x3A
      else scratch_8bits = (uint8_t)carac1;
      scratch_8bits -= 0x30;                                    // on récupère la valeur décimale                                       
      scratch_32bits |= (uint32_t)scratch_8bits;
      if (*ptr_chaine_hexascii != '\0') scratch_32bits <<= 4;   // décalage de 4 emplacements vers la gauche à chaque itération de la boucle 
    } else return 0;                                  
  } while(*ptr_chaine_hexascii != '\0');
  return scratch_32bits;
}
/****************************************************************************************************/
/* fonction pour vérifier la présence du fichier qui est mentionné dans la commande.                */
/* le nom du fichier et l'option seront précisés et séparés par un point.                           */
/* La commande transmise est de la forme : 'checkFile_'<NomDuFichier.option>. Cette fonction tient  */
/* compte d'un objet SdFat qui a été créé à l'initialisation de la carte et qui a été passé comme   */
/* paramètre formel.                                                                                */
/****************************************************************************************************/
void Check_File(String Cde_received, SdFat Card1, ArduinoOutStream cout, SdFile FichierAVerifier) {
  pos1 = Cde_received.indexOf('_');                         // https://www.arduino.cc/reference/en/language/variables/data-types/string/functions/lastindexof/
  pos0 = Cde_received.indexOf('.');                         // le premier correspond au début de l'option
  NomDuFichier = Cde_received.substring(pos1 + 1, pos0);    // pos0 exclu
  cout << F("Nom du fichier à vérifier : ") << NomDuFichier << endl;          // NomDuFichier = String                            
  pos2 = Cde_received.length();
  OptionFichier = Cde_received.substring(pos0 + 1, pos2);   // OptionFichier = String
  cout << F("Option du fichier à vérifier : ") << OptionFichier << endl;
  FichierOuvert = Cde_received.substring(pos1 + 1, pos2);   // nom du fichier + option (FichierOuvert = String)
  FichierOuvert.toCharArray(filepath_array, FichierOuvert.length() + 1);      // filepath_array[58]
  // open the file. note that only one file can be open at a time, so you have to close this one before opening another.
  if (Card1.exists(filepath_array)) {
    cout << F("Le fichier ") << FichierOuvert << F(" existe...") << endl;       // cout = Serial
  } else {
    cout << F("Le fichier ") << FichierOuvert << F(" n'existait pas mais il a été créé") << endl;
    FichierAVerifier.open(filepath_array, O_CREAT | O_APPEND | O_WRITE);        // http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.LibrairieSdFatLibSdFileOpen
    FichierAVerifier.close();
  }
}
/****************************************************************************************************/
/* Pour créer un fichier dont on aura extrait le nom et l'option. La commande interceptée est de la */
/* forme : 'mkfile_'<NomDuFichier.option>. L'utilisation de cette fonction remplace le fichier      */
/* précédent par un fichier vide si celui ci existait déjà. La longueur est limité à 12 caractères  */
/* ASCII avec 8 pour le nom du fichier et 3 pour l'option.                                          */
/* filepath_array[58] comprend le chemin d'accès et le nom du fichier                               */
/****************************************************************************************************/
void CreationFichier(String Cde_received, SdFat Card1, SdFile fichier1) {
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);                        // fin de tableau avec '\0'
  for (n = 0; n < Cde_received.length() - 7; n++) filepath_array[n] = TabASCII[n + 7];  // après mkfile_
  filepath_array[Cde_received.length() - 7] = '\0';                           // filepath_array[58]
  cmd = &filepath_array[0];
  FilePathFromRoot = (String)cmd;                                             // FilePathFromRoot.reserve(58)
  test[0] = FilePathFromRoot.indexOf('.');                                    // l'index débute toujours par 0
  if (test[0] > 0) {
    NomDuFichier = FilePathFromRoot.substring(0, test[0]);                    // NomDuFichier.reserve(16)
    OptionFichier = FilePathFromRoot.substring(test[0] + 1);                  // OptionFichier.reserve(5)
  }
  Card1.vwd()->getName(filename, 13);                                         // fichier1.openNext(Card1.vwd(), O_READ))
  cout << F("Le répertoire courant est : ") << filename << endl;
  NomDuFichier.toCharArray(filename, 13);                                     // filename[13] prévoir toujours un caractère supplémentaire
  cout << F("Nom du fichier lu : ") << filename << endl;
  OptionFichier.toCharArray(optionfile, 4);
  cout << F("Option du fichier lu : ") << optionfile << endl;
  filename[test[0]] = '.';
  for (k = 0; k < 3; k++) filename[test[0] + k + 1] = optionfile[k];          // nécessaire pour l'ouverture
  filename[test[0] + 4] = '\0';                                               // test[0] + k + 1 avec k = 3
  test2 = fichier1.open(&filename[0], O_CREAT | O_APPEND | O_WRITE);          // http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.LibrairieSdFatLibSdFileOpen
  if (test2 == true) cout << F("Le fichier : ") << filename << F(" a été créé.") << endl;
  else cout << F("Le fichier : ") << filename << F(" n'a pas pu être créé.") << endl;
  //FichierAVerifier = MySD.open(FichierOuvert, FILE_WRITE);
  fichier1.close();
}
/****************************************************************************************************/
/* Pour connaître les propriétés de la micro SDCard. On lit son type et le formatage qui lui est    */
/* appliqué. La dernière instruction liste les propriétés des fichiers stockés dans la mémoire.     */
/* SD2Card = card1, SdVolume = volume1 (taille en octets), SdFile = fichier1.                       */
/* Définition : https://github.com/greiman/SdFat-beta/blob/master/SdFat/examples/SdInfo/SdInfo.ino  */
/* librairie : https://www.arduinolibraries.info/libraries/sd-fat                                   */
/****************************************************************************************************/
void CheckTypeSDCard(SdFat Card1, SdFile fichier1) {             // (SdFat Card1, ArduinoOutStream cout, SdFile fichier1)
  //while (!Card1.init(SPI_HALF_SPEED, SD_CHIP_SELECT)) {   // change to SPI_FULL_SPEED for more performance.
  while (!Card1.begin(SD_CHIP_SELECT, SD_SCK_MHZ(50))) {
    cout << F("Initialisation impossible. Il faut vérifier les 3 recommandations ci-dessous :") << endl;
    cout << F("\t\t - Présence de la SDCard ?") << endl;
    cout << F("\t\t - Vérifier le câblage : MOSI => 11 ou 51, MISO => 12 ou 50, CLK => 13 ou 52, CS => 4 ou 53") << endl;
    cout << F("\t\t - Vérifier la déclaration de la valeur du Chip Select dans le programme") << endl;
    Card1.initErrorHalt();
  }
  cout << F("........................................................................") << endl;
  cout << F("** Identification du type de carte utilisée **") << endl;
  cout << F("Card type : ");
  cardSize = Card1.card()->cardSize();  // http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.LibrairieSdFatLibSd2CardcardSize
  switch(Card1.card()->type()) {
    case SD_CARD_TYPE_SD1:
      cout << F("SD1\n");
      break;
    case SD_CARD_TYPE_SD2:
      cout << F("SD2\n");
      break;
    case SD_CARD_TYPE_SDHC:           // https://github.com/greiman/SdFat-beta/blob/master/SdFat/examples/SdInfo/SdInfo.ino
      if (cardSize < 70000000) cout << F("SDHC\n");   // taille totale divisée par 512
      else cout << F("SDXC\n");
      break;
    default:
      cout << F("Unknown\t\t=> peut être une carte de type SDXC\n");
  }
  cout << F("........................................................................") << endl;
  cout << F("** Identification du formatge appliqué **") << endl;         // print the type and size of the first FAT-type volume
  scratch_32bits = Card1.vol()->fatType();
  if (scratch_32bits == 0) {                                              // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    cout << F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card") << endl;
    return;
  } else {
    cout << F("Volume type is FAT") << scratch_32bits << endl;    // par défaut c'est en décimal https://msdn.microsoft.com/en-us/library/hh438469.aspx
    cout << F("........................................................................") << endl;
    volumesize = Card1.vol()->blocksPerCluster();       // clusters are collections of blocks
    volumesize *= Card1.vol()->clusterCount();          // we'll have a lot of clusters
    volumesize *= 512;                                  // SD card blocks are always 512 bytes
    cout << F("Volume size (bytes): ");
    var4 = ConvertUint64ToASCIIChar(TabConversionInteger, volumesize);
    for (i = 0; i < var4; i++) cout << TabConversionInteger[i];
    cout << endl << F("Volume size (Kbytes): ");
    volumesize /= 1024;
    var4 = ConvertUint32ToASCIIChar(TabConversionInteger, volumesize);
    for (i = 0; i < var4; i++) cout << TabConversionInteger[i];
    cout << F("\nVolume size (Mbytes): ");
    volumesize /= 1024;
    var4 = ConvertUint32ToASCIIChar(TabConversionInteger, volumesize);
    for (i = 0; i < var4; i++) cout << TabConversionInteger[i];
    cout << F("\n........................................................................") << endl;
    cout << F("Files found on the card (name, date and size in bytes) : ") << endl;                   // cout << F("\nList of files on SDCard :") << endl;
    // list all files in the card with date and size http://chipkit.net/forum/viewtopic.php?t=1470
    Card1.ls("/", LS_R | LS_SIZE);           // void SdFile::ls(uint8_t flags, uint8_t indent) Card1.ls("/", LS_R | LS_DATE | LS_SIZE);
  }
  cout << F("\n........................................................................") << endl;
  Card1.vwd()->rewind();
  while (fichier1.openNext(Card1.vwd(), O_READ)) {
    fichier1.getName(filename, 13);
    if (fichier1.isDir()) cout << F("Nom du répertoire : ") << filename << '/';
    else cout << F("Nom du fichier : ") << filename;
    cout << F("\tNombre d'octets : ") << fichier1.fileSize() << endl; //F("\tNiveau dans l'arborescence : ") << fichier1.curPosition() << endl;
    fichier1.close();
  }
  Card1.chdir('/');     //Card2.vwd()->rewind();
  var4 = NbrFoldersRoot(Card1, fichier1, &FoldersOnRoot[0][0], 0);     // Cette fonction remplit le tableau FoldersOnRoot[][] => NbrFoldersRoot(SdFat, SdFile, char *, uint8_t)
  cout << F("Nombre de répertoires sous la racine : ") << (uint32_t)var4 << endl;
  for (n = 0; n < var4; n++) {
    t = 0;
    do {
      cout << FoldersOnRoot[n][t++];
    } while(FoldersOnRoot[n][t] != '\0');
    cout << endl;
  }
}
/****************************************************************************************************/
/* Pour lire la capacité mémoire en Go. La fonction récupère un entier long uint64_t et renvoie un  */
/* entier uint8_t pour indiquer la capacité arrondie.                                               */
/****************************************************************************************************/
/****************************************************************************************************/
/* Pour identifier le contenu d'une mémoire SDCard avec ses répertoires et ses fichiers.            */
/* La fonction est appelée au démarrage du programme pour lister le contenu sur le terminal.        */
/* Cette fonction est un modèle qui est une version de celle qui est régulièrement vue sur tous les */
/* forums. Cette fonction est récurrente et s'appelle elle même donc numTabs a plusieurs références */
/* pour chaque appel de la fonction comme avec un tableau de dimension à n hiérarchies.             */
/****************************************************************************************************/
void ViewingMemory(SdFat Card2, SdFile Files0rFolder, uint16_t numTabs, ArduinoOutStream cout) {
  while(true) {
    EtatLecture =  Files0rFolder.openNext(Card2.vwd(), O_READ);   // Reports the next file or folder in a directory
    if (! EtatLecture) {             // renvoie un booléen
      // no more files
      Files0rFolder.rewind();        // rewindDirectory() will bring you back to the first file in the directory, used in conjunction with openNextFile().
      if (numTabs == 0) cout << F("**no more files or directories**") << endl;
      Files0rFolder.close();      
      // nécessaire pour des commandes dir en série
      break;                                      // sortie de la boucle principale while(true) mais qui peut renvoyer après le deuxième appel
    }                                             // au break numTabs reprend la valeur initiale
    for (i = 0; i < numTabs; i++) cout << '\t';   // affichage d'une tabulation pour indiquer le niveau de la hiérarchie
    Files0rFolder.getName(filename, 13);          // ici cela peut être un fichier ou un répertoire
    k = 0;
    do { cout << filename[k++]; } while(filename != '\0');                        
    if (Files0rFolder.isDir()) {                  // On fait évoluer l'indice hiérarchique
       cout << "/" << endl;
       ViewingMemory(Card2, Files0rFolder, numTabs + 1, cout);         // On identifie dans ce répertoire le niveau hiérarchique des répertoires et des fichiers
     } else {
       // files have sizes, directories do not
       for (m = 0; m < 8 - numTabs; m++) cout << '\t';
       scratch_32bits = Files0rFolder.fileSize();
       cout << scratch_32bits << F(" bytes") << endl;
       Files0rFolder.close();                               // On ferme le fichier
    }
  }
}
/****************************************************************************************************/
/* Autre fonction qui liste le contenu de la SDCard.                                                */
/****************************************************************************************************/
void DirectoryList(SdFat Card1, SdFile MyFile, ArduinoOutStream cout1) {                         
  if (! Card1.chdir()) {          // On se place à la racine de la carte
    cout1 << F("Impossible de remonter à la racine car la carte n'existe pas ou n'est pas formatée") << endl;
  }
  cout1 << F("Lecture du contenu de la micro SDCard :") << endl;
  ViewingMemory(Card1, MyFile, 0, cout1);
}
/****************************************************************************************************/
/* dir : Inventaire du contenu d'une SD Card avec fichiers et répertoires selon l'arborescence.     */
/* Open next file in root.  The volume working directory vwd, is root. Warning, openNext starts     */
/* at the current position of sd.vwd() so a rewind may be neccessary in your application.           */
/* https://github.com/greiman/SdFat-beta/blob/master/SdFat/examples/OpenNext/OpenNext.ino           */
/* vwd : volume working directory, cwd : current working directory                                  */
/* Le flux cout ne fonctionne pas avec des objets de type String, il faut passer un tableau.        */
/* http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.LibrairieSdFatLib         */
/* Fonction NbrFoldersRoot() : renvoie le nombre de répertoires du niveau scruté et affiche le nom  */
/* des fichiers avec leurs caractéristiques.
/****************************************************************************************************/
void List_folder(SdFat Card1, SdFile fichier1) {
  uint8_t Folders_Ranking;          // niveau racine = 0
  Card1.vwd()->rewind();
  foldername[0] = '/';              // correspond au répertoire de la racine soit Card.vwd()->chdir("/")
  foldername[1] = '\0';

  Card1.chdir(&foldername[0]);      // pas besoin d'activer Card1.vwd()->rewind() revient à Card.chdir('/');
  // Affichage des fichiers et identification du nombre de répertoires sous la racine avec noms stockés dans FoldersOnRoot[][] et Index dans IndexFolders[0][x]
  Folders_Ranking = 0;              // niveau de la racine sans changement de répertoire
  Nbr_FoldersFromRoot[Folders_Ranking] = NbrFoldersRoot(Card1, fichier1, &FoldersOnRoot[0][0], Folders_Ranking);  // contient : while (fichier1.openNext(Card1.vwd(), O_READ))
  for (NbrFolder0 = 0; NbrFolder0 < Nbr_FoldersFromRoot[Folders_Ranking]; NbrFolder0++) {                         // On scrute les répertoires de la racine
    ChargementNomRep(NbrFolder0, Folders_Ranking);        // met à jour filename[] et foldername[] à partir d'un tableau à 2 dimensions et de Folders_Ranking
    FolderNameDisplayed(&filename[0], Folders_Ranking);   // filename[] contient le nom du répertoire qui va être ouvert                           
    AfficheFolderParam(Folders_Ranking, NbrFolder0);      // NbrFolder0 représente un numéro de répertoire pour un même niveau  (Index dans IndexFolders[Folders_Ranking][NbrFolder0])
    Card1.chdir(&foldername[0]);                          // ici on fixe le répertoire à scruter : foldername = / + filename
    // Affichage des fichiers et identification du nombre de répertoires sous la racine avec noms stockés dans FoldersOnLevel1[][] et Index dans IndexFolders[1][x]
    Folders_Ranking = 1;
    Nbr_FoldersFromRoot[Folders_Ranking] = NbrFoldersRoot(Card1, fichier1, &FoldersOnLevel1[0][0], Folders_Ranking);  // contient while (fichier1.openNext(Card1.vwd(), O_READ))
    for (NbrFolder1 = 0; NbrFolder1 < Nbr_FoldersFromRoot[Folders_Ranking]; NbrFolder1++) {
      ChargementNomRep(NbrFolder1, Folders_Ranking);
      FolderNameDisplayed(&filename[0], Folders_Ranking);
      AfficheFolderParam(Folders_Ranking, NbrFolder1);
      Card1.chdir(&foldername[0]);
      // Affichage des fichiers et identification du nombre de répertoires sous la racine avec noms stockés dans FoldersOnLevel2[][] et Index dans IndexFolders[2][x]
      Folders_Ranking = 2;
      Nbr_FoldersFromRoot[Folders_Ranking] = NbrFoldersRoot(Card1, fichier1, &FoldersOnLevel2[0][0], Folders_Ranking);
      for (NbrFolder2 = 0; NbrFolder2 < Nbr_FoldersFromRoot[Folders_Ranking]; NbrFolder2++) {
        ChargementNomRep(NbrFolder2, Folders_Ranking);
        FolderNameDisplayed(&filename[0], Folders_Ranking);
        AfficheFolderParam(Folders_Ranking, NbrFolder2);
        Card1.chdir(&foldername[0]);
        // Affichage des fichiers et identification du nombre de répertoires sous la racine avec noms stockés dans FoldersOnLevel3[][] et Index dans IndexFolders[3][x]
        Folders_Ranking = 3;
        Nbr_FoldersFromRoot[Folders_Ranking] = NbrFoldersRoot(Card1, fichier1, &FoldersOnLevel3[0][0], Folders_Ranking);
        for (NbrFolder3 = 0; NbrFolder3 < Nbr_FoldersFromRoot[Folders_Ranking]; NbrFolder3++) {
          ChargementNomRep(NbrFolder3, Folders_Ranking);
          FolderNameDisplayed(&filename[0], Folders_Ranking);
          AfficheFolderParam(Folders_Ranking, NbrFolder3);
          Card1.chdir(&foldername[0]);
          // Affichage des fichiers et identification du nombre de répertoires sous la racine avec noms stockés dans FoldersOnLevel4[][] et Index dans IndexFolders[4][x]
          Folders_Ranking = 4;
          Nbr_FoldersFromRoot[Folders_Ranking] = NbrFoldersRoot(Card1, fichier1, &FoldersOnLevel4[0][0], Folders_Ranking);
          for (NbrFolder4 = 0; NbrFolder4 < Nbr_FoldersFromRoot[Folders_Ranking]; NbrFolder4++) {
            ChargementNomRep(NbrFolder4, Folders_Ranking);
            FolderNameDisplayed(&filename[0], Folders_Ranking);
            AfficheFolderParam(Folders_Ranking, NbrFolder4);
          }
          Folders_Ranking = 3;
        }
        Folders_Ranking = 2;
      }
      Folders_Ranking = 1;
    }
    Folders_Ranking = 0;
  }
  cout << F("Nombre de répertoires sous la racine : ") << (uint32_t)Nbr_FoldersFromRoot[0] << endl;
  cout << F("**no more files or directories**") << endl;
}
/****************************************************************************************************/
/* Fonction pour lire le nombre de répertoires créés sous la racine et dans un répertoire donné.    */
/* Cette fonction renvoie un entier uint8_t qui correspond au nombre de répertoires.                */
/* Tous les répertoires sous la racine sont enregistrés dans un tableau qui est passé comme         */
/* paramètre à un pointeur de type char. Les taleaux possibles sont : FoldersOnRoot[20][12],        */
/* FoldersOnLevel1, FoldersOnLevel2, FoldersOnLevel3, FoldersOnLevel4 => [ligne][colonne].          */
/* Seuls les fichiers du répertoire sollicité sont affichés sur le terminal avec leurs paramètres.  */
/****************************************************************************************************/
uint8_t NbrFoldersRoot(SdFat Card2, SdFile MyFile2, char *ptr_Array2D, uint8_t Niveau_Arborescence) {   // (SdFat Card2, SdFile MyFile2, ArduinoOutStream cout2, char *ptr_Array2D, uint8_t Niveau_Arborescence)
  var2 = 0;
  j = 0;                                            // indice de ligne qui distinguera les 20 répertoires possibles pour un même niveau 
  char *ptr_scratch;
  ptr_scratch = ptr_Array2D;                        // initialisation (recopie de l'adresse)
  while (MyFile2.openNext(Card2.vwd(), O_READ)) {   // while file or folder entries available
    MyFile2.getName(filename, 13);                  // mise à jour de filename[13]
    cmd = &filename[0];
    NomDuFichier = String(cmd);                     // à ne pas utiliser avec le flux de sortie cout
    if (MyFile2.isDir()) {
      var2++;
      if (NomDuFichier == "System Volum") {         // vrai uniquement pour une lecture sous la racine
        var2--;                                     // on ne comptabilise pas ce répertoire particulier
      } else {
        IndexFolders[Niveau_Arborescence][j] = MyFile2.dirIndex();       // IndexFolders[5][20] pour 5 niveau il y a 20 possibilités
        t = 0;                                      // indice de colonne pour le tableau qui enregistre le nom des répertoires
        do {                                        // on récupère le nom du répertoire
          *(ptr_Array2D++) = filename[t++];         // FoldersOnRoot[j][t] = filename[t];  // array[LIGNES][COLONNES]                   
        } while(filename[t] != '\0');
        *ptr_Array2D = '\0';                        // pour ajouter la fin du tableau
        j++;                                        // pour le répertoire suivant
        ptr_Array2D = ptr_scratch + (j * NbCrFldName);       // 12 * sizeof(char);
      }
    } else {                                        // affichage de tous les fichiers sous la racine
      AffichNomFic(Niveau_Arborescence);            //  AffichNomFic(ArduinoOutStream cout, uint8_t Niveau_Racine) utilise filename[]
      AfficheParamFile(MyFile2, Niveau_Arborescence);        // indique l'index du fichier
    }
    MyFile2.close();
  }                                                 // End while file or folder entries available
  return var2;
}
/****************************************************************************************************/
/* Pour mettre à jour les caractéristiques essentielles d'un fichier ou d'un répertoire lu.         */
/* On modifie le tableau filename[13] et on met à jour la variable globale (String) NomDuFichier.   */
/* if (MyFile.isRoot()) cout << F("Ce fichier est à la racine") << endl;                            */
/* if (MyFile.isSubDir()) cout << F("Ce fichier est situé dans un sous-répertoire") << endl;        */
/****************************************************************************************************/
uint8_t Identification_Index_filename(SdFile MyFile2) {
  MyFile2.getName(filename, 13);
  cmd = &filename[0];
  NomDuFichier = String(cmd);     // à ne pas utiliser avec le flux de sortie cout
  return MyFile2.dirIndex();      // peut être nécessaire de le coupler avec dirBlock
}
/**************************************************************************************************************/
/* Pour afficher le nom du répertoire en tenant compte du niveau de l'arborescence.                           */
/* La tabulation se produit jusqu'à 6 caractères ce qui implique qu'un affichage de 5 caractères              */
/* produit un déplacement de 1 seul caractère. Au delà de 6 caractères, c'est une tabulation supplémentaire.  */
/**************************************************************************************************************/
void FolderNameDisplayed(char *ptr_filename, uint8_t Niveau_Racine) {
  NomDuRepertoire = String(ptr_filename);
  var5 = NomDuRepertoire.length();          // le nombre de caractères hors '\0'
  if (Niveau_Racine != 0) {
    do {
      cout << '\t';
      Niveau_Racine--;
    } while(Niveau_Racine != 0);
  }
  do {
    cout << *(ptr_filename++);
  } while(*ptr_filename != '\0');
  cout << '/';
  if (var5 < 6) cout << '\t';
}
/****************************************************************************************************/
/* Pour afficher l'index du fichier ou du répertoire lu. Racine => Root_level                       */
/* Index représente la position dans le tableau qui a été incrémentée avant l'appel ) => -1         */
/* Niveau_Racine sera strictement inférieur à 5.                                                    */
/****************************************************************************************************/
void AfficheParamFile(SdFile MyFile3, uint8_t Niveau_Racine) {     // (SdFile MyFile3, ArduinoOutStream cout, uint8_t Niveau_Racine)
  n = 6 - Niveau_Racine;    // pour compenser le déplacement induit par la tabulation du nom du fichier ou du répertoire
  if (Filename_length < 8) n++;
  do {
    cout << '\t';
    n--;
  } while(n != 0);
  scratch_32bits = MyFile3.fileSize();
  if (scratch_32bits < 10) cout << scratch_32bits << F(" bytes") << "\t\t" << F("Index du fichier : ") << (uint32_t)MyFile3.dirIndex();
  else cout << scratch_32bits << F(" bytes") << '\t' << F("Index du fichier : ") << (uint32_t)MyFile3.dirIndex();
  //cout << F("\tNiveau dans l'arborescence : ") << MyFile3.curPosition();
  //cout << F("\tNiveau dans l'arborescence : ") << MyFile3.dirEntry(&Folder);
  cout << endl;
}
/****************************************************************************************************/
/* Pour afficher les paramètres du répertoire qui ne concerne que son index.                        */
/* fonction qui doit tenir compte du paramètre Foldername_length pour le nombre de tabulations.     */
/* Folders_Ranking est strictement inférieur à 5.                                                   */
/****************************************************************************************************/
void AfficheFolderParam(uint8_t Niveau_Racine, uint8_t ArrayPos) {
  n = 6 - Niveau_Racine;           // pour compenser le déplacement induit par la tabulation du nom du fichier ou du répertoire
  if (Foldername_length < 8) n++;
  do {
    cout << '\t';
    n--;
  } while(n != 0);
  //cout << '\t' << F("Index du répertoire : ") << (uint32_t)IndexFolders[Niveau_Racine][ArrayPos];   // IndexFolders[][] mis à jour par NbrFoldersRoot()
  cout << endl;
}
/****************************************************************************************************/
/* Pour afficher le nom du fichier ou du répertoire en tenant compte du niveau de l'arborescence.   */
/* filename contient le nom du fichier et une option. Le paramètre Filename_length permettra de     */
/* définir le nombre de tabulation à appliquer.                                                     */
/****************************************************************************************************/
void AffichNomFic(uint8_t Niveau_Racine) {
  Filename_length = 0;
  do {
    Filename_length++;
  } while(filename[Filename_length] != '\0');     // on récupère le nombre de caractères du nom du fichier
  if (Niveau_Racine != 0) {
    do {
      cout << '\t';             // cout est un objet public propre à cette bibliothèque de fonction
      Niveau_Racine--;
    } while(Niveau_Racine != 0);
  }
  cout << filename;             // on transmet un tableau de char 
}
/****************************************************************************************************/
/* Réduction du nom du répertoire et Changement du répertoire de travail. Cela reveint à remonter   */
/* d'un niveau dans l'arborescence. https://forum.arduino.cc/index.php?topic=368391.0               */
/* Il faut fermer le fichier ou le répertoire avant de changer de répertoire.                       */
/****************************************************************************************************/
uint8_t ReduceFolderPath_ChangeFolder(SdFat Card1, uint8_t TreeLevel, SdFile MyFile4) {
  cmd = foldername;                 // foldername[45] est une variable globale, cmd est un pointeur de char
  FilePathFromRoot = String(cmd);
  var1 = FilePathFromRoot.lastIndexOf('/');     
  if (var1 == 0) {                  // cas d'un seul niveau
    TreeLevel = 0;                  // variable locale de retour qui renseigne Folders_Ranking
    Root_level = true;              // variable globale
    foldername[1] = '\0';           // sachant que foldername[0] est toujours égal à '/' => Card1.chdir('/'), on retourne à la racine
  } else {
    TreeLevel--;
    Root_level = false;
    foldername[var1] = '\0';        // on remonte dans la racine d'un seul niveau
  }
  MyFile4.close();                  // avant de changer de répertoire, il faut fermer le précédent fichier ou répertoire ouvert
  Card1.chdir(&foldername[0], true);      // Card.chdir("/", true);
  MyFile4.open("/", O_READ);
  MyFile4.openNext(&MyFile4, O_READ);       // bool openNext(FatFile* dirFile, uint8_t oflag = O_READ);
  Drapeaux_groupe1 |= (1<<ChgDir_flag);     // drapeau pour vérifier que le chemin a été réduit pour remonter dans la racine
  return TreeLevel;
}
/****************************************************************************************************/
/* pour modifier le contenu de filename[13] et foldername[45] qui contient le chemin absolu.        */
/* NumFolder correspond à l'ordre de lecture du répertoire pour un même niveau.                     */
/* Le chemin (path) est absolu et il est impératif de le représenter complètement.                  */
/* https://forum.arduino.cc/index.php?topic=368391.0                                                */
/****************************************************************************************************/
void ChargementNomRep(uint8_t NumFolder, uint8_t NiveauArbre) {
  t = 0;        // indice pour chaque caractère du nom du répertoire
  char Carac1;
  do {
    switch(NiveauArbre) {
      case 0:                                           // pour ce niveau il y a un seul caractère / (foldername[0] = '/')
        filename[t] = FoldersOnRoot[NumFolder][t++];    // [ligne][colonne]
        Carac1 = FoldersOnRoot[NumFolder][t];
        break;
      case 1:
        filename[t] = FoldersOnLevel1[NumFolder][t++];  // on récupère seulement le nom du répertoire
        Carac1 = FoldersOnLevel1[NumFolder][t];
        break;
      case 2:
        filename[t] = FoldersOnLevel2[NumFolder][t++];
        Carac1 = FoldersOnLevel2[NumFolder][t];
        break;
      case 3:
        filename[t] = FoldersOnLevel3[NumFolder][t++];
        Carac1 = FoldersOnLevel3[NumFolder][t];
        break;
      case 4:
        filename[t] = FoldersOnLevel4[NumFolder][t++];
        Carac1 = FoldersOnLevel4[NumFolder][t];         // caractère suivant
        break;
      default:
        break;
    }
  } while(Carac1 != '\0');
  filename[t] = '\0';                     // On récupère seulement le nom du répertoire sans le chemin complet
  Foldername_length = t;                  // fixe le nombre de caractères qui seront affichés
  FolderPathModification(NiveauArbre);    // connaissant filename on modifie foldername
}
/****************************************************************************************************/
/* Modification du chemin d'accès en absolu.                                                        */
/* foldername[45] est une variable globale qui n'est modifiée qu'ici et donc en tenant compte du    */
/* niveau dans la racine on identifie la partie à modifier. sd.chdir("/Folders/Folder_A")           */
/****************************************************************************************************/
void FolderPathModification(uint8_t Levelroot) {
  if (Levelroot == 0) {                     // cas pour lequel on se situe à la racine
    k = 1;                                  // La valeur foldername[0] = '/' restera inchangée
    do { 
      foldername[k++] = filename[k - 2];    // k est incrémenté dès la première occurence k++ 
    } while(filename[k - 2] != '\0');
  } else ChangeAbsolutePath(Levelroot);     // Ceci implique que foldername contient au moins un nom de répertoire
}
/****************************************************************************************************/
/* Remplace le chemin absolu en fonction du niveau de la racine et du contenu des tableaux          */
/* foldername[] et filename[]. Le niveau permet d'identifier le nombre de caractère '/'             */
/****************************************************************************************************/
void ChangeAbsolutePath(uint8_t LevelPath) {
  uint8_t Pos_Insertion;                  // pour insérer le caractère '/'
  k = 0;
  var2 = 0;
  do {                                    // on comptabilise le nombre de caractères '/'
    if (foldername[k] == '/') {
      var2++;                             // var2 vaut au moins 1 car foldername[0] = '/'
      if (var2 == LevelPath + 1) Pos_Insertion = k + 1;
    }
    k++;
  } while(foldername[k] != '\0');
  i = 0;
  if (var2 < (LevelPath + 1)) {           // Dans ce cas on complète le chemin
    foldername[k++] = '/';
    do {
      foldername[k++] = filename[i++];
    } while(filename[i] != '\0');
    foldername[k] = '\0';
  } else {                                // Le caractère '/' existe déjà
    do {
      foldername[Pos_Insertion++] = filename[i++];
    } while(filename[i] != '\0');
    foldername[Pos_Insertion] = '\0';
  }
}
/****************************************************************************************************/
/* Fonction pour supprimer un fichier quand le chemin d'accès est identifié à  partir de la racine. */
/* Commande 'del_'<Répertoire> ou Répertoire = REP1/REP2/REP/.../FichierASupprimer.xxx              */
/* filepath_array[58] contient le chemin et le nom du fichier et se termine par le caractère '\0'.  */
/****************************************************************************************************/
void DeleteFile(String Cde_received, SdFat Card1, ArduinoOutStream cout) {
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);      // fin de tableau avec '\0'
  for (n = 0; n < Cde_received.length() - 4; n++) filepath_array[n] = TabASCII[n + 4];  // après del_
  filepath_array[Cde_received.length() - 4] = '\0';
  cout << F("Chemin et fichier saisis : ");
  for (k = 0; k < Cde_received.length() - 4; k++) cout << filepath_array[k];  // on affiche le chemin complet du fichier
  cout << endl;
  var3 = Cde_received.lastIndexOf('/');           // https://www.arduino.cc/en/Tutorial/StringIndexOf
  for (n = 0; n < Cde_received.length() - var3; n++) filename[n] = TabASCII[var3 + 1 + n];    // on extrait le nom du fichier avec l'option
  filename[Cde_received.length() - var3] = '\0';  // filename[12]
  cmd = &filename[0];
  NomDuFichier = String(cmd);
  //EtatLecture = SD.remove(filepath_array[0]);   // bool SD.remove(char *filepath)
  EtatLecture = Card1.remove(filepath_array);
  cout << F("Le fichier ") << NomDuFichier;
  if (EtatLecture == false) cout << F(" n'existe pas ou le chemin d'accès est faux......") << endl;
  else cout << F(" a été supprimé.") << endl;
}
/****************************************************************************************************/
/* Fonction pour supprimer un répertoire quand le chemin d'accès est connu à  partir de la racine.  */
/* Commande 'rmdir_'<Répertoire> ou Répertoire = REP1/REP2/REP/.../REPx                             */
/* filepath_array[58] contient le chemin complet du répertoire depuis la racine.                    */
/****************************************************************************************************/
void DeleteFolder(String Cde_received, SdFat Card1, ArduinoOutStream cout) {
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  for (n = 0; n < Cde_received.length() - 6; n++) foldername[n] = TabASCII[n + 6];      // après rmdir_
  foldername[Cde_received.length() - 6] = '\0';
  cout << F("Nom du répertoire à supprimer : ");
  cmd = &foldername[0];                             // cmd est un pointeur de char
  FolderPathFromRoot = String(cmd);                 // l'écriture ne correspond pas à un cast   
  k = FolderPathFromRoot.lastIndexOf('/') + 1;      // le comptage débute avec 0
  t = 0;
  do {
    filename[t++] = foldername[k];                  // filename[] contient le répertoire à supprimer
    cout << foldername[k++];
  } while(foldername[k] != '\0');                   // on affiche le nom du répertoire à supprimer
  cout << endl;
  filename[t] = '\0';
  EtatLecture = Card1.rmdir(foldername);
  cout << F("Le répertoire ") << filename;
  if (EtatLecture == true) cout << F(" a été supprimé...") << endl;
  else cout << F(" n'a pas pu être supprimé car il n'existe pas.") << endl;
}
/****************************************************************************************************/
/* Fonction pour créer un répertoire avec le nom du chemin complet à partir de la racine.           */
/* La commande est de la forme 'mkdir_'<Répertoire>.                                                */
/* foldername[45] contient le chemin sur 5 niveaux possibles et se termine par le caractère '\0'.   */
/****************************************************************************************************/
void CreateDirectory(String Cde_received, SdFat Card1) {
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  for (n = 0; n < Cde_received.length() - 6; n++) foldername[n] = TabASCII[n + 6];    // après mkdir_
  foldername[Cde_received.length() - 6] = '\0';
  cout << F("Chemin complet saisi : ");         
  k = 0;
  do {
    cout << foldername[k++];
  } while(foldername[k] != '\0');     // on affiche le chemin complet du répertoire à créer
  cout << endl;
  cout << F("Nom du répertoire à créer : ");
  cmd = &foldername[0];                             // cmd est un pointeur de char
  FolderPathFromRoot = String(cmd);                 // l'écriture ne correspond pas à un cast
  k = FolderPathFromRoot.lastIndexOf('/') + 1;      // le comptage débute avec 0
  t = 0;
  do {
    filename[t++] = foldername[k];                  // filename[] contient le répertoire à créer
    cout << foldername[k++];
  } while(foldername[k] != '\0');                   // on affiche le nom du répertoire à créer
  cout << endl;
  filename[t] = '\0';
  EtatLecture = Card1.mkdir(foldername);            // on transmet le chemin complet
  cout << F("Le répertoire ") << filename;
  if (EtatLecture == false) cout << F(" n'a pas pu être créé......") << endl;
  else cout << F(" a été créé.") << endl;
}
/****************************************************************************************************/
/* Fonction pour changer de répertoire courant. La commande est de la forme 'chdir_'<Répertoire>.   */
/* Le but est de permettre à un utilisateur de changer  de répertoire pour y créer son fichier.     */
/* La personne est censée démarrer au niveau de la racine. Commande 'chdir_'<Répertoire>            */
/****************************************************************************************************/
void ChangeDirectory(String Cde_received, SdFat Card1, SdFile fichier1) {
  Indice3 = 0;                                                                        // Cette variable doit être maintenue à 0 si le chemin existe
  InitArray2dimensions(&FoldersOnRoot[0][0], NbCrFldName, NbrFolders);                // remplit toutes les cases avec '\0' [lignes][colonnes]
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  for (n = 0; n < Cde_received.length() - 6; n++) foldername[n] = TabASCII[n + 6];    // après mkdir_
  foldername[Cde_received.length() - 6] = '\0';                                       // foldername[45]
  cout << F("Chemin complet saisi : ") << foldername << endl;                         // on affiche le chemin complet du répertoire à créer
  cmd = &foldername[0];                                                               // cmd est un pointeur de char
  FolderPathFromRoot = (String)cmd;                                                   // FolderPathFromRoot.reserve(45)
  Affiche_ligne_String(FolderPathFromRoot);                                           // "chaîne lue : "
  m = 0;
  Indice5 = 0;                                                                        // Cet indice calcule le déplacement dans le chemin complet après le caractère /
  test[m] = FolderPathFromRoot.indexOf('/');                                          // niveau de la racine pour le premier test
  while(test[m] != -1) {
    for (k = 0; k < test[m]; k++) FoldersOnRoot[m][k] = foldername[k + Indice5];      // on extrait le premier répertoire sous la racine
    Indice5 += test[m] + 1;
    FoldersOnRoot[m][test[m]] = '\0';
    FolderPathFromRoot = FolderPathFromRoot.substring(test[m++] + 1);                 // On récupère tout le reste
    test[m] = FolderPathFromRoot.indexOf('/');
    cout << F("Valeur de Indice5 : ") << (uint32_t)Indice5 << endl;
  }
  cout << F("valeur de m à la sortie de boucle : ") << (uint32_t)m << endl;
  /*Affiche_ligne_Array(&FoldersOnRoot[0][0]);
  Affiche_ligne_Array(&FoldersOnRoot[1][0]);
  Affiche_ligne_String(FolderPathFromRoot);
  EtatLecture = Card1.chdir("REP4");
  cout << F("Test REP4 : ") << EtatLecture;
  EtatLecture = Card1.chdir("TEMP");
  cout << F("Test TEMP : ") << EtatLecture;
  EtatLecture = Card1.chdir("TOPICS1");
  cout << F("Test TOPICS1 : ") << EtatLecture;*/
  if (m != 0) {                                         // le chemin comprend plusieurs répertoires
    for (t = 0; t < m; t++) {
      TestFolderExist(&FoldersOnRoot[t][0], Card1);     // On teste tous les répertoires mentionnés et modifie Indice3
      if (Indice3 != 0) break;
    }
  }
  if (Indice3 == 0) {                                   // initialisé à 0
    FolderPathFromRoot.toCharArray(foldername, 45);     // chaîne réduite pour conserver le répertoire le plus éloigné de la racine
    TestFolderExist(&foldername[0], Card1);             // Il faut tester au moins le répertoire lu qaund il n'est pas associé
  }
  /*switch(m) {                                                         // On accepte jusqu'à 5 niveaux de répertoires
    case 0:                                                           // du type REP/REP/REP/REP/REP
      TestFolderExist(&foldername[0], Card1);                         // il n'y avait qu'un répertoire annoncé
      break;
    case 1:
      TestFolderExist(&FoldersOnRoot[0][0], Card1);                   // On teste le premier répertoire mentionné
      if (Indice3 == 0) {
        FolderPathFromRoot.toCharArray(foldername, 45);               // Celui pour lequel il y a eu sortie de boucle
        TestFolderExist(&foldername[0], Card1);
      }
      break;
    case 2:
      TestFolderExist(&FoldersOnRoot[0][0], Card1);                   // On teste le premier répertoire mentionné
      if (Indice3 == 0) {
        TestFolderExist(&FoldersOnRoot[1][0], Card1);                 // On teste le deuxième répertoire mentionné
        if (Indice3 == 0) {
          FolderPathFromRoot.toCharArray(foldername, 45);             // Celui pour lequel il y a eu sortie de boucle
          TestFolderExist(&foldername[0], Card1);          
        }
      }
      break;
    case 3:
      TestFolderExist(&FoldersOnRoot[0][0], Card1);                   // On teste le premier répertoire mentionné
      if (Indice3 == 0) {
        TestFolderExist(&FoldersOnRoot[1][0], Card1);                 // On teste le deuxième répertoire mentionné
        if (Indice3 == 0) {
          TestFolderExist(&FoldersOnRoot[2][0], Card1);               // On teste le troisième répertoire
          if (Indice3 == 0) {
            FolderPathFromRoot.toCharArray(foldername, 45);           // Celui pour lequel il y a eu sortie de boucle
            TestFolderExist(&foldername[0], Card1);   
          }
        }
      }
      break;
    case 4:
      TestFolderExist(&FoldersOnRoot[0][0], Card1);                   // On teste le premier répertoire mentionné
      if (Indice3 == 0) {
        TestFolderExist(&FoldersOnRoot[1][0], Card1);                 // On teste le deuxième répertoire mentionné
        if (Indice3 == 0) {
          TestFolderExist(&FoldersOnRoot[2][0], Card1);               // On teste le troisième répertoire
          if (Indice3 == 0) {
            TestFolderExist(&FoldersOnRoot[3][0], Card1);
            if (Indice3 == 0) {
              FolderPathFromRoot.toCharArray(foldername, 45);           // Celui pour lequel il y a eu sortie de boucle
              TestFolderExist(&foldername[0], Card1);
            } 
          }
        }
      }
      break;
    default:
      break;
  }*/
  if (Indice3 == 0) cout << F("Le répertoire proposé existe dans l'arborescence de la SDCard.") << endl;
  Card1.vwd()->getName(foldername, 45);                                 // foldername[45]
  cout << F("Le répertoire courant est : ") << foldername << endl;
}
/****************************************************************************************************/
/* Changement de répertoire avec test pour vérifier son existence. On accepte des noms de           */
/* répertoires comprenant jusqu'à 12 caractères.                                                    */
/****************************************************************************************************/
void TestFolderExist(char *ptrNomFolder, SdFat CardSD) {
  k = 0;
  do {
    filename[k++] = *(ptrNomFolder++);                      // filename[13]
  } while(*ptrNomFolder != '\0');
  filename[k] = '\0';
  cmd = &filename[0];
  NomDuRepertoire = (String)cmd;                            // NomDuRepertoire.reserve(12)
  EtatLecture = CardSD.chdir(&filename[0]);
  if (EtatLecture == false) {
    Indice3++;
    cout << F("Le répertoire : ") << &filename[0] << F(" n'existe pas...") << endl;
  }
}
/****************************************************************************************************/
/* Fonction pour identifier le répertoire dans lequel on se situe après avoir utilisé la commande   */
/* 'check'.                                             */
/****************************************************************************************************/
//void CheckDirectory(String Cde_received, SdFat Card1, SdFile FicFolder) {
//  //Card1.vwd()->getName(filename, 13);                     // filename[13]; 
//  FicFolder.getName(filename, 13);         
//  if (fichier1.isDir()) cout << F("Nom du répertoire : ") << filename << '/';
//
//NomDuRepertoire = String(cmd); // NomDuRepertoire.reserve(12)
//
//  
//}



/****************************************************************************************************/
/* Edition du contenu d'un fichier vers le terminal. Commande : cat_'<REP/REP/NomDuFichier>         */
/* Le nom du fichier comprend 8 caractères au maximum avec 3 caractères pour définir l'option.      */
/* https://forum.arduino.cc/index.php?topic=368391.0                                                */
/****************************************************************************************************/
void LectureFichier(String Cde_received, SdFat Card1, SdFile fichier1) {
  FolderFileOptionPath(Cde_received, 4);        // modifie filepath_array[58] longueur : "_cat"
  cout << F("Le fichier avec son option et le chemin complet est : ") << filepath_array << endl;
  fichier1.open(&filepath_array[0], O_READ);
  while (fichier1.available()) Serial.write(fichier1.read());       // cout << fichier1.read() : ne fonctionne pas
  fichier1.close();
}
/******************************************************************************************************/
/* Ecriture dans un fichier en mode append pour ajouter des lignes en fin de fichier acquises à       */
/* partir du terminal. Commande : add_'<REP/REP/NomDuFichier> La commande permet d'appeler une        */
/* fonction dans laquelle on attend un envoi par l'utilisateur. On sort de la fonction si deux CR     */
/* + lF sont envoyés.                                                                                 */
/* http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.LibrairieSdFatLibSdFileOpen */
/******************************************************************************************************/
void AppendWrite(String Cde_received, SdFat Card1, SdFile fichier1) {
  FolderFileOptionPath(Cde_received, 4);        // modifie filepath_array[58] longueur : "add_"
  computer_bytes_received = 0;
  memset(computerdata, '\0', sizeof(computerdata));         // computerdata[60] équivalent à : computerdata[0] = '\0'
  cout << F("Vous pouvez écrire une ligne de moins de 60 caractères => ?") << endl;                 
  //Serial.flush();                                               
  do {
    if (Serial.available() > 0) {         // https://www.arduino.cc/en/Reference/Serial renvoie le nombre de caractères dans le buffer de réception  
      computer_bytes_received = Serial.readBytesUntil(CR, computerdata, 60);  // lecture du buffer de réception des données venant du terminal jusqu'à <CR> CR exclu
      computerdata[computer_bytes_received] = '\0';                           // ajout de Null pour définir la fin du tableau après le dernier caractère reçu 
    }
  } while(computer_bytes_received == 0 || computer_bytes_received == 1);
  cout << F("Chaîne réceptionnée : ") << computerdata << endl;
  fichier1.open(&filepath_array[0], O_APPEND | O_WRITE);                    // http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.LibrairieSdFatLibSdFileOpen
  fichier1.println(computerdata);
  fichier1.close();
}
/****************************************************************************************************/
/* Fonction pour identifier le répertoire, le fichier et son option dans la commande transmise.     */
/* Cde_length représente le nombre de caractères de la commande en y associant le caractère _.      */
/* Les principaux tableaux de caractères utiles pour désigner un fichier et son répertoire :        */
/* filepath_array[58] et FilePathFromRoot doivent avoir des dimensions équivalentes.                */
/****************************************************************************************************/
void FolderFileOptionPath(String CommandeRecue, uint8_t Nbr_car_Cde) {          // uint8_t Cde_Length, 
  CommandeRecue.toCharArray(TabASCII, CommandeRecue.length() + 1);              // fin de tableau avec '\0'
  for (n = 0; n < CommandeRecue.length() - Nbr_car_Cde; n++) filepath_array[n] = TabASCII[n + Nbr_car_Cde];
  filepath_array[CommandeRecue.length() - Nbr_car_Cde] = '\0';                  // filepath_array[58]
  cmd = &filepath_array[0];                                   // contient le chemin complet et le fichier à ouvrir
  FilePathFromRoot = (String)cmd;
  test[0] = FilePathFromRoot.lastIndexOf('/');                // la chaîne commence par l'indice 0 => 17
  test[1] = FilePathFromRoot.indexOf('.');                    // REP4/TEMP/TOPICS2/MOOC_005.txt => 26
  if (test[1] == -1) {
    cout << F("The file's option has been forgotten...") << endl;
  } else {
    if (test[0] != -1) {
      for (t = 0; t < test[1] - test[0] - 1; t++) filename[t] = filepath_array[t + test[0] + 1];
      filename[test[1] - test[0] - 1] = '\0';                                         // filename[13];
    } else {
      for (t = 0; t < test[1]; t++) filename[t] = filepath_array[t];
      filename[test[1]] = '\0';
    }
    for (t = 0; t < CommandeRecue.length() - 4 - test[1]; t++) optionfile[t] = filepath_array[t + test[1] + 1];
    optionfile[CommandeRecue.length() - 4 - test[1]] = '\0';                          // optionfile[4];
    cout << F("Fichier à lire : ") << filename << F(" avec l'option : ") << optionfile << endl;
  }
}
/****************************************************************************************************/
/* Fonction pour afficher sur le terminal, une chaîne de caractères de type String.                 */
/****************************************************************************************************/
void Affiche_ligne_String(String machaine) {
  machaine.toCharArray(TabASCII, 60);
  cout << F("chaîne lue : ") << TabASCII << endl;
}
/****************************************************************************************************/
/* Fonction pour afficher sur le terminal, une chaîne de caractères stockée dans un tableau.        */
/****************************************************************************************************/
void Affiche_ligne_Array(char *ptr_array) {
  cout << F("chaîne lue : ");
  do {
    cout << *(ptr_array++);
  } while(*ptr_array != '\0');
  cout << endl;
}
/****************************************************************************************************/
/* Fonction pour initialiser un tableau à deux dimensions en le remplissant de caractères NULL.     */
/****************************************************************************************************/
void InitArray2dimensions(char *ptr_arrayinit, uint8_t Nbr_Col, uint8_t Nbr_Row) {
  char *ptr_scratch2;
  ptr_scratch2 = ptr_arrayinit;                         // initialisation (recopie de l'adresse)
  for (i = 0; i < Nbr_Row; i++) {
    ptr_arrayinit = ptr_scratch2 + (i * NbCrFldName);   // array[LIGNES][COLONNES], FoldersOnRoot[20][NbCrFldName] 
    for (k = 0; k < Nbr_Col; k++) *(ptr_arrayinit++) = '\0';
  }
}


/* ######################################################################################################## */
// END of file
