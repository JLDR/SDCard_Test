/*
  SD card file dump
  This example shows how to read a file from the SD card using the
  SD library and send it over the serial port.
  contact : jean-louis.druilhe@univ-tlse3.fr
  The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11         pin 51
 ** MISO - pin 12         pin 50
 ** CLK - pin 13          pin 52
 ** CS - pin 4            pin 53            // défini comme constante
 */
// Set DISABLE_CHIP_SELECT to disable a second SPI device.
// For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
// to 10 to disable the Ethernet controller.
/* const int8_t DISABLE_CHIP_SELECT = -1; */
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select 
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
// Version used to test this library is 1.1.2 from https://www.arduinolibraries.info/libraries/sd-fat
/**************************************************************************************************************
This example shows how to read a file from the SD card using the SD library and send it over the serial port.
Application à une carte Arduino MEGA qui pourra aussi Ãªtre adaptée pour d'autres cibles (espace programme important à prévoir) 
Forums à consulter pour la mise en oeuvre : https://www.arduino.cc/en/Reference/SD
Connexions utilisées pour la carte UNO :        |               Connexions utilisées pour la carte MEGA :
    ** MOSI - pin 11                            |                   ** MOSI - pin 51  
    ** MISO - pin 12                            |                   ** MISO - pin 50
    ** CLK - pin 13                             |                   ** CLK - pin 52
    ** CS - pin 4 (SS = pin 10)                 |                   ** CS - pin 53 (SS) Any pin can be used for SD chip select
***************************************************************************************************************/
    
#include            "Fonctions.h"
#include            "RTC_DS3231.h"
#include            "ComUART.h"

/* Variables locales */
uint32_t            EntierLong1, EntierLong2, EntierLong3, EntierLong4;  // version 32 bits
const uint32_t      *ptr_lng0, *ptr_lng1;
const char          *ptrChar1;
bool                test0, test1, test2, test3;
uint8_t             Indice0, Indice1, Indice2, Indice3, Indice4, Indice5, Indice6, Indice7;
uint16_t            param0, param1, param2, param3;
uint8_t             i, j, k, l, m, n, t;
//volatile uint16_t   compt1, compt2;
//volatile uint8_t    cmpt_100us, cmpt_5ms;
char                TabConversionInteger[20];     // 2^64 = 18 446 744 073 709 551 616
char                TabASCII[60];
char                Scratch_tab[20];
//char                ConvAsciiToInt[5]; 
char                computerdata[60];             // permet de stocker les commandes ASCII provenant du PC ou du terminal
char                cinBuf[40];
uint8_t             computer_bytes_received;      // nombre d'octets transmis par le PC ou le terminal
uint32_t            scratch_32bits;
uint16_t            scratch_16bits;
uint8_t             scratch_8bits;
uint8_t             Ctrl_Flags;
uint16_t            lecture_masque;
String              Cde_received;
uint16_t            Val_dec;
uint8_t             adresse_scan;
String              Commande;                     // on créé une chaîne String à partir d'un pointeur String(nom_du_pointeur)
char                *cmd;                         // pointeur de type char utilisé pour les opérations de parsing
uint8_t             add_stamp;
uint8_t             var0, var1, var2, var3, var4, var5, var6;
int                 test[6];
uint8_t             pos0, pos1, pos2, pos3, pos4;
uint8_t             Nbr_FolderOnRoot;
uint8_t             Nbr_FoldersFromRoot[5];
char                FoldersOnRoot[NbrFolders][NbCrFldName];     // array[LIGNES][COLONNES], nom des répertoires limité à 8 caractères et situé sous la racine
char                FoldersOnLevel1[NbrFolders][NbCrFldName];
char                FoldersOnLevel2[NbrFolders][NbCrFldName];
char                FoldersOnLevel3[NbrFolders][NbCrFldName];
char                FoldersOnLevel4[NbrFolders][NbCrFldName];
uint8_t             IndexFolders[5][NbrFolders];                // array[LIGNES][COLONNES] 20 colonnes pour 20 répertoires différents au même niveau et pour 5 niveaux possibles
uint8_t             Drapeaux_groupe1;
uint8_t             Flags_reading;   
char                filepath_array[58];           // comprend le chemin d'accès et le nom du fichier
char                filename[13];                 // 8 caractères au maximum pour le nom du fichier et 4 pour l'option .xxx + '\0'
char                foldername[45];               // On prévoit 5 niveaux d'arborescence au maximum soit 8 caractères par répertoire + '/'
char                optionfile[4];
uint8_t             Foldername_length;
uint8_t             Filename_length;
String              FilePathFromRoot;             // lié à filepath_array[58]
String              FolderPathFromRoot;           // lié à foldername
bool                Root_level;
uint8_t             NbrFolder0, NbrFolder1, NbrFolder2, NbrFolder3, NbrFolder4, NbrFolder5;   // 5 niveaux de strates dans l'arborescence
String              NomDuFichier;
String              NomDuRepertoire;
String              OptionFichier;
String              FichierOuvert;;
uint64_t            volumesize;
uint32_t            cardSize;
bool                EtatLecture;
const int8_t        DISABLE_CHIP_SELECT = -1;
// Forme globale et structurée pour (FORMAT_SHORT ou FORMAT_LONG) associé à (FORMAT_LITTLEENDIAN ou FORMAT_BIGENDIAN ou FORMAT_MIDDLEENDIAN)
// FormatShort, FormatLong
// Format_LittleEndian, Format_BigEndian, Format_MiddleEndian
Representation_t    MyRepresentation;   // structure globale pour identifier dans ce module les réglages appliqués
Parity_t            MyParityTest;



/* Variables externes */
extern volatile uint16_t   cmpt1, cmpt2, cmpt3, cmpt4, cmpt5;                                 // Timers
extern volatile uint8_t    compt1, compt2, compt3, compt4, compt5, cmpt_100us, cmpt_5ms;      // Timers

/* Instanciation d'objets */
ArduinoOutStream VersLeTerminal(Serial);                            // Create a Serial output stream (ofstream class)
ArduinoInStream DepuisLeTerminal(Serial, cinBuf, sizeof(cinBuf));   // Create a serial input stream uniquement vrai avec SdFat.h déclarée
DS3231 RTC(SDA, SCL);
Time temps;
SdFile fichier;     /* instanciation des objets propres à SdFat */
SdFat MySD;         // File system object

void setup() {
  cli();
  Ctrl_Flags = ((1<<Timer1_ON)|(1<<Timer2_ON)|(1<<Timer3_ON)|(1<<Timer4_ON));   // 5 ms (timer1) et 100 µs (timer2)     
  Init_Timers(Ctrl_Flags, 250, 10000, 200, 2500, 25000, 3125);                  // Timer0 .... Timer5, avoid : Timer5_ON and Timer0_ON
  sei();
  Serial.begin(115200);           // Open serial communications and wait for port to open:
  while (!Serial);                // wait for serial port to connect. Needed for Leonardo only
  
  RTC.begin();                                        // Initialize the rtc object (méthode qui ne fait absolument rien)
  LINE("Liaison I2C SDA :", SDA);
  LINE("Liaison I2C SCL :", SCL);
  Serial.println(F("\nDernière programmation de la mémoire flash :"));
  LINE("__DATE__", __DATE__);                         // indique la date de la mise à jour hardware
  LINE("__TIME__", __TIME__);                         // indique l'heure de la mise à jour hardware
  Serial.println(F("\nType de carte connectée"));
  #if defined(__AVR_ATmega328P__)
    LINE("__AVR_ATmega328P__", __AVR_ATmega328P__);
    #define PROGMEM_SIZE 32768
  #endif
  #if defined(__AVR_ATmega2560__)
    LINE("__AVR_ATmega2560__", __AVR_ATmega2560__);   // Arduino Mega 2560
    #define PROGMEM_SIZE 262144
  #endif
  
  VersLeTerminal << F("Initializing SD card.............") << endl;
  //if (!MySD.init(SPI_HALF_SPEED, chipSelect)) sd.initErrorHalt();
  if (!MySD.begin(SD_CHIP_SELECT, SD_SCK_MHZ(50))) {          // see if the card is present and can be initialized (SPI_HALF_SPEED)
    VersLeTerminal << F("Card failed, or not present") << endl << F("You must disconnect your application....") << endl;
    MySD.initErrorHalt();
    do { 
      VersLeTerminal << '.';
    } while(1);                           // don't do anything more
  } else {
    VersLeTerminal << F("card initialized.") << endl << endl;
    //VersLeTerminal << F("Lecture du contenu de la micro SDCard :") << endl;
    //help();
  }
  
//  Fichier_courant = SD.open("datalog.txt", FILE_WRITE);     // re-open the file for writing
//  Serial.print(F("Writing..."));
//  if (Fichier_courant) {                                    // if the file is available, write to it:
//    for (i = 0; i < 10; i++) {
//      Fichier_courant.println("writing test " + (char)(0x30 + i));
//    }
//    Fichier_courant.close();                                // close the file:
//    Serial.println(F("Ecriture dans le fichier faite"));
//    Serial.println(F("Fichier fermé\n\n\n"));
//  }

//  Fichier_courant = SD.open("datalog.txt");                 // re-open the file for reading
//  if (Fichier_courant) {
//    Serial.println("Lecture du contenu du fichier datalog.txt");
//    while (Fichier_courant.available()) {                   // read from the file until there's nothing else in it
//      Serial.write(Fichier_courant.read());
//    }
//    Fichier_courant.close();                                // close the file:
//  } else {
//    Serial.println(F("error opening datalog.txt"));         // if the file didn't open, print an error:
//  }
  /*if (SD.exists("ESSAI_7/D_45.TXT")) {                      // test pour vérifier l'emplacement
    SD.remove("ESSAI_7/D_45.TXT");
  }*/
  Drapeaux_groupe1 = 0;                                   // initialisation des drapeaux
  Commande.reserve(60);
  FilePathFromRoot.reserve(58);                           // le chemin complet avec le nom du fichier et son option (lié à filepath_array[58])
  FolderPathFromRoot.reserve(45);                         // seulement le chemin complet (lié à foldername[45])
  computer_bytes_received = 0;                            // nombre de caractères reçus avec le terminal
  NomDuFichier.reserve(16);                               // seulement le nom du fichier
  NomDuRepertoire.reserve(12);                            // On autorise la dénomination d'un répertoire avec 12 caractères ASCII
  OptionFichier.reserve(5);                               // son option
  FichierOuvert.reserve(21);                              // nom du fichier '.' option
  MyRepresentation.MyEndianness = Format_LittleEndian;    // constante par directive FORMAT_LITTLEENDIAN;
  MyRepresentation.MyShortLong = FormatShort;             // constante par directive FORMAT_SHORT;
  UART_StatusWrite(UART1, enable_UART);                   // UART1 est connu de tout le module dès que ComUART.h est appelé
  UART_StatusWrite(UART2, disable_UART);                  // UART2 est un entier de type int et vaut 2
  UART_StatusWrite(UART3, disable_UART);                  // UART3 = 3
  Init_UART(UART1, 19200UL, CharSize8bits, NoneParity, OneStopBit);
  
}

void loop() {
 
  compt1 = 0;
  do { } while (compt1 <= 100);           /* temporisation de 100 x 5 ms = 500 ms */
  
  do {
    if (computer_bytes_received != 0) {               // commande reçue ou du moins texte ASCII à interpréter
      computer_bytes_received = 0;                    // pour la réception d'autres commandes
      cmd = computerdata;                             // initialise le pointeur avec la première adresse du tableau cmd = &computerdata[0];
      Commande = (String)cmd;                         // https://www.arduino.cc/en/Reference/StringConstructor
      if (computerdata[0] != LF && Flags_reading == 0) VersLeTerminal << F("Commande lue : ") << computerdata << endl;      // il faut passer un tableau de caractères
      if (Commande.startsWith("help")) help();
      if (Commande.startsWith("checkFile")) Check_File(Commande, MySD, VersLeTerminal, fichier);    // Commande du type 'checkFile_'<NomDuFichier.option>
      if (Commande.startsWith("mkfile")) CreationFichier(Commande, MySD, fichier);
      if (Commande.startsWith("cat_")) LectureFichier(Commande, MySD, fichier);
      if (Commande.startsWith("add_")) AppendWrite(Commande, MySD, fichier);
      if (Commande.startsWith("scan")) scani2c();
      if (Commande.startsWith("checkSD")) CheckTypeSDCard(MySD, fichier);               // CheckTypeSDCard(MySD, VersLeTerminal, fichier)
      if (Commande.startsWith("sd")) DirectoryList(MySD, fichier, VersLeTerminal);      // à éviter
      if (Commande.startsWith("dir")) List_folder(MySD, fichier);
      if (Commande.startsWith("del")) DeleteFile(Commande, MySD, VersLeTerminal);
      if (Commande.startsWith("rmdir")) DeleteFolder(Commande, MySD, VersLeTerminal);
      if (Commande.startsWith("mkdir")) CreateDirectory(Commande, MySD);
      if (Commande.startsWith("chdir")) ChangeDirectory(Commande, MySD, fichier);       // utile pour créer un fichier dans l'arborescence des répertoires
      //if (Commande.startsWith("check")) ChangeDirectory(Commande, MySD);
      if (Commande.startsWith("cfgtime_")) Change_heure(Commande, RTC);                 // 'cfgtime_'<hhmmss>
      if (Commande.startsWith("cfgdate_")) Change_date(Commande, RTC);                  // 'cfgdate_'<aaaammjj>
      // met à jour les variables heures_lues, minutes_lues, secondes_lues, jour_lu, mois_lu, annee_lue
      if (Commande.startsWith("lecture")) MyRepresentation = lecture(RTC, MyRepresentation);
      if (Commande.startsWith("rtchelp")) helprtc();                                    // spécifique à l'horloge RTC
      if (Commande.startsWith("format_")) formatage(Commande);                          // 'format_'<xx>
      if (Commande.startsWith("format?")) lect_format(MyRepresentation);                // 'format?'
      if (Commande.startsWith("uarthelp")) uarthelp();
      if (Commande.startsWith("uart")) ConfigUART(Commande);                            // 'uart'<d_d_dAd>
      if (Commande.startsWith("readuart_")) UARTReading(Commande);                      // 'readuart_'<d>
      if (Commande.startsWith("intuart_")) UART_Interrupts_config(Commande);            // 'intuart_'<xxxx>
      if (Commande.startsWith("readint_")) ReadUARTInterrupts(Commande);                // 'readint_'<d>

      
    }
    /* ici on récupère tout ce qui est présent dans le buffer à condition que le caractère retour chariot soit entré */
    if (Serial.available() > 0) { // https://www.arduino.cc/en/Reference/Serial renvoie le nombre de caractères dans le buffer de réception                 
      computer_bytes_received = Serial.readBytesUntil(CR, computerdata, 60);   // lecture du buffer de réception des données venant du terminal jusqu'à <CR> CR exclu
      computerdata[computer_bytes_received] = Null;   // ajout de Null pour définir la fin du tableau après le dernier caractère reçu
      //Flags_reading = Drapeaux_groupe1 & (1 << TermOutput_flag);
      if (computerdata[0] != LF) {                    // '\n' (computerdata[0] != LF && Flags_reading == 0)
        VersLeTerminal << F("\nNombre de caractères reçus : ") << (uint32_t)computer_bytes_received << endl;
        //VersLeTerminal << F("Caractère reçus : ") << computerdata[0] << F("---") << endl;   // pour vérifier la présence d'un caractère LF en position 0 une fois que le buffer est vidé
      }
    }
  } while (1);
}
