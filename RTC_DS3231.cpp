
#include    "RTC_DS3231.h"
//#include    "Fonctions.h"

// Variables externes
extern uint32_t             EntierLong1, EntierLong2, EntierLong3, EntierLong4;  // version 32 bits
extern const uint32_t       *ptr_lng0, *ptr_lng1;
extern const char           *ptrChar1;
extern bool                 test0, test1, test2, test3;
extern uint8_t              Indice0, Indice1, Indice2, Indice3, Indice4, Indice5, Indice6, Indice7;
extern uint16_t             param0, param1, param2, param3;
extern uint8_t              i, j, k, l, m, n, t;
extern char                 TabConversionInteger[20];     // 2^64 = 18 446 744 073 709 551 616

extern char                 Scratch_tab[20];

extern char                 computerdata[60];             // permet de stocker les commandes ASCII provenant du PC ou du terminal
extern char                 cinBuf[40];
extern uint8_t              computer_bytes_received;      // nombre d'octets transmis par le PC ou le terminal
extern uint32_t             scratch_32bits;
extern uint16_t             scratch_16bits;
extern uint8_t              scratch_8bits;
extern uint16_t             lecture_masque;

// Variables locales propres au module
char                        ConvAsciiToInt[5];            // tableau de '0' à "65535"
char                        CdeAsciiArray[40];
uint8_t                     heures, minutes, secondes, jour, mois;
uint8_t                     heures_lues, minutes_lues, secondes_lues, jour_lu, mois_lu, annee_lue;
uint8_t                     check_date, check_month, check_year;
uint8_t                     check_heure, check_minutes;
uint8_t                     cycle_saison;   // contient 0x5B ou 0xA4
uint8_t                     jour_ref;       // jour pour lequel il y aura changement d'horaire soit à 3H00 (horaire d'hiver) soit à 2H00 (horaire d'été)
uint16_t                    annee;
uint8_t                     drapeaux_maj;
uint32_t                    temps_UNIX;     // from -2,147,483,648 to 2,147,483,647 (-2^31 to 2^31 - 1)
char                        DateArray[10];
EndianFormat_t              Endianness;     // endianisme ou boutisme
FormatType_t                ShortLong;      // SHORT or LONG format




// Instanciation d'objets
DS3231 MyRTC(SDA, SCL);
static Eeprom24C32_64 eeprom(EEPROM_ADDRESS);   // instanciation de l'objet appelé eeprom https://forum.arduino.cc/index.php?topic=38143.0
Time horloge;   // instanciation d'un objet Time dont la méthode vide (constructeur) initialise l'objet courant soit temps


/****************************************************************************************************/
/* On récupère les paramètres transmis qui sont contenus dans la commande 'cfgtime_'<hhmmss>.       */
/* Chaque information transmise correspond à un caractère ASCII et pour les nombres, les valeurs    */
/* entières sont comprises entre 0x30 et 0x39.                                                      */
/****************************************************************************************************/
void Change_heure(String Cde_received, DS3231 MyRTC) {              // cfgtime_hhmmss
  uint8_t k = 0;
  memset(CdeAsciiArray, '\0', sizeof(CdeAsciiArray));
  memset(ConvAsciiToInt, '\0', sizeof(ConvAsciiToInt));
  Cde_received = Cde_received.substring(8);  
  Cde_received.toCharArray(CdeAsciiArray, Cde_received.length() + 1);     // lié au caractère null de fin de tableau '\0'
  ConvAsciiToInt[0] = CdeAsciiArray[k++];                                 // dizaine des heures qui ont été transmises
  ConvAsciiToInt[1] = CdeAsciiArray[k++];                                 // valeur retenue pour définir le changement d'horaire
  ConvAsciiToInt[2] = '\0';
  heures = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  minutes = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);  
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  secondes = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  if (heures <= 23 && heures >= 0) {                                // pour éliminer les cas exotiques
    if (minutes <= 59 && minutes >= 0) {
      if (secondes <= 59 && secondes >= 0) {
        MyRTC.setTime(heures, minutes, secondes);
        drapeaux_maj |= (1<<flag_maj_heure);
      }
    }
  }
}
/****************************************************************************************************/
/* La fonction ci-dessous n'est jamais sollicitée entre 2H00 et 3H00 du matin, ce qui implique que  */
/* quand le drapeau concerné est activé, on peut définir la période hivernale ou estivale. C'est le */
/* programme principal qui se charge d'écrire en eeprom la valeur 0x5B aux adresses 0x200 ou 0x201. */
/* Le tableau TabASCII[] correspond au conteneur du texte lu avec le terminal.                      */
/* La commande est du type : 'cfgdate_'<aaaammjj>                                                   */
/****************************************************************************************************/
void Change_date(String Cde_received, DS3231 MyRTC) {
  uint8_t k = 0;
  memset(CdeAsciiArray, '\0', sizeof(CdeAsciiArray));
  memset(ConvAsciiToInt, '\0', sizeof(ConvAsciiToInt));
  Cde_received = Cde_received.substring(8);
  Cde_received.toCharArray(CdeAsciiArray, Cde_received.length() + 1);
  ConvAsciiToInt[0] = CdeAsciiArray[k++];                     // premier chiffre de l'année
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = CdeAsciiArray[k++];                     // ceux qui nous intéressent pour le changement d'horaire
  ConvAsciiToInt[3] = CdeAsciiArray[k++];
  ConvAsciiToInt[4] = '\0';
  annee = Conv5AsciiCharToUint16_t(&ConvAsciiToInt[0]);       // format long
  check_year = Conv2AsciiCharToUint8_t(&CdeAsciiArray[2]);    // format court pour calculer l'indice dans les tableaux pour identifier le changement d'horaire
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  mois = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  check_month = Conv2AsciiCharToUint8_t(&CdeAsciiArray[4]);
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  jour = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  check_date = Conv2AsciiCharToUint8_t(&CdeAsciiArray[6]);
  if (annee > 2000 && annee <= 2040) {
    if (mois > 0 && mois <= 12) {
      if (jour > 0 && jour <= 31) {
        MyRTC.setDate(jour, mois, annee);
      }
    }
  }
  if (check_year >= 18 && check_year <= 99) {
    drapeaux_maj |= (1<<flag_maj_date);         // lorsque la carte démarre ce drapeau est supprimé
    Serial.print(F("Analyse de l'année avec check_year : "));
    Serial.println(check_year, DEC);
    Serial.print(F("Analyse du mois avec check_month : "));
    Serial.println(check_month, DEC);
    Serial.print(F("Analyse du jour avec check_date : "));
    Serial.println(check_date, DEC);
  }
}
/********************************************************************************************************/
/* Lecture du temps et de la date stockés dans le circuit spécialisé DS3231.                            */
/* Différence entre FORMAT_SHORT et FORMAT_LONG => 17 devient 2017                                      */
/* FORMAT_LITTLEENDIAN => 13/04/2017, FORMAT_BIGENDIAN => 2017/04/13, FORMAT_MIDDLEENDIAN => 04/13/2017 */
/* La fonction Affiche_heure met à jour heures_lues, minutes_lues et secondes_lues.                     */
/* En lisant l'heure, on récupère une pointeur qui est initialisé sur le premier caractère d'une chaîne */
/* comprenant 8 caractères.                                                                             */
/********************************************************************************************************/
Representation_t lecture(DS3231 MyRTC, Representation_t LocalRepresentation) {
  uint8_t var, k;
  char *ptr;
  Affiche_heure(MyRTC.getTime());                           // t = MyRTC.getTime() mise à jour de l'heure lue => Heure lue : 08:59:33
  ptr = MyRTC.getTimeStr(LocalRepresentation.MyShortLong);  // FormatShort, FormatLong
  Serial.print(F("Représentation de l'heure enregistrée "));
  if (LocalRepresentation.MyShortLong == FormatShort) {
    Serial.println(F("(FORMAT_SHORT) : "));
    var = 5;
    ShortLong = FormatShort;
  } else {
    Serial.println(F("(FORMAT_LONG) : "));
    var = 8;
    ShortLong = FormatLong;
  }
  LocalRepresentation.MyShortLong = ShortLong;
  for (k = 0; k < var; k++) Serial.print(*(ptr++));       // ptr1 pointe vers un tableau de la méthode getTimeStr()
  Serial.println(F("\n\n***** Formatage des données *****"));
  Serial.print(F("Endianisme ou boutisme : "));           // https://fr.wikipedia.org/wiki/Boutisme
  switch (LocalRepresentation.MyEndianness) {
    case Format_LittleEndian:
      Serial.println(F("FORMAT_LITTLEENDIAN"));
      Endianness = Format_LittleEndian;                   // variable globale à ce module
      break;
    case Format_BigEndian:
      Serial.println(F("FORMAT_BIGENDIAN"));
      Endianness = Format_BigEndian;
      break;
    case Format_MiddleEndian:
      Serial.println(F("FORMAT_MIDDLEENDIAN"));
      Endianness = Format_MiddleEndian;
      break;
  }
  LocalRepresentation.MyEndianness = Endianness;
  // pointeur sur char qui renvoie 2017/04/10 avec format LONG (initialisé au début de la chaîne)
  ptr = MyRTC.getDateStr(LocalRepresentation.MyShortLong, LocalRepresentation.MyEndianness, '/');        
  Serial.print(F("Représentation de l'année enregistrée : "));
  k = 0;
  do {
    DateArray[k++] = *ptr;
    Serial.print(*(ptr++));
  } while (*ptr != '\0');    // on vient lire le contenu du tableau output[] de la méthode getDateStr()
  switch(ShortLong) {
    case FormatShort:
      switch(Endianness) {
        case Format_LittleEndian:                             // Représentation de l'année : 25/07/17
          jour_lu = Conv2AsciiCharToUint8_t(&DateArray[0]);   // on fixe le premier caractère de l'octet à convertir
          mois_lu = Conv2AsciiCharToUint8_t(&DateArray[3]);
          annee_lue = Conv2AsciiCharToUint8_t(&DateArray[6]);
          break;
        case Format_BigEndian:                                // Représentation de l'année : 17/07/25
          jour_lu = Conv2AsciiCharToUint8_t(&DateArray[6]);
          mois_lu = Conv2AsciiCharToUint8_t(&DateArray[3]);
          annee_lue = Conv2AsciiCharToUint8_t(&DateArray[0]);
          break;
        case Format_MiddleEndian:                             // Représentation de l'année : 07/25/17
          jour_lu = Conv2AsciiCharToUint8_t(&DateArray[3]);
          mois_lu = Conv2AsciiCharToUint8_t(&DateArray[0]);
          annee_lue = Conv2AsciiCharToUint8_t(&DateArray[6]);
          break;
        default:
          break;
      }
      break;
    case FormatLong:
      switch(Endianness) {
        case Format_LittleEndian:                             // Représentation de l'année : 25/07/2017
          jour_lu = Conv2AsciiCharToUint8_t(&DateArray[0]);
          mois_lu = Conv2AsciiCharToUint8_t(&DateArray[3]);
          annee_lue = Conv2AsciiCharToUint8_t(&DateArray[8]);
          break;        
        case Format_BigEndian:                                // Représentation de l'année : 17/07/25
          jour_lu = Conv2AsciiCharToUint8_t(&DateArray[8]);
          mois_lu = Conv2AsciiCharToUint8_t(&DateArray[5]);
          annee_lue = Conv2AsciiCharToUint8_t(&DateArray[0]);
          break;        
        case Format_MiddleEndian:                             // Représentation de l'année : 07/25/17
          jour_lu = Conv2AsciiCharToUint8_t(&DateArray[3]);
          mois_lu = Conv2AsciiCharToUint8_t(&DateArray[0]);
          annee_lue = Conv2AsciiCharToUint8_t(&DateArray[8]);
          break;        
        default:
          break;
      }
      break;
    default:
      break;
  }
  Serial.print("\n");
  separateur1(61, '-');
  Serial.println(F(" Identification des paramètres : jour_lu, mois_lu, annee_lue "));
  separateur1(61, '-');
  Serial.print(F("jour_lu : ")); Serial.println(jour_lu);
  Serial.print(F("mois_lu : ")); Serial.println(mois_lu);
  Serial.print(F("annee_lue : ")); Serial.print(annee_lue);
  Serial.print(F("\n- Date Of the Week : "));
  ptr = MyRTC.getDOWStr(ShortLong);
  do {
    Serial.print(*(ptr++));
  } while (*ptr != '\0');
  Serial.print(F("\n- Mois enregistré : "));
  ptr = MyRTC.getMonthStr(ShortLong);
  do {
    Serial.print(*(ptr++));
  } while (*ptr != '\0');
  Serial.print(F("\n\n- Temps Unix : "));
  temps_UNIX = MyRTC.getUnixTime(MyRTC.getTime());
  Serial.println(temps_UNIX);
  return LocalRepresentation;
}
/****************************************************************************************************/
/* Affichage de l'heure. L'argument transmis à cette fonction est un objet de la classe Time.       */
/* Cette fonction met à jour les variables heures_lues, minutes_lues et secondes_lues.              */
/****************************************************************************************************/
void Affiche_heure(Time temps_lu) {
  Serial.print(F("Heure lue : "));
  heures_lues = temps_lu.hour;                    // uint8_t
  if (heures_lues <= 9) {
    Serial.print('0');
    Serial.print((char)(heures_lues + 0x30));
  } else Serial.print(heures_lues, DEC);
  Serial.print(':');
  minutes_lues = temps_lu.min;                     // uint8_t
  if (minutes_lues <= 9) {
    Serial.print('0');
    Serial.print((char)(minutes_lues + 0x30));
  } else Serial.print(minutes_lues, DEC);
  Serial.print(':');
  secondes_lues = temps_lu.sec;
  if (secondes_lues <= 9) {
    Serial.print('0');
    Serial.println((char)(secondes_lues + 0x30));
  } else Serial.println(secondes_lues, DEC);
}
/****************************************************************************************************/
/* On convertit un tableau de 2 caractères ASCII identifiés en un entier de 8 bits pour une valeur  */
/* de 0 à 99. Le premier paramètre transmis correspond au poids fort de l'entier qui ne peut pas    */
/* dépasser 99.                                                                                     */
/****************************************************************************************************/
uint8_t Conv2AsciiCharToUint8_t(char *ptr_ascii) {
  uint8_t scratch_8bits = 0;
  uint8_t n;
  for (n = 0; n < 2; n++) {     // pour 2 caractères ASCII représentant un nombre de 0 à 99
    if ((uint8_t)(*ptr_ascii) < 0x30 || (uint8_t)(*ptr_ascii) > 0x39) break;    // rupture de la boucle for
    else {
      if (n == 0) {
        scratch_8bits = ((uint8_t)(*ptr_ascii) - 0x30) * 10;        // Most significant byte
        ptr_ascii++;
      } else scratch_8bits += (uint8_t)(*ptr_ascii) - 0x30;
    }
  }
  if (scratch_8bits <= 1 && scratch_8bits >= 100) return 0;          // 2002 - 2099
  else return scratch_8bits;
}
/****************************************************************************************************/
/* Séparateur                                                                                       */
/****************************************************************************************************/
void separateur1(uint8_t nbr_carac, char caract) {
  for (i = 0; i < nbr_carac; i++) {
    Serial.print(caract);
  }
  Serial.println();
}
/****************************************************************************************************/
/* Conversion d'un tableau de char représentant une chaîne ASCII de 5 caractères au maximum pour    */
/* représenter un entier non signé de 16 bits (<65536).                                             */
/****************************************************************************************************/
uint16_t Conv5AsciiCharToUint16_t(char *ptr_array) {
  uint8_t n = 0;      // doit permettre d'identifier les caractères ASCII interprétables
  uint32_t entier_verif = 0UL;
  uint16_t result_local;
  char *ptr_local;
  uint8_t i;
  ptr_local = ptr_array;
  do {
    if ((uint8_t)(*ptr_array) < 0x30 || (uint8_t)(*ptr_array) > 0x39) break;    // rupture de la boucle do while
    else {
      Serial.print(*(ptr_array++));
      n++;            // nombre de caractères interprétables
    }
  } while(1);
  if (n != 0) {
    ptr_local += (n - 1) * sizeof(char);      // on fixe la position de l'unité dans le tableau
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
    }
  }
  if (entier_verif >= 65536) return 0;
  else return (uint16_t)entier_verif;
}
/****************************************************************************************************/
/* Menu Help pour identifier les commandes possibles avec l'horloge RTC de type DS3231.             */
/****************************************************************************************************/
void helprtc() {                         //print help dialogue
  separateur1(92, '=');
  Serial.println(F("- Les caractères < et > ne doivent pas être envoyés, ils précisent le paramètre contenu à l'intérieur"));
  Serial.println(F("- La commande est précisée entre apostrophe (qui ne doit pas être utilisée) avec symbole : '"));
  Serial.println(F("- Pour lire les adresses I2C courantes des modules connectés : 'scan'"));
  Serial.println(F("- Pour modifier l'heure avec minutes et secondes : 'cfgtime_'<hhmmss>"));
  Serial.println(F("- Pour modifier la date (année, mois et jour) : 'cfgdate_'<aaaammjj>"));
  Serial.println(F("- Pour lire l'ensemble des paramètres contenus dans l'horloge RTC : 'lecture'"));
  Serial.println(F("- Pour définir le format de lecture, on utilise la commande : 'format_'<xx>"));
  Serial.println(F("\t\t\t - avec deux chiffres :"));
  Serial.println(F("\t\t\t\t - le premier :\n\t\t\t\t\t\tFORMAT SHORT : 1\n\t\t\t\t\t\tFORMAT LONG : 2"));
  Serial.println(F("\t\t\t\t - le second :\n\t\t\t\t\t\tFORMAT LITTLEENDIAN : 1\n\t\t\t\t\t\tFORMAT BIGENDIAN : 2\n\t\t\t\t\t\tFORMAT MIDDLEENDIAN : 3"));
  Serial.println(F("- Pour lire le format : 'format?'"));
  Serial.println(F("- Pour modifier le jour en cours : 'jour' ou 'jour_'<x> avec x de 1 à 7"));
  Serial.println(F("\nCommande pour tester la fonction de conversion :"));
  Serial.println(F("- Pour lire un entier 16 bits à partir d'un tableau de type char : 'conv_'<x> avec x de 0 à 6"));
  Serial.println(F("- Pour modifier la variable (uint8_t) Indice : 'indice_'<x> avec x de 0 à 5"));
  Serial.println(F("- Pour écrire un octet en eeprom à l'adresse comprise entre 0 et 0xFFF : 'WRbyte_'<XXXxx>"));
  Serial.println(F("- Pour lire un octet en eeprom à l'adresse comprise entre 0 et 0xFFF : 'RDbyte_'<XXX>"));
  Serial.println(F("- Pour vérifier la conversion hexadécimale d'une chaîne HexASCII: 'Conv_'<XXXX>"));
  separateur1(92, '=');
//  cout_Fonctions << F("CONFIGURATION de l'horloge Real Time Clock (DS3231) :\n");
//  cout_Fonctions << F("- Pour lire le contenu de l'horloge RTC : 'lect'\n");
//  cout_Fonctions << F("- Pour modifier l'heure : 'cfgtime_'<hhmmss>\n");
//  cout_Fonctions << F("- Pour modifier la date : 'cfgdate_'<aaaammjj>\n");
//  cout_Fonctions << F("- Pour imposer un jour de la semaine : 'jour_'<d>\n");
//  cout_Fonctions << F("\t\tLundi -> 1,\tmardi -> 2,\tmercredi -> 3,\tjeudi -> 4,\n\t\tvendredi -> 5,\tsamedi -> 6,\tdimanche -> 7 \n");
//  cout_Fonctions << F("- Pour consulter le jour de la semaine : 'jour_'<?>\n");
//  cout_Fonctions << F("- Pour modifier automatiquement le jour de la semaine (DOW) en fonction de la date : 'jour'\n");
}
/****************************************************************************************************/
/* Modification des paramètres format et Endianisme avec la commande 'format_'<xx>.                 */
/* On renseigne les varaibles Endianness et ShortLong qui sont déclarées globales à ce module.      */
/****************************************************************************************************/  
void formatage(String Cde_received) {
  uint8_t var1, var2;
  Cde_received = Cde_received.substring(7);
  Cde_received.toCharArray(CdeAsciiArray, Cde_received.length() + 1);
  var1 = ((uint8_t)CdeAsciiArray[0] - 0x30);
  var2 = ((uint8_t)CdeAsciiArray[1] - 0x30);
  switch (var1) {
    case 1:
      ShortLong = FormatShort;
      break;
    case 2:
      ShortLong = FormatLong;
      break;
    default:
      ShortLong = FormatShort;
      break;
  }
  switch (var2) {
    case 1:
      Endianness = Format_LittleEndian;
      break;
    case 2:
      Endianness = Format_BigEndian;
      break;
    case 3:
      Endianness = Format_MiddleEndian;
      break;
    default:
      Endianness = Format_LittleEndian;
      break;
  }
}
/****************************************************************************************************/
/* Lecture du format ce qui revient à identifier l'endianisme ou boutisme utilisé mais aussi le     */
/* format short ou long des informations enregistrées oar l'horloge RTC.                            */
/****************************************************************************************************/
void lect_format(Representation_t LocalRepresentation) {
  Serial.print(F("Format de l'heure et de la date : "));
  if (LocalRepresentation.MyShortLong == FormatShort) Serial.println(F("FORMAT_SHORT"));
  else Serial.println(F("FORMAT_LONG"));
  Serial.print(F("Type d'écriture standardisée (boutisme ou endianisme) : "));
  switch (LocalRepresentation.MyEndianness) {
    case Format_LittleEndian:
      Serial.println(F("FORMAT_LITTLEENDIAN"));
      break;
    case Format_BigEndian:
      Serial.println(F("FORMAT_BIGENDIAN"));
      break;
    case Format_MiddleEndian:
      Serial.println(F("FORMAT_MIDDLEENDIAN"));   
      break;
    default:
      Serial.println(F("FORMAT_LITTLEENDIAN"));
      break;
  }
}

/* ######################################################################################################## */
// END of file
