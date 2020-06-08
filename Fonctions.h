/* ******************************************************************************************** */
/* Ensemble de fonctions dédiées pour cette application.                    */
/* ******************************************************************************************** */
#ifndef FONCTIONS_H_
#define FONCTIONS_H_  1

/* constantes */
#define         Nbr_Car_LCD       20
#define         Nbr_ligne         4
#define         Space             0x20
#define         Null              '\0'              // 0 ou '\0'
#define         Masque            0x8000
#define         SD_CHIP_SELECT    53                // connexion Chip select du support SDCard
#define         ShieldSDW5100     4
#define         W5100chipSelect   10
#define         F_cpu             16000000UL
#define         LF                0x0A              // '\n'
#define         CR                0x0D              // '\r'
#define         SPI_SPEED         SD_SCK_MHZ(4)     // if (!sd.begin(chipSelect, SPI_SPEED)) cas oÃ¹ la vitesse max ne fonctionne pas
#define         NbCrFldName       12
#define         NbrFolders        20
/* drapeaux Timers */
#define         Timer0_ON         0
#define         Timer1_ON         1
#define         Timer2_ON         2
#define         Timer3_ON         3
#define         Timer4_ON         4
#define         Timer5_ON         5
/* Déclaration de drapeaux */
#define         ChgDir_flag       0
#define         TermOutput_flag   3                 // drapeau pour permettre à la boucle principale d'être déroutée

/* contrairement au PSoC, c'est bien à cet endroit qu'il faut déclarer les appels des autres bibliothèques */
#include    <SPI.h>                 /* Serial */
#include    <Wire.h>                // bibliothèque pour la gestion des ressources I2C
#include    <SdFat.h>
#include    <string.h>              /* traitement des chaînes de caractères */
#include    <stdint.h>
#include    <stdio.h>
#include    <stdlib.h>              /* nécessaire pour la fonction atof */
#include    <math.h>
#include    <avr/pgmspace.h>
#include    <avr/interrupt.h>       /* vecteurs d'interruptions déclarés dans Fonctions.cpp */
#include    <LiquidCrystal_I2C.h>

/* prototypage des fonctions de la bibliothèque */
extern void Init_Timers(uint8_t, uint8_t, uint16_t, uint8_t, uint16_t, uint16_t, uint16_t);
extern uint8_t ConvertUint32ToASCIIChar(char *, uint32_t);    /* sprintf pout uint32_t */
extern uint8_t ConvertUint64ToASCIIChar(char *, uint64_t);    /* sprintf pout uint64_t */
extern uint8_t ConvertUint32ToASCIICharBis(char *, uint32_t);
extern void Affich_Cmd(char *, uint8_t);
extern void Affich_val_dec(uint16_t, uint8_t, uint8_t);
extern uint16_t detect_entier(char *, String);
extern uint16_t Lecture_DEC(char *, uint8_t);
extern uint16_t ConvASCIItoInt16(char *);

extern uint8_t Recherche_addI2C(uint8_t);       // fonction scan I2C
extern void scani2c(void);
extern void help(void);
extern void operateur_unaire(String);
extern uint16_t Convert_HexASCII_to_uint16(char *);
extern void Check_File(String, SdFat, ArduinoOutStream);
extern void Check_File(String, SdFat, ArduinoOutStream, SdFile);
extern void CreationFichier(String, SdFat, SdFile);
//extern void CheckTypeSDCard(SdFat, ArduinoOutStream, SdFile);
extern void CheckTypeSDCard(SdFat, SdFile);
extern void ViewingMemory(SdFat, SdFile, uint16_t, ArduinoOutStream);
extern void DirectoryList(SdFat, SdFile, ArduinoOutStream);
extern void List_folder(SdFat, SdFile);
extern uint8_t NbrFoldersRoot(SdFat, SdFile, char *, uint8_t);
extern uint8_t Identification_Index_filename(SdFile);
//extern void AfficheParamFile(SdFile, ArduinoOutStream, uint8_t);
extern void FolderNameDisplayed(char *, uint8_t);
extern void AfficheParamFile(SdFile, uint8_t);
extern void AfficheFolderParam(uint8_t, uint8_t);
extern void AffichNomFic(uint8_t);        // (ArduinoOutStream, uint8_t)
extern uint8_t ReduceFolderPath_ChangeFolder(SdFat, uint8_t, SdFile);
extern void ChargementNomRep(uint8_t, uint8_t);
extern void FolderPathModification(uint8_t);
extern void ChangeAbsolutePath(uint8_t);
extern void DeleteFile(String, SdFat, ArduinoOutStream);
extern void DeleteFolder(String, SdFat, ArduinoOutStream);
extern void CreateDirectory(String, SdFat);
extern void ChangeDirectory(String, SdFat, SdFile);
extern void TestFolderExist(char *, SdFat);
extern void LectureFichier(String, SdFat, SdFile);
extern void AppendWrite(String, SdFat, SdFile);
extern void FolderFileOptionPath(String, uint8_t);
extern void Affiche_ligne_String(String);
extern void Affiche_ligne_Array(char *);
extern void InitArray2dimensions(char *, uint8_t, uint8_t);
extern uint8_t NbrCharInArray(char *);


#endif /* FONCTIONS_H_ */
