#define SERIAL_RX_BUFFER_SIZE 100

#define BUFFER_SIZE 50

struct TeleInfo_t {
  char ADCO[15];
  char OPTARIF[6];
  unsigned char ISOUSC;
  char PTEC[6];
  unsigned char IINST;
  unsigned char ADPS;
  unsigned char IMAX;
  unsigned short PAPP;
  unsigned long HC_HC;
  unsigned long HC_HP;
  char HHPHC[4];
};

struct DHT22_t {
  float Temp;
  float Hum;
};

struct Stat_t {
  unsigned short ErreurChecksum;
  unsigned short OldErreurChecksum;

  unsigned short Erreur;
  unsigned short OldErreur;

  unsigned short OldErreurTrameGrande;
  unsigned short ErreurTrameGrande;

  unsigned short OldErreurTramePetite;
  unsigned short ErreurTramePetite;

  char OldErreurTrame[30];
  char ErreurTrame[30];
};
 

#define MY_DISABLED_SERIAL

#define MY_NODE_ID 20

#define MY_RADIO_NRF24

#define MY_RF24_PA_LEVEL RF24_PA_MAX

#define MY_RF24_CE_PIN  9
#define MY_RF24_CS_PIN  10
#define MY_RF24_CHANNEL 125

#define MY_INCLUSION_MODE_FEATURE

#define MY_INCLUSION_MODE_DURATION 60
#define MY_DEFAULT_LED_BLINK_PERIOD 100
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
#define MY_DEFAULT_ERR_LED_PIN 6  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  7  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  8  // the PCB, on board LED

#include <MySensors.h>
#include "MyDHT.h"
#include "UpdateData.h"

#define DHTPIN_1  2         // Pin which is connected to the DHT sensor.
#define DHTPIN_2  3         // Pin which is connected to the DHT sensor.

DHT22_t gActualDHT22_1;
DHT22_t gActualDHT22_2;
DHT22_t gLastDHT22_1;
DHT22_t gLastDHT22_2;

TeleInfo_t gCurrentTI;
TeleInfo_t gLastTI; // dernière lecture téléinfo
Stat_t gStatistiques;

unsigned long gLastUpdate;
unsigned char gEtape;
unsigned long gOldTime;
  
MyMessage gMsg;

MyDHT gMyDHT1(DHTPIN_1);
MyDHT gMyDHT2(DHTPIN_2);

#define CHILD_ID_ADCO 0
#define CHILD_ID_OPTARIF 1
#define CHILD_ID_ISOUSC 2
#define CHILD_ID_PTEC 3
#define CHILD_ID_IINST 4
#define CHILD_ID_ADPS 5
#define CHILD_ID_IMAX 6
#define CHILD_ID_PAPP 7 
#define CHILD_ID_BASE 10
#define CHILD_ID_HC_HC 20
#define CHILD_ID_HC_HP 21
#define CHILD_ID_HHPHC 50
#define CHILD_ID_1_TEMPERTURE 100
#define CHILD_ID_1_HUMIDITY   101
#define CHILD_ID_2_TEMPERTURE 102
#define CHILD_ID_2_HUMIDITY   103
#define CHILD_ID_ERREUR_CS 200
#define CHILD_ID_ERREUR_MYSENSORS 201
#define CHILD_ID_ERREUR_TRAME_GRANDE 202
#define CHILD_ID_ERREUR_TRAME_PETITE 203
#define CHILD_ID_CMD_RESET_STATS  220
#define CHILD_ID_TRAME_ERREUR  230

UpdateData_Char UpdateData_ADCO(                 CHILD_ID_ADCO,                  V_TEXT);
UpdateData_Char UpdateData_OPTARIF(              CHILD_ID_OPTARIF,               V_TEXT);
UpdateDataChar  UpdateData_ISOUSC(               CHILD_ID_ISOUSC,                V_CURRENT);
UpdateData_Char UpdateData_PTEC(                 CHILD_ID_PTEC,                  V_TEXT);
UpdateDataChar  UpdateData_IINST(                CHILD_ID_IINST,                 V_CURRENT);
UpdateDataChar  UpdateData_ADPS(                 CHILD_ID_ADPS,                  V_CURRENT);
UpdateDataChar  UpdateData_IMAX(                 CHILD_ID_IMAX,                  V_CURRENT);
UpdateDataShort UpdateData_PAPP(                 CHILD_ID_PAPP,                  V_WATT);
UpdateDataLong  UpdateData_HC_HC(                CHILD_ID_HC_HC,                 V_KWH);
UpdateDataLong  UpdateData_HC_HP(                CHILD_ID_HC_HP,                 V_KWH);
UpdateData_Char UpdateData_HHPHC(                CHILD_ID_HHPHC,                 V_TEXT);
UpdateDataFloat UpdateData_1_TEMPERTURE(         CHILD_ID_1_TEMPERTURE,          V_TEMP);
UpdateDataFloat UpdateData_1_HUMIDITY(           CHILD_ID_1_HUMIDITY,            V_HUM);
UpdateDataFloat UpdateData_2_TEMPERTURE(         CHILD_ID_2_TEMPERTURE,          V_TEMP);
UpdateDataFloat UpdateData_2_HUMIDITY(           CHILD_ID_2_HUMIDITY,            V_HUM);
UpdateDataShort UpdateData_ERREUR_CS(            CHILD_ID_ERREUR_CS,             V_KWH);
UpdateDataShort UpdateData_ERREUR_MYSENSORS(     CHILD_ID_ERREUR_MYSENSORS,      V_KWH);
UpdateDataShort UpdateData_ERREUR_TRAME_GRANDE(  CHILD_ID_ERREUR_TRAME_GRANDE,   V_KWH);
UpdateDataShort UpdateData_ERREUR_TRAME_PETITE(  CHILD_ID_ERREUR_TRAME_PETITE,   V_KWH);
UpdateData_Char UpdateData_TRAME_ERREUR(         CHILD_ID_TRAME_ERREUR,          V_TEXT);

void Lecture(TeleInfo_t* const l_currentTI,Stat_t* const l_Statistiques)
{
  static char l_Buffer[BUFFER_SIZE]; 
  static unsigned char l_PositionBuffer;
  static char l_Etape;
  char l_Label[10];
  char l_Value[20];
  bool l_FirstFrame;
  unsigned long l_NbCaracteresRecus;
  unsigned long l_i;
  char l_c;
  unsigned char l_Checksum;
  unsigned char l_MonCS;
  unsigned short l_PuissanceInst;
  static float l_PuissanceMoy;

  l_NbCaracteresRecus=Serial.available();
  switch(l_Etape)
  {
    /**********************************************************
                          Purge
    ***********************************************************/
    case 0:
      memset(l_Buffer,'\0',BUFFER_SIZE);
      l_PositionBuffer=0;
      Serial.flush();
      l_Etape=10;
      Serial.write("Attente\n");
      break;
    /**********************************************************
                Attente caractère 0x02 début message
    ***********************************************************/      
    case 10:
      if (l_NbCaracteresRecus==0) return;
      for(l_i=0;l_i<l_NbCaracteresRecus;l_i++)
      {
        l_c = 0x7F & Serial.read();  
        if (l_c==0x02) 
        {
          l_Etape=20;
          Serial.write("Debut\n");
          break;
        }
      }    
      break;
    case 20:  
      if (l_NbCaracteresRecus==0) return;
      for(l_i=0;l_i<l_NbCaracteresRecus;l_i++)
      {
        l_c = 0x7F & Serial.read();
        //Serial.write(l_c);
        if ((l_c!=2)&&(l_c!=3)&&(l_c!=13)&&(l_c!=10)) 
        {
          l_Buffer[l_PositionBuffer]=l_c;
          l_PositionBuffer++;
          l_Buffer[l_PositionBuffer]=0;
        }
        if ((l_c==2)||(l_c==10)||(l_c==3)) 
        {
          memset(l_Buffer,'\0',BUFFER_SIZE);
          l_PositionBuffer=0;   
        }
        /*Trop de caractères => pas normal => on purge*/ 
        if (l_PositionBuffer>50) 
        {
          (/*l_Statistiques->ActualErreurTrameGrande)++;*/l_Statistiques->ErreurTrameGrande)++;
          l_Etape=0;
          Serial.write("Trame trop grande\0");
          break;
        }
        if (l_c==13) 
        {
          /*Pas assez de données dans cette trame => on purge la trame et on passe à la trame suivante même si pas normal*/
          if ((l_PositionBuffer)&&(l_PositionBuffer<3)) 
          {
            (/*l_Statistiques->ActualErreurTramePetite)++;*/l_Statistiques->ErreurTramePetite)++;
            memset(l_Buffer,'\0',BUFFER_SIZE);
            l_PositionBuffer=0;
            Serial.write("Trame trop petite");
            break;            
          }
          /*on récupère le label et la value*/
          sscanf(l_Buffer,"%s %s",l_Label,l_Value);
          /*on récupère le caractère de checksum*/
          l_Checksum=l_Buffer[l_PositionBuffer-1];
          /*on recalcul le checksum*/
          l_Buffer[l_PositionBuffer-2]='\0';
          l_MonCS=CalculChecksum(l_Buffer);
          /*Suivant si le checksum est ok ou non*/
          if (l_Checksum==l_MonCS)
          {
            if ( strcmp(l_Label,"ADCO\0"   )==0 ) sscanf(l_Value,"%s",l_currentTI->ADCO);
            if ( strcmp(l_Label,"OPTARIF\0")==0 ) sscanf(l_Value,"%2s",l_currentTI->OPTARIF);
            if ( strcmp(l_Label,"ISOUSC\0" )==0 ) sscanf(l_Value,"%hhu",&l_currentTI->ISOUSC);
            if ( strcmp(l_Label,"PTEC\0"   )==0 ) sscanf(l_Value,"%2s",l_currentTI->PTEC);
            if ( strcmp(l_Label,"IINST\0"  )==0 ) sscanf(l_Value,"%hhu",&l_currentTI->IINST);
            if ( strcmp(l_Label,"ADPS\0"   )==0 ) sscanf(l_Value,"%hhu",&l_currentTI->ADPS);
            if ( strcmp(l_Label,"IMAX\0"   )==0 ) sscanf(l_Value,"%hhu",&l_currentTI->IMAX);
            if ( strcmp(l_Label,"PAPP\0"   )==0 ) sscanf(l_Value,"%hu",&l_currentTI->PAPP);
            if ( strcmp(l_Label,"HCHC\0"   )==0 ) sscanf(l_Value,"%lu",&l_currentTI->HC_HC);
            if ( strcmp(l_Label,"HCHP\0"   )==0 ) sscanf(l_Value,"%lu",&l_currentTI->HC_HP);
            if ( strcmp(l_Label,"HHPHC\0"  )==0 ) sscanf(l_Value,"%s",l_currentTI->HHPHC);
          }
          else
          {
            /*(l_Statistiques->ActualErreurChecksum)++;*/(l_Statistiques->ErreurChecksum)++;
            strcpy(gStatistiques.ErreurTrame,l_Buffer);
            Serial.print(l_Buffer);
            Serial.print(" ");
            Serial.write(l_Checksum);
            Serial.write(" Erreur Checksum: \0");
            Serial.print(l_Checksum);
            Serial.write(" ");
            Serial.println(l_MonCS);
          }
          /*on vient de traiter une trame complète => on sort*/
          memset(l_Buffer,'\0',BUFFER_SIZE);
          l_PositionBuffer=0;       
          break;
        }
      }       
      break;           
  }
}

char CalculChecksum(const char* const l_Frame)
{
  char l_CS = 0;
  char l_i;
  char l_longueur;
  l_longueur=strlen(l_Frame);
  for(l_i=0 ; l_i < l_longueur ; l_i++) l_CS +=(int) l_Frame[l_i];
  l_CS = (l_CS & 0x3F) + 0x20;
  return(l_CS);
}

void ResetStatistiques()
{
  gStatistiques.ErreurTrameGrande=0;
  gStatistiques.ErreurTramePetite=0;
  gStatistiques.ErreurChecksum=0;
  gStatistiques.Erreur=0; 
  gStatistiques.OldErreurTrameGrande=99;
  gStatistiques.OldErreurTramePetite=99;
  gStatistiques.OldErreurChecksum=99;
  gStatistiques.OldErreur=99; 
  strcpy(gStatistiques.ErreurTrame,"\0");
  strcpy(gStatistiques.OldErreurTrame,"Old\0");
}
 
void before()
{
  ResetStatistiques();
}

void presentation()
{
  sendSketchInfo(F("Informations Garage\0"), F("2.20\0")); 
  UpdateData_PAPP.Present(S_POWER);    
  UpdateData_ADPS.Present(S_POWER);
  UpdateData_HC_HC.Present(S_POWER); 
  UpdateData_HC_HP.Present(S_POWER); 
  UpdateData_IINST.Present(S_POWER);  
  UpdateData_ADCO.Present(S_INFO);   
  UpdateData_OPTARIF.Present(S_INFO);   
  UpdateData_ISOUSC.Present(S_POWER);   
  UpdateData_PTEC.Present(S_INFO);
  UpdateData_IMAX.Present(S_POWER);  
  UpdateData_HHPHC.Present(S_INFO);
  UpdateData_1_TEMPERTURE.Present(S_TEMP);            
  UpdateData_1_HUMIDITY.Present(S_HUM);                   
  UpdateData_2_TEMPERTURE.Present(S_TEMP);           
  UpdateData_2_HUMIDITY.Present(S_HUM);       
  UpdateData_ERREUR_CS.Present(S_POWER);   
  UpdateData_ERREUR_MYSENSORS.Present(S_POWER);     
  UpdateData_ERREUR_TRAME_GRANDE.Present(S_POWER);   
  UpdateData_ERREUR_TRAME_PETITE.Present(S_POWER);  
  UpdateData_TRAME_ERREUR.Present(S_INFO);     
  present(CHILD_ID_CMD_RESET_STATS, S_INFO); 
}

void setup()
{
  gMyDHT1.begin();
  gMyDHT2.begin();
  memset(&gCurrentTI,0,sizeof(gCurrentTI)); 
  memset(&gLastTI,0,sizeof(gLastTI));
  //Serial.begin(1200,SERIAL_7E1);
  Serial.begin(1200,SERIAL_8N1);
}

void loop()
{
  float Temp;
  float hum;  
  unsigned long ActualTime = millis();
  Lecture(&gCurrentTI,&gStatistiques);

  switch (gEtape)
  {     
    case 0:
      gMyDHT1.read(&gActualDHT22_1.Temp,&gActualDHT22_1.Hum);
      gMyDHT2.read(&gActualDHT22_2.Temp,&gActualDHT22_2.Hum);
      gOldTime=millis();
      gEtape=250;
      break;   
      
    case 100:
      if (DHT_SUCCESS==gMyDHT1.read(&Temp,&hum))
      {
        gActualDHT22_1.Temp  =(19.0 * gActualDHT22_1.Temp + Temp)/20;
        gActualDHT22_1.Hum  =(19.0 * gActualDHT22_1.Hum + hum)/20;
      }
      if (DHT_SUCCESS==gMyDHT2.read(&Temp,&hum))
      {
        gActualDHT22_2.Temp  =(19.0 * gActualDHT22_2.Temp + Temp)/20;
        gActualDHT22_2.Hum  =(19.0 * gActualDHT22_2.Hum + hum)/20;  
      }
      gOldTime=ActualTime;
      gEtape=250;
      break;  
            
    case 250:
      if (((ActualTime - gOldTime) > 2000)&&((ActualTime - gLastUpdate)>200)&&((ActualTime - gLastUpdate)>300))  gEtape=100;
      break;
  }

  UpdateData_PAPP.Update(                  gCurrentTI.PAPP                  ,&gLastTI.PAPP                        ,&gLastUpdate,1500,1000,5000,30);  
  UpdateData_ADPS.Update(                  gCurrentTI.ADPS                  ,&gLastTI.ADPS                        ,&gLastUpdate,1000,1);  
  UpdateData_HC_HC.Update(                 gCurrentTI.HC_HC                 ,&gLastTI.HC_HC                       ,&gLastUpdate,(unsigned long)7000,(unsigned long)1);       
  UpdateData_HC_HP.Update(                 gCurrentTI.HC_HP                 ,&gLastTI.HC_HP                       ,&gLastUpdate,(unsigned long)8000,(unsigned long)1);
  UpdateData_IINST.Update(                 gCurrentTI.IINST                 ,&gLastTI.IINST                       ,&gLastUpdate,5000,1);    
  UpdateData_ADCO.Update(                  gCurrentTI.ADCO                  ,gLastTI.ADCO                         ,&gLastUpdate);                
  UpdateData_OPTARIF.Update(               gCurrentTI.OPTARIF               ,gLastTI.OPTARIF                      ,&gLastUpdate);               
  UpdateData_ISOUSC.Update(                gCurrentTI.ISOUSC                ,&gLastTI.ISOUSC                      ,&gLastUpdate);                
  UpdateData_PTEC.Update(                  gCurrentTI.PTEC                  ,gLastTI.PTEC                         ,&gLastUpdate);  
  UpdateData_IMAX.Update(                  gCurrentTI.IMAX                  ,&gLastTI.IMAX                        ,&gLastUpdate);     
  UpdateData_HHPHC.Update(                 gCurrentTI.HHPHC                 ,gLastTI.HHPHC                        ,&gLastUpdate);     
  UpdateData_1_TEMPERTURE.Update(          gActualDHT22_1.Temp              ,&gLastDHT22_1.Temp                   ,&gLastUpdate,11000,0.5);              
  UpdateData_1_HUMIDITY.Update(            gActualDHT22_1.Hum               ,&gLastDHT22_1.Hum                    ,&gLastUpdate,12000,5.0);                   
  UpdateData_2_TEMPERTURE.Update(          gActualDHT22_2.Temp              ,&gLastDHT22_2.Temp                   ,&gLastUpdate,13000,0.5);                 
  UpdateData_2_HUMIDITY.Update(            gActualDHT22_2.Hum               ,&gLastDHT22_2.Hum                    ,&gLastUpdate,14000,5.0);           
  UpdateData_ERREUR_CS.Update(             gStatistiques.ErreurChecksum     ,&gStatistiques.OldErreurChecksum     ,&gLastUpdate);   
  UpdateData_ERREUR_MYSENSORS.Update(      gStatistiques.Erreur             ,&gStatistiques.OldErreur             ,&gLastUpdate);   
  UpdateData_ERREUR_TRAME_GRANDE.Update(   gStatistiques.ErreurTrameGrande  ,&gStatistiques.OldErreurTrameGrande  ,&gLastUpdate);   
  UpdateData_ERREUR_TRAME_PETITE.Update(   gStatistiques.ErreurTramePetite  ,&gStatistiques.OldErreurTramePetite  ,&gLastUpdate);  
  UpdateData_TRAME_ERREUR.Update(          gStatistiques.ErreurTrame        ,gStatistiques.OldErreurTrame         ,&gLastUpdate);  
  //wait(50);           
} 

void receive(const MyMessage &l_message)
{
  if ((l_message.type==V_TEXT)&&(l_message.sensor==CHILD_ID_CMD_RESET_STATS))
  {
    ResetStatistiques(); 
  }
}
