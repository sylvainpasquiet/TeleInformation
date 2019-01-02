#include "MyDHT.h"

MyDHT::MyDHT(uint8_t pin) 
{
  _pin = pin;
  //#ifdef __AVR
  _bit = digitalPinToBitMask(_pin);
  _port = digitalPinToPort(_pin);
  //#endif

  _ddr = portModeRegister(_port);   // Registre MODE (INPUT / OUTPUT)
  _out = portOutputRegister(_port); // Registre OUT (écriture)
  _in = portInputRegister(_port);   // Registre IN (lecture)  
}

void MyDHT::begin(void) 
{
  pinMode(_pin, INPUT_PULLUP);  
}

byte MyDHT::read(float* temperature, float* humidity)
{
    /* Lit le capteur */ 
  byte ret = readDHTxx(1, 1000);
  
  /* Détecte et retourne les erreurs de communication */
  //if (ret != DHT_SUCCESS) 
  //  return ret;
    
  /* Calcul la vraie valeur de la température et de l'humidité */
  float fh = _data[0];
  fh *= 256;
  fh += _data[1];
  fh *= 0.1;
  *humidity = fh;
 
  float ft = _data[2] & 0x7f;
  ft *= 256;
  ft += _data[3];
  ft *= 0.1;
  if (_data[2] & 0x80) {
    ft *= -1;
  }
  *temperature = ft;

  /* Ok */
  return ret;
}

byte MyDHT::readDHTxx(unsigned long start_time, unsigned long timeout) 
{
  _data[0] = _data[1] = _data[2] = _data[3] = _data[4] = 0;
  // start_time est en millisecondes
  // timeout est en microsecondes

  /* Conversion du temps de timeout en nombre de cycles processeur */
  unsigned long max_cycles = microsecondsToClockCycles(timeout);
 
  /* Réveil du capteur */
  *_ddr |= _bit;  // OUTPUT
  *_out &= ~_bit; // LOW
  delayMicroseconds(start_time*1000); // Temps d'attente à LOW causant le réveil du capteur
  // N.B. Il est impossible d'utilise delayMicroseconds() ici car un délai
  // de plus de 16 millisecondes ne donne pas un timing assez précis.
  
  /* Portion de code critique - pas d'interruptions possibles */
  noInterrupts();
  
  /* Passage en écoute */
  *_out |= _bit;  // PULLUP
  delayMicroseconds(40);
  *_ddr &= ~_bit; // INPUT
 
  /* Attente de la réponse du capteur */
  timeout = 0;
  while(!(*_in & _bit)) { /* Attente d'un état LOW */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }
    
  timeout = 0;
  while(*_in & _bit) { /* Attente d'un état HIGH */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }

  /* Lecture des données du capteur (40 bits) */
  for (byte i = 0; i < 40; ++i) {
 
    /* Attente d'un état LOW */
    unsigned long cycles_low = 0;
    while(!(*_in & _bit)) {
      if (++cycles_low == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    /* Attente d'un état HIGH */
    unsigned long cycles_high = 0;
    while(*_in & _bit) {
      if (++cycles_high == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }
    
    /* Si le temps haut est supérieur au temps bas c'est un "1", sinon c'est un "0" */
    _data[i / 8] <<= 1;
    if (cycles_high > cycles_low) {
      _data[i / 8] |= 1;
    }
  }
  
  /* Fin de la portion de code critique */
  interrupts();

  /* Evite les problèmes de pull-up */
  *_out |= _bit;  // PULLUP
  *_ddr &= ~_bit; // INPUT

  /*
   * Format des données :
   * [1, 0] = humidité en %
   * [3, 2] = température en degrés Celsius
   * [4] = checksum (humidité + température)
   */

  /* Vérifie la checksum */
  byte checksum = (_data[0] + _data[1] + _data[2] + _data[3]) & 0xff;
  if (_data[4] != checksum)
    return DHT_CHECKSUM_ERROR; /* Erreur de checksum */
  else
    return DHT_SUCCESS; /* Pas d'erreur */
}
