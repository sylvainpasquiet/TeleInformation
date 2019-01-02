#ifndef MYDHT_H
#define MYDHT_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/* Code d'erreur de la fonction readDHT11() et readDHT22() */
const byte DHT_SUCCESS = 0;        // Pas d'erreur
const byte DHT_TIMEOUT_ERROR = 1;  // Temps d'attente dépassé
const byte DHT_CHECKSUM_ERROR = 2; // Données reçues erronées

class MyDHT {
  public:
    MyDHT(uint8_t pin);
    void begin(void);
    byte read(float* temperature, float* humidity);
    
  private:
    uint8_t _pin;
    byte _data[5];
    uint8_t _bit, _port;
    volatile uint8_t* _ddr;   // Registre MODE (INPUT / OUTPUT)
    volatile uint8_t* _out; // Registre OUT (écriture)
    volatile uint8_t* _in;   // Registre IN (lecture)  
    
    byte readDHTxx(unsigned long start_time, unsigned long timeout);
};

#endif
