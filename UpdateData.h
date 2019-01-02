#ifndef UPDATADATA_H
#define UPDATADATA_H
  #define INTERVAL_TRAME 700
  #define INTERVAL_DATA_MAXI  30000
    
  #if ARDUINO >= 100
   #include "Arduino.h"
  #else
   #include "WProgram.h"
  #endif
  
  #include <core/MySensorsCore.h>

  class UpdateDataFloat 
  {
    public:        
      UpdateDataFloat(unsigned char Child,unsigned char Type);   
      void Update(float           l_Actual,float*          l_Last,unsigned long* _LastGlobalRequest);
      void Update(float           l_Actual,float*          l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,float _Delta);
      void Present(unsigned char Type);
      
    private:
      unsigned long _LastUpdate;

      MyMessage _Msg;
      
      unsigned char _Child;
      unsigned char _Type;
  };

  class UpdateDataChar
  {
    public:        
      UpdateDataChar(unsigned char Child,unsigned char Type);
      void Update(unsigned char   l_Actual,unsigned char*  l_Last,unsigned long* _LastGlobalRequest);
      void Update(unsigned char   l_Actual,unsigned char*  l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned char _Delta);
      void Present(unsigned char Type);
      
    private:
      unsigned long _LastUpdate;

      MyMessage _Msg;
      
      unsigned char _Child;
      unsigned char _Type;
  };

  class UpdateDataShort 
  {
    public:        
      UpdateDataShort(unsigned char Child,unsigned char Type);
      void Update(unsigned short  l_Actual,unsigned short* l_Last,unsigned long* _LastGlobalRequest);
      void Update(unsigned short  l_Actual,unsigned short* l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned short _Delta);
      void Update(unsigned short  l_Actual,unsigned short* l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned short _Delta,unsigned long _Minimum2,unsigned short _Delta2);
      void Present(unsigned char Type);
      
    private:
      unsigned long _LastUpdate;

      MyMessage _Msg;
      
      unsigned char _Child;
      unsigned char _Type;
  };

  class UpdateDataLong
  {
    public:        
      UpdateDataLong(unsigned char Child,unsigned char Type);
      void Update(unsigned long   l_Actual,unsigned long*  l_Last,unsigned long* _LastGlobalRequest);
      void Update(unsigned long   l_Actual,unsigned long*  l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned long _Delta);
      void Present(unsigned char Type);
      
    private:
      unsigned long _LastUpdate;

      MyMessage _Msg;
      
      unsigned char _Child;
      unsigned char _Type; 
  };

  class UpdateData_Char 
  {
    public:        
      UpdateData_Char(unsigned char Child,unsigned char Type);
      void Update(char* l_Actual,char* l_Last,unsigned long* _LastGlobalRequest);
      void Present(unsigned char Type);
      
    private:
      unsigned long _LastUpdate;

      MyMessage _Msg;
      
      unsigned char _Child;
      unsigned char _Type;
  };

#endif
