#include "UpdateData.h"

UpdateDataFloat::UpdateDataFloat(unsigned char Child,unsigned char Type)
{
  _Child            = Child;
  _Type             = Type;
}

UpdateDataChar::UpdateDataChar(unsigned char Child,unsigned char Type)
{
  _Child            = Child;
  _Type             = Type;
}

UpdateDataShort::UpdateDataShort(unsigned char Child,unsigned char Type)
{
  _Child            = Child;
  _Type             = Type;
}

UpdateDataLong::UpdateDataLong(unsigned char Child,unsigned char Type)
{
  _Child            = Child;
  _Type             = Type;
}

UpdateData_Char::UpdateData_Char(unsigned char Child,unsigned char Type)
{
  _Child            = Child;
  _Type             = Type;
}

void UpdateDataFloat::Update(float l_Actual,float* l_Last,unsigned long* _LastGlobalRequest)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (fabs(l_Actual-*l_Last)>=0.01)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set((float)l_Actual,1),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }     
  }
  return(true);
}

void UpdateDataFloat::Update(float l_Actual,float* l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,float _Delta)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (fabs(l_Actual-*l_Last)>=0.01)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set((float)l_Actual,1),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }  
    if (Time - _LastUpdate > _Minimum)
    {
      if (fabs(l_Actual-*l_Last) >= _Delta)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set((float)l_Actual,1),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }     
  }
  return(true);
}

void UpdateDataChar::Update(unsigned char l_Actual,unsigned char* l_Last,unsigned long* _LastGlobalRequest)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (abs(l_Actual-*l_Last)>0)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }      
  }
  return(true);
}
void UpdateDataChar::Update(unsigned char   l_Actual,unsigned char*  l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned char _Delta)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {  
    if (Time - _LastUpdate > INTERVAL_DATA_MAXI)
    {
      if ((strlen(l_Actual)!=strlen(l_Last))||(strcmp(l_Actual,l_Last)!=0))              
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    } 
    if (Time-_LastUpdate > _Minimum)
    {
      if (abs(l_Actual-*l_Last)>=_Delta)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }       
  }
  return(true);
}

void UpdateDataShort::Update(unsigned short l_Actual,unsigned short* l_Last,unsigned long* _LastGlobalRequest)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (abs(l_Actual-*l_Last)>0)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))   
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }    
  }
  return(true);
}
void UpdateDataShort::Update(unsigned short l_Actual,unsigned short* l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned short _Delta)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (abs(l_Actual-*l_Last)>0)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }  
    if (Time - _LastUpdate > _Minimum)
    {
      if (abs(l_Actual-*l_Last) > _Delta)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }     
  }
  return(true);
}
void UpdateDataShort::Update(unsigned short l_Actual,unsigned short* l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned short _Delta,unsigned long _Minimum2,unsigned short _Delta2)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (abs(l_Actual-*l_Last)>0)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }  
    if (Time - _LastUpdate > _Minimum)
    {
      if (abs(l_Actual-*l_Last) > _Delta)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }    
    if (Time - _LastUpdate > _Minimum2)
    {
      if (abs(l_Actual-*l_Last) > _Delta2)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }      
  }
  return(true);
}

void UpdateDataLong::Update(unsigned long l_Actual,unsigned long* l_Last,unsigned long* _LastGlobalRequest)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (abs(l_Actual-*l_Last)>0)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }   
  }
  return(true);
}
void UpdateDataLong::Update(unsigned long  l_Actual,unsigned long* l_Last,unsigned long* _LastGlobalRequest,unsigned long _Minimum,unsigned long _Delta)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {
    if (Time-_LastUpdate > INTERVAL_DATA_MAXI)
    {
      if (abs(l_Actual-*l_Last)>0)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    } 
    if (Time-_LastUpdate > _Minimum)
    {
      if (abs(l_Actual-*l_Last)>=_Delta)                 
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            *l_Last=l_Actual;
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }   
  }
  return(true);
}


void UpdateData_Char::Update(char* l_Actual,char*  l_Last,unsigned long* _LastGlobalRequest)
{
  unsigned long Time;
  
  Time=millis();
  
  if (Time-*_LastGlobalRequest > INTERVAL_TRAME)
  {  
    if (Time - _LastUpdate > INTERVAL_DATA_MAXI)
    {
      if ((strlen(l_Actual)!=strlen(l_Last))||(strcmp(l_Actual,l_Last)!=0))              
      {
          if (send(_Msg.setSensor(_Child).setType(_Type).set(l_Actual),true))
          {
            strcpy(l_Last,l_Actual);
            _LastUpdate=Time;
          }
          *_LastGlobalRequest=Time;
      }
    }     
  }
  return(true);
}

void UpdateDataFloat::Present(unsigned char Type)
{
  present( _Child, Type);
  wait(100);
}

void UpdateDataChar::Present(unsigned char Type)
{
  present( _Child, Type);
  wait(100);
}

void UpdateDataShort::Present(unsigned char Type)
{
  present( _Child, Type);
  wait(100);
}

void UpdateDataLong::Present(unsigned char Type)
{
  present( _Child, Type);
  wait(100);
}

void UpdateData_Char::Present(unsigned char Type)
{
  present( _Child, Type);
  wait(100);
}
