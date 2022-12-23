/**
 * Connects to SP200A SonarPhone as a first or second master after
 * first master has been established.
 * The SonarPhone phone app is not required unless SP200A is 
 * factory reset.
 * Numerical depth is transmitted on SeaTalk on pin 4.
 * SeaTalk protocol thanks to Thomas Knauf thomasknauf.de/seatalk.htm
 * Output on pin 4 drives a FET gate directly. FET drain connects to  
 * SeaTalk data, FET source to SeaTalk ground. 
 * 
 * - Jim McKeown
 * Last update 23 December 2022
 */

#include "WiFi.h"
#include "AsyncUDP.h"
#include <WiFiManager.h>

const char * ssid = "T-BOX-720";
const char * password = "";
uint8_t FC[] = {70,67,21,0,244,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // mac address, units, beam width, depth max/min, checksum not set
uint8_t FX[] = {70,88,21,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,179,0,0,0,0,0,0,0,0,0}; // 
const int wordLength = 11;
int seaTalkOut = 4;
int seaTalkIn = 16; // connect to pin 4
int busQuiet = 17; // monitor for troubleshooting  
int bitTime = 208; // 1/4800 bps = 208 uS
int idleTime = bitTime * 11;
float metersToFeet = 3.28084;

bool haveMac = false;
int depthMin = 0;
int depthMax = 0; 
int beamWidth = 0; // 8 = 20 deg, 2 = 40 deg
int dataCount = 0;
int depthUnits = 0; //0 = meters, 1 = feet
float depth = 0.0;
float depthFrac = 0.0;
float vBatt = 0.0;
float vBattFrac = 0.0;
int temp = 0.0;;
int loopCount = 0;
bool newData = false;
bool FCsent = false;

volatile bool busQuietFlag = false;

int sendBuffer[] = {1536,1028,1024,0,0};

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


AsyncUDP udp;

void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  busQuietFlag = true;
  digitalWrite(busQuiet, HIGH);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR inputChangeInterrupt() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  timerWrite(timer, 0);
  busQuietFlag = false;
  digitalWrite(busQuiet, LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
}


/*  Send contents of sendBuffer array on SeaTalk output.
 *  Verify same level on SeaTalk receive pin
 */  
 
bool sendSeaTalk()
{
  int currentByte = 0;
  int currentBit = 0;
  int sendBufferLen = sizeof sendBuffer / sizeof sendBuffer[0];
  float depthToSend = 0.0;

  // set depth units to feet - convert meters to feet
  if(depthUnits == 1)
  {
    // sendBuffer[2] = 1024;
    depthToSend = depth; 
  }
  else
  {
    // sendBuffer[2] = 1032; // bit 6 of Y set 1024 + 128 = 1152, 1024 + 8 = 1032
    depthToSend = depth * metersToFeet;
  }
  
  // calculate lower byte depth
  int intDepthX10 = depthToSend * 10; 
  sendBuffer[3] = 1024 + ((intDepthX10 % 256) * 2);

  // calculate upper byte depth
  sendBuffer[4] = 1024 + ((intDepthX10 / 256) * 2);
  
  for(int i = 0;i < sendBufferLen;i++)
  {
    currentByte = sendBuffer[i];
    for(int j = 0;j < wordLength;j++)
    {
        currentBit = currentByte % 2; // get lsb
        currentByte /= 2; // shift current byte right
        if(currentBit)
        {
          digitalWrite(seaTalkOut, LOW);
          if(digitalRead(seaTalkIn)) return false;
          delayMicroseconds(bitTime / 2);
          if(digitalRead(seaTalkIn)) return false;
          delayMicroseconds(bitTime / 2);
          if(digitalRead(seaTalkIn)) return false;
        }
        else
        {
          digitalWrite(seaTalkOut, HIGH);
          if(!digitalRead(seaTalkIn)) return false;
          delayMicroseconds(bitTime / 2);
          if(!digitalRead(seaTalkIn)) return false;
          delayMicroseconds(bitTime / 2);
          if(!digitalRead(seaTalkIn)) return false;
        }
    }
  }
  return true;
}

void setup()
{
    pinMode(seaTalkOut, OUTPUT);
    digitalWrite(seaTalkOut, 0);
    pinMode(seaTalkIn, INPUT);
    pinMode(busQuiet, OUTPUT);
    digitalWrite(busQuiet, 0);
    attachInterrupt(digitalPinToInterrupt(seaTalkIn), inputChangeInterrupt, CHANGE);
    
    Serial.begin(115200);

    // setup and enable busQuiet timer
    timer = timerBegin(0, 80, true); // timer 0, 80 prescaler, count up
    timerAttachInterrupt(timer, &onTimer, true); // pointer to timer, address of interrupt handler, edge
    timerAlarmWrite(timer, idleTime, true); // pointer to timer, interrupt value (11/4800 seconds), auto-reload
    timerAlarmEnable(timer);
    //timerStop(timer); // timer enabled but not started


    //WiFiManager intialization
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    //wm.resetSettings();
    bool res;
    res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    if(!res) 
    {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else 
    {
        Serial.println("Connected to WiFi");
    }    
    
    if(udp.connect(IPAddress(192,168,1,1), 5000)) 
    {
        Serial.println("UDP connected");
        //udp.write(FC, 29);
        udp.onPacket([](AsyncUDPPacket packet) {
            // check for REDYFX. Update FC request
            if(packet.data()[6] == 82 && packet.data()[7] == 69 && packet.data()[8] == 68 && packet.data()[9] == 89 && packet.data()[10] == 70 && packet.data()[11] == 88) // is this REDYFX?
            {
              // set FC mac address
              for(int i = 0;i < 6;i++)
              {
                FC[21 + i] = packet.data()[26 + i];
              }
              
              // set units
              FC[11] = depthUnits;              
              
              // set max, min depth (4 bytes)
              FC[6] = depthMin % 256; // lower byte
              FC[7] = depthMin / 256; // upper byte
              FC[8] = depthMax % 256; // lower byte
              FC[9] = depthMax / 256; // upper byte
              
              // set beam width
              FC[13] = beamWidth;
              
              // set checksum
              int checksum = 0;
              for(int i = 0;i < 19;i++)
              {
                checksum += FC[i];
              }
              FC[19] = checksum % 256; // lower byte
              FC[20] = checksum / 256; // upper byte
              haveMac = true;
            }
            // check for REDYFC. Set Units, Depth, Range, Temp, BatteryVolts, newData flag
            if(packet.data()[6] == 82 && packet.data()[7] == 69 && packet.data()[8] == 68 && packet.data()[9] == 89 && packet.data()[10] == 70 && packet.data()[11] == 67) // is this REDYFC?
            {
              depthUnits = packet.data()[21];
              depthFrac = packet.data()[25];
              depth = packet.data()[23] + (packet.data()[24] * 256) + (depthFrac / 100);
              depthMin = packet.data()[16] + (packet.data()[17] * 256);
              depthMax = packet.data()[18] + (packet.data()[19] * 256);
              beamWidth = packet.data()[32];
              vBattFrac = packet.data()[31];
              vBatt = packet.data()[30] + (vBattFrac / 100);
              temp = packet.data()[26];
              newData = true;
            }
            dataCount++;
            packet.flush();          
        });
        delay(1000);
    }

    
}

void loop()
{
    if(haveMac)
    {

      if(newData)
      {
        Serial.printf("Depth: %.1f ", depth);
        while(!busQuietFlag);
        if(!sendSeaTalk())
        {
          Serial.println("Error in sendSeaTalk()");
        }
        newData = false;
      }
      else
      {
        Serial.print("Depth: --.-- ");
      }
      
      
      if(FCsent)
      {
        Serial.println("-");
        FCsent = false;
      }
      else
      {
        Serial.println("");
      }
      
      if(loopCount > 19)
      {
        // update units
        FC[11] = depthUnits;
        
        // update max, min depth (4 bytes)
        FC[6] = depthMin % 256; // lower byte
        FC[7] = depthMin / 256; // upper byte
        FC[8] = depthMax % 256; // lower byte
        FC[9] = depthMax / 256; // upper byte
  
        // set beam width
        FC[13] = beamWidth;
        
        // update checksum
        int checksum = 0;
        for(int i = 0;i < 19;i++)
        {
          checksum += FC[i];
        }
        FC[19] = checksum % 256; // lower byte
        FC[20] = checksum / 256; // upper byte      
        
        dataCount = 0;
        udp.write(FC, sizeof FC);
        loopCount = 0;
        FCsent = true;
     }
      
      loopCount++;
      delay(500);
    }
    else
    {
      udp.write(FX, sizeof FX);
      delay(1000);
    }
}
