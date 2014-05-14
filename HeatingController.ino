/** riban heating controller
*   This code provides a central heating controller.
*   It is designed to run on AVR ATMega328 microcontroller, specifically Arduino MiniPro.
*
*   Dallas one-wire network of senosrs connected to (Arduino) pin defined by g_nOneWire
*   Heating pump relay controlled from (Arduino) pin defined by g_nPump
*   Boiler relay contolled from (Arduino) pin defined by g_nBoiler
*   DS1307 Real Time Clock I2C buss connected to (Arduino) A4 (SDA) & A5 (SCL)
*
*   Libraries used:
*       Wire - I2C interface library
*           TWI/I2C library for Arduino & Wiring  Copyright (c) 2006 Nicholas Zambetti.  All right reserved. GLPL 2.1
*       OneWire - Dallas one wire protocol interface library
*           Copyright (c) 2007, Jim Studt  (original old version - many contributors since)
*           CRC code Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
*       EEPROM - Access to MCU EEPROM
*            Copyright (c) 2006 David A. Mellis.  All right reserved. GLPL
*       ribanTimer - Timer functions without 32-bit roll-over issues
*           Copyright (c) 2014, Brian Walton. All rights reserved. GLPL.
*
*/

#define _DEBUG_

#include <Wire.h>
#include <OneWire.h>
#include <EEPROM.h>
#include <ribanTimer.h>

const unsigned int MAX_SENSORS = 10;
const unsigned int MAX_EVENTS = 100;
const unsigned int MAX_SERIAL = 30;
const int DS1307_I2C_ADDRESS = 0x68;
const unsigned int EEPROM_SENSOR_START = 0;
const unsigned int EEPROM_SENSOR_SIZE = 10;
const unsigned int EEPROM_ZONE_START = 100;
const unsigned int EEPROM_ZONE_SIZE = 2;
const unsigned int EEPROM_EVENT_START = 200;
const unsigned int EEPROM_EVENT_SIZE = 10;

int g_nPump = 8;
int g_nBoiler = 9;
int g_nOneWire = 7;
int g_nButton = 2;
unsigned int g_nSensorQuant;
unsigned int g_nEventQuant;
OneWire ds(g_nOneWire);
byte g_bufferInput[MAX_SERIAL];
byte g_nCursorInput;

struct timestamp
{
    int nTime; //Minutes since 00:00
    byte nDay; //Bitwise flag of day. 1 = Sunday
};

struct sensor
{
    byte address[8]; //UID
    int nValue; //Current value (C/100)
    byte nZone; //Zone this sensor measures or contributes to
};

struct event
{
    unsigned int nTime; //Seconds since 00:00:00 Sunday
    byte nDay; //Bitwise flag of which days of week. LSB = Sunday
    byte nZone; //Zone this event relates to
    int nValue; //Temperature trigger value
};

struct zone
{
    int nSetpoint; //Temperature set-point (C/10)
    byte nHyst; //Hysteresis value (C/10)
    bool bOn; //True if calling for heat
    bool bSpace; //True if space heating zone (room, not water cylinder, requires pump)
};

timestamp g_tsNow; //Current time
timestamp g_tsNextEvent; //Number of minutes since 00:00 Sunday of next event
sensor g_sensors[MAX_SENSORS]; //reserve space for maximum number of sensors
event g_events[MAX_EVENTS]; //reserve space for maximum number of events
zone g_zones[10]; //Current temperature set-point for each zone

Timer timerMinute; //Instantiate a timer to find minute boundaries

/** @brief  Initialisation */
void setup()
{
    // initialize the digital pin as an output.
    pinMode(g_nBoiler, OUTPUT);
    pinMode(g_nPump, OUTPUT);
    pinMode(g_nButton, INPUT_PULLUP);
    Wire.begin();
    Serial.begin(9600);
    g_tsNextEvent.nDay = 0;
    g_tsNextEvent.nTime = 0;
    ReadConfig();
    timerMinute.start(1000, true); //start minute timer to trigger on first second to start minute sync promptly
    Serial.println("Initialised");
}

/** Main program loop */
void loop()
{
    if(timerMinute.IsTriggered())
    {
        //Update clock
        getTime(true);
        if(g_tsNextEvent.nTime == g_tsNow.nTime && (g_tsNextEvent.nDay & g_tsNow.nDay))
            ProcessEvents();
        //Update temperature readings
        bool bPump = false;
        bool bBoiler = false;
        for(unsigned int nSensor = 0; nSensor < g_nSensorQuant; nSensor++)
        {
            GetTemperature(nSensor);
            if(g_zones[g_sensors[nSensor].nZone].nSetpoint < g_sensors[nSensor].nValue)
                g_zones[g_sensors[nSensor].nZone].bOn = false; //Gone over setpoint

            if(g_zones[g_sensors[nSensor].nZone].nSetpoint - g_zones[g_sensors[nSensor].nZone].nHyst > g_sensors[nSensor].nValue / 10)
                g_zones[g_sensors[nSensor].nZone].bOn = true; //Gone below hysteresis point
            bBoiler |= g_zones[g_sensors[nSensor].nZone].bOn; //Contributes to call for heat
            if(g_zones[g_sensors[nSensor].nZone].bSpace)
                bPump |= g_zones[g_sensors[nSensor].nZone].bOn; //Contributes to call for heat (not zone 0 - water sensor)
        }

        digitalWrite(g_nBoiler, bBoiler);
        digitalWrite(g_nPump, bPump);

        unsigned int nSecs = 60 - getTime(false); //!@todo Is there a way to avoid calling getTime twice? First is to feed this minute's processing. Second is to estimate next minute boundary
        timerMinute.start((nSecs) * 1000, true); //Restart timer to hit next minute boundary
    }

    if(Serial.available())
        ReadSerial();

    if(!digitalRead(g_nButton))
    {
        //Button pressed
        //!@todo Debounce button.
        //!@todo Toggle current heating state
        //!@todo Add another buttong for water?
    }
}

/** Reads configuration from EEPROM
    Slot 0 = Number of sensors
    Slots 10-99 temperature sensor configuration (10 slots per sensor):
      Offset  Use
      0-7     UID (Set first byte to zero to clear sensor configuration)
      8       Zone
    Slots 100 - 1099 event configuration (10 slots per event):
      Offset  Use
      0       Day of week (Set to zero to disable event)
      1-2     Timestamp
      3       Zone
      4-5     Temperature value
    Slots 1100 - 1119 zone configuration (2 slots per zone)
      Offset  Use
      0       Hysteresis (C*10 below setpoint to turn off)
      1       Space (True if space heating. False if water heating. Not sure if this is used! Maybe for toggling heat / water?)
*/
void ReadConfig()
{
    Serial.println("Reading configuration...");
    //Get sensor configuration
    for(g_nSensorQuant = 0; g_nSensorQuant < MAX_SENSORS; g_nSensorQuant++)
    {
        if(EEPROM.read(g_nSensorQuant * EEPROM_SENSOR_SIZE + EEPROM_SENSOR_START) == 0)
            break; //Sensor not configured
        for(unsigned int i = 0; i < 8; ++i)
        {
            g_sensors[g_nSensorQuant].address[i] = EEPROM.read(g_nSensorQuant * EEPROM_SENSOR_SIZE + EEPROM_SENSOR_START + i);
        }
        g_sensors[g_nSensorQuant].nZone = EEPROM.read(g_nSensorQuant * EEPROM_SENSOR_SIZE + EEPROM_SENSOR_START + 8);
    }
    Serial.print(g_nSensorQuant);
    Serial.println(" sensors configured");

    //Get event configuration
    for(g_nEventQuant = 0; g_nEventQuant < MAX_EVENTS; g_nEventQuant++)
    {
        g_events[g_nEventQuant].nDay = EEPROM.read(g_nEventQuant * EEPROM_EVENT_SIZE + EEPROM_EVENT_START);
        if(0 == g_events[g_nEventQuant].nDay)
            break; //Event not configured and expect all events to be stored contiguously
        g_events[g_nEventQuant].nTime = EEPROM.read(g_nEventQuant * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 1) << 8 + EEPROM.read(g_nEventQuant * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 2);
        g_events[g_nEventQuant].nZone = EEPROM.read(g_nEventQuant * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 3);
        g_events[g_nEventQuant].nValue = EEPROM.read(g_nEventQuant * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 4) << 8 + EEPROM.read(g_nEventQuant * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 5);
    }
    Serial.print(g_nEventQuant);
    Serial.println(" events configured");

    for(unsigned int nZone = 0; nZone < 10; nZone++)
    {
        g_zones[nZone].nHyst = EEPROM.read(nZone * EEPROM_ZONE_SIZE + EEPROM_ZONE_START);
        g_zones[nZone].bSpace = (EEPROM.read(nZone * EEPROM_ZONE_SIZE + EEPROM_ZONE_START + 1) == 1);
    }
}

/** @brief  Reads input from serial port
*   @note   Read up to MAX_SERIAL (30) characters from serial port terminated with any combination of <CR> & <LF>
*   @note   Discards all input if buffer fills
*/
bool ReadSerial()
{
    while(g_nCursorInput < MAX_SERIAL && Serial.available())
    {
        g_bufferInput[g_nCursorInput] = Serial.read();
        if(g_bufferInput[g_nCursorInput] == 10 || g_bufferInput[g_nCursorInput] == 13)
        {
            //eol
            ParseSerial();
            g_nCursorInput = 0;
            return true;
        }
        ++g_nCursorInput;
    }
    if(g_nCursorInput >= 20)
        g_nCursorInput = 0; //buffer full so dispose of current message
    return false;
}

/** @brief  Parses serial data
*   @note   Uses global data buffer g_bufferInput
*/
void ParseSerial()
{
    switch (g_bufferInput[0])
    {
    case 'S':
        //Sensor config
        if(g_nCursorInput >= 20)
        {
            //Format S aaaaaaaaaaaaaaaa b
            // aaaaaaaaaaaaaaaa = sensor UID in hexadecimal
            // b = sensor zone
            byte data[8];
            for(unsigned int i = 0; i < 8; i++)
            {
                byte nValue = g_bufferInput[2 + i * 2] - 48;
                if(nValue > 9)
                    nValue -= 7;
                byte nData = nValue * 16;
                nValue = g_bufferInput[3 + i * 2] - 48;
                if(nValue > 9)
                    nValue -= 7;
                data[i] = nData + nValue;
            }
            AddSensor(data, *(g_bufferInput + 19) - 48);
        }
        else
        {
            //List sensors
            Serial.print("List sensors - quantity=");
            Serial.println(g_nSensorQuant);
            for(unsigned int nSensor = 0; nSensor < g_nSensorQuant; ++nSensor)
            {
                Serial.print("Sensor [");
                for(unsigned int i = 0; i < 8; i++)
                {
                    if(g_sensors[nSensor].address[i] < 0x10)
                        Serial.print("0");
                    Serial.print(g_sensors[nSensor].address[i], HEX);
                }
                Serial.print("] Zone ");
                Serial.print(g_sensors[nSensor].nZone);
                Serial.print(". Temp=");
                float fTemp = g_sensors[nSensor].nValue / 100;
                Serial.print(fTemp);
                Serial.println("C");
            }
        }
        break;
    case 'E':
        //Event
        /*
            "E" List
            "E aa" Show event aa NOT IMPLEMENTED
            "E aa 0" Delete event aa NOT IMPLEMENTED
            "E aa b hh:mm z +vvv" Add / modify event aa for day b (bitwise flag), time hh:mm, zone z, value +/-vvv
            Event index (aa) is 0 - 99
            Zone (z) is 0 - 9
        */
        if(g_nCursorInput >= 6) //!@todo check size
        {
            byte nEvent = (g_bufferInput[2] - 48) * 10 + g_bufferInput[3] - 48;
            if(nEvent > MAX_EVENTS)
                return;
            g_events[nEvent].nDay = g_bufferInput[5];
            if(0 == g_events[nEvent].nDay)
            {
                DeleteEvent(nEvent);
                return;
            }
            if(g_nCursorInput < 19)
                return;
            g_events[nEvent].nTime = (g_bufferInput[7] - 48) * 600 + (g_bufferInput[8] - 48) * 60 + (g_bufferInput[10] - 48) * 10 + g_bufferInput[11] - 48;
            g_events[nEvent].nZone = g_bufferInput[13] - 48;
            int nValue = (g_bufferInput[16] - 48) * 100 + (g_bufferInput[17] - 48) * 10 + g_bufferInput[18] - 48;
            if(g_bufferInput[15] == '-')
                nValue = -nValue;
            g_events[nEvent].nValue = nValue;
            SaveEvent(nEvent);
            ProcessEvents();
        }
        else
        {
            Serial.print("List events - quantity=");
            Serial.println(g_nEventQuant);
            for(unsigned int nEvent = 0; nEvent < g_nEventQuant; nEvent++)
            {
                //!@todo print timestamp in human readable form
                Serial.print(g_events[nEvent].nDay);
                Serial.print(" ");
                Serial.print(g_events[nEvent].nTime);
                Serial.print("  ");
                Serial.print(g_events[nEvent].nZone);
                Serial.print("  ");
                Serial.println(float(g_events[nEvent].nValue)/10);
            }
            Serial.print("Next event at ");
            Serial.print(g_tsNextEvent.nDay);
            Serial.print(" ");
            Serial.println(g_tsNextEvent.nTime);
        }
        break;
    case('Z'):
        //Zone
        if(g_nCursorInput >= 8)
        {
            byte nZone = (g_bufferInput[2] - 48);
            if(nZone > 9)
                return; //Only handle zones 0 - 9
            g_zones[nZone].nHyst = (g_bufferInput[4] - 48) * 10;
            g_zones[nZone].nHyst += (g_bufferInput[5] - 48);
            g_zones[nZone].bSpace = (g_bufferInput[7] != 48);
            SaveZone(nZone);
        }
        else
        {
            Serial.println("List zones");
            for (unsigned int nZone = 0; nZone < 10; nZone++)
            {
                Serial.print(nZone);
                Serial.print("  ");
                Serial.print(float(g_zones[nZone].nSetpoint) / 10);
                Serial.print("C Hyst=");
                Serial.print(float(g_zones[nZone].nHyst) / 10);
                Serial.print(g_zones[nZone].bSpace?" Space ":" Water ");
                Serial.println(g_zones[nZone].bOn?" On ":" Off ");
            }
        }
        //Zz aa b - Configure zone z=zone, a=hysteresis (C/10), b=1 for space heating
        break;
    case 'T':
        //Time
        //T hh:mm:ss a dd-mm-yy (a=day of week, 1 = Sunday. Date optional - sets just time. seconds optional, sets seconds to zero. Must provide seconds if setting date.)
        if(g_nCursorInput >= 7)
        {
            //time
            unsigned int nHour = 10 * (g_bufferInput[2] - 48);
            nHour += g_bufferInput[3] - 48;
            unsigned int nMinute = 10 * (g_bufferInput[5] - 48);
            nMinute += g_bufferInput[6] - 48;
            unsigned int nSecond = 0;
            if(g_nCursorInput < 10)
                return;
            //seconds
            nSecond = 10 * (g_bufferInput[8] - 48);
            nSecond += g_bufferInput[9] - 48;
            setTime(nHour, nMinute, nSecond);
            if(g_nCursorInput < 21)
                return;
            //date
            unsigned int nDow = g_bufferInput[11] - 48;
            unsigned int nDay = 10 * (g_bufferInput[13] - 48);
            nDay += g_bufferInput[14] - 48;
            unsigned int nMonth = 10 * (g_bufferInput[16] - 48);
            nMonth += g_bufferInput[17] - 48;
            unsigned int nYear = 10 * (g_bufferInput[19] - 48);
            nYear += g_bufferInput[20] - 48;
            setDate(nDow, nDay, nMonth, nYear);
        }
        getTime(true);
        break;
    case 'C':
        //Clear
        if(g_nCursorInput < 2)
            return;
        if(g_bufferInput[1] == 'S')
        {
            //Clear all sensors
            Serial.println("Clear all sensors");
            g_nSensorQuant = 0;
            for(unsigned int i = 0; i < MAX_SENSORS; i++)
                EEPROM.write(i * EEPROM_SENSOR_SIZE + EEPROM_SENSOR_START, 0);
        }
        else if(g_bufferInput[1] == 'E')
        {
            //Clear events
            Serial.println("Clear all events");
            g_nEventQuant = 0;
            g_tsNextEvent.nTime = 0;
            for(unsigned int i = 0; i < MAX_EVENTS; i++)
                EEPROM.write(i * EEPROM_EVENT_SIZE + EEPROM_EVENT_START, 0xFF);
        }
        break;
    case 's':
        //Scan
        Scan();
        break;
    case 'd':
        //debug
        for(unsigned int i = 0; i < 11; i++)
        {
            for(unsigned int j = 0; j < 10; j++)
            {
                byte nVal = EEPROM.read(i * EEPROM_SENSOR_SIZE + EEPROM_SENSOR_START + j);
                if(nVal < 0x10)
                    Serial.print("0");
                Serial.print(nVal, HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
        break;
    default:
        //Show help
        //!@todo Remove serial help if memory becomes scarce
        Serial.println(F("E - List Events"));
        Serial.println(F("E aa b hh:mm z +vvv - Add / modify event a=event id b=DoW, hh:mm-time, z=zone, +/-v=temperature (x10)"));
        Serial.println(F("S uuuuuuuuuuuuuuuuz - Add / modify sensor u=UID, z=zone"));
        Serial.println(F("S - List Sensors"));
        Serial.println(F("T hh:mm:ss a dd/mm/yy - Set time and date a=DoW, Sunday = 1"));
        Serial.println(F("T - Show time"));
        Serial.println(F("CE - Clear all events"));
        Serial.println(F("CS - Clear all sensors"));
        Serial.println(F("Z z aa b - Configure zone z=zone, a=hysteresis (C/10), b=1 for space heating"));
        Serial.println(F("Z - List zones"));
        Serial.println(F("s - Scan for sensors"));
        Serial.println(F("d - Debug output"));
    }
}

/**  @brief  Saves an event to EEPROM
*    @param  nEvent Event index
*/
void SaveEvent(unsigned int nEvent)
{
    EEPROM.write(nEvent * EEPROM_EVENT_SIZE + EEPROM_EVENT_START, g_events[nEvent].nDay);
    EEPROM.write(nEvent * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 1, (g_events[nEvent].nTime & 0xFF00) >> 8);
    EEPROM.write(nEvent * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 2, g_events[nEvent].nTime & 0xFF);
    EEPROM.write(nEvent * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 3, g_events[nEvent].nZone);
    EEPROM.write(nEvent * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 4, g_events[nEvent].nValue >> 8);
    EEPROM.write(nEvent * EEPROM_EVENT_SIZE + EEPROM_EVENT_START + 5, g_events[nEvent].nValue & 0xFF);
}

/**  @brief  Saves a zone to EEPROM
*    @param  nZone Zone index
*/
void SaveZone(unsigned int nZone)
{
    EEPROM.write(nZone * EEPROM_ZONE_SIZE + EEPROM_ZONE_START, g_zones[nZone].nHyst);
    EEPROM.write(nZone * EEPROM_ZONE_SIZE + EEPROM_ZONE_START + 1, g_zones[nZone].bSpace?1:0);
}

/** @brief  Saves a sensor configuration to EEPROM
*   @param  nSensor Sensor index
*/
void SaveSensor(unsigned int nSensor)
{
    for(unsigned int i = 0; i < 8; ++i)
        EEPROM.write(nSensor * EEPROM_SENSOR_SIZE + EEPROM_SENSOR_START + i, g_sensors[nSensor].address[i]);
    EEPROM.write(nSensor * EEPROM_SENSOR_SIZE + EEPROM_SENSOR_START + 8, g_sensors[nSensor].nZone);
}

/**  Add a sensor and write configuration to EEPROM
*/
void AddSensor(byte* pAddress, byte nZone)
{
    bool bDuplicate = false;
    unsigned int nSensor;
    for(nSensor = 0; nSensor < g_nSensorQuant; nSensor++)
    {
        for(unsigned int i = 0; i < 8; ++i)
        {
            if(*(pAddress + i) == g_sensors[nSensor].address[i])
            {
                bDuplicate = true;
            }
            else
            {
                bDuplicate = false;
                break;
            }
        }
        if(bDuplicate)
            break;
    }
    if(!bDuplicate)
    {
        if(g_nSensorQuant >= MAX_SENSORS)
        {
            Serial.println("Can't add any more sensors.");
            return; //Can't add any more sensors
        }
        Serial.print("Adding new sensor [");
        for(unsigned int i = 0; i < 8; ++i)
        {
            g_sensors[g_nSensorQuant].address[i] = *(pAddress + i);
            EEPROM.write(g_nSensorQuant * 10 + i, g_sensors[g_nSensorQuant].address[i]);
            Serial.print(g_sensors[g_nSensorQuant].address[i], HEX);
        }
        Serial.println("]");
        ++g_nSensorQuant;
    }
    else
        Serial.println("Updating existing sensor");
    g_sensors[nSensor].nZone = nZone;
    EEPROM.write(nSensor * 10 + 8, nZone);
}

/** @brief  Scans sensor network
*   @note   Prints list of sensor UID and current values to serial port
*/
void Scan()
{
    byte pAddress[8];
    while(ds.search(pAddress))
    {
        for(unsigned int i = 0; i < 8; i++)
        {
            if(pAddress[i] < 0x10)
                Serial.print("0");
            Serial.print(pAddress[i], HEX);
        }
        Serial.print(" Value=");
        float fTemp = GetTemperature(pAddress);
        if(fTemp == -2000)
            Serial.println("Error reading temperature");
        else
        {
            Serial.print(fTemp / 100);
            Serial.println("C");
        }
    }
}

/** @brief  Gets the temperature value from a sensor
*   @param  pAddress Pointer to the UID of the sensor
*   @return <i>int</i> Temperature in 1/100ths of degrees
*   @note   Returns -2000 on error
*/
int GetTemperature(byte* pAddress)
{
    ds.reset();
    ds.select(pAddress);
    ds.write(0x44); //Start conversion
    delay(1000); //Wait for read to complete

    ds.reset();
    ds.select(pAddress);
    ds.write(0xBE);
    byte data[9];
    for(int i = 0; i < 9; i++)
        data[i] = ds.read();

    if(OneWire::crc8(data, 8) == data[8])
    {
        int nValue = (data[0] + (data[1] << 8));
        if(nValue & 0x8000) // negative
            nValue = (nValue ^ 0xffff) + 1; // 2's comp
        return nValue * 6.25;
    }
    return -2000;
}

/**  @brief  Updates the temperature reading from a sensor
*    @param  nSensor Sensor index
*    @return <i>bool</i> True on success
*/
bool GetTemperature(unsigned int nSensor)
{
    if(nSensor >= MAX_SENSORS)
        return false;
    int nValue = GetTemperature(g_sensors[nSensor].address);
    if(nValue == -2000)
        return false;
    g_sensors[nSensor].nValue = nValue;
    return true;
}

/** @brief  Gets the date and time from the DS1307 RTC
*   @param  bShow Print result to serial if true
*   @return <i>byte<i> Number of seconds since minute boundary
*   @note   Updates g_nNow with number of minutes since 00:00 Sunday
*/
byte getTime(bool bShow)
{
#ifdef _DEBUG_
    return 0; //Allow debugging without RTC connected
#endif // _DEBUG_
    // Reset the register pointer
    Wire.beginTransmission(DS1307_I2C_ADDRESS);
    Wire.write(0);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_I2C_ADDRESS, 7); //Get 7 bytes of data from RTC

    // A few of these need masks because certain bits are control bits
    byte nSecond     = bcdToDec(Wire.read() & 0x7f);
    byte nMinute     = bcdToDec(Wire.read());
    byte nHour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
    g_tsNow.nDay     = bcdToDec(Wire.read());
    byte nDay        = bcdToDec(Wire.read());
    byte nMonth      = bcdToDec(Wire.read());
    byte nYear       = bcdToDec(Wire.read());
    g_tsNow.nTime    = nMinute + nHour * 60;

    if(bShow)
    {
        //!@todo Reduce vebosity of date if memory becomes sparse
        char  *Day[] = {"","Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
        char  *Mon[] = {"","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
        if (nHour < 10)
            Serial.print("0");
        Serial.print(nHour, DEC);
        Serial.print(":");
        if (nMinute < 10)
            Serial.print("0");
        Serial.print(nMinute, DEC);
        Serial.print(":");
        if (nSecond < 10)
            Serial.print("0");
        Serial.print(nSecond, DEC);
        Serial.print("  ");
        Serial.print(Day[g_tsNow.nDay]);
        Serial.print(", ");
        Serial.print(nDay, DEC);
        Serial.print(" ");
        Serial.print(Mon[nMonth]);
        Serial.print(" 20");
        if (nYear < 10)
            Serial.print("0");
        Serial.println(nYear, DEC);
    }
    return nSecond;
}

/** @brief  Set time of the RTC
*   @param  nHour Current hour
*   @param  nMinute Current minute
*   @param  nSecond Current second
*/
void setTime(unsigned int nHour, unsigned int nMinute, unsigned int nSecond)
{
    Wire.beginTransmission(DS1307_I2C_ADDRESS);
    Wire.write(0); //Set cursor to seconds register
    Wire.write(decToBcd(nSecond) & 0x7f); // Reset bit 7 starts the clock
    Wire.write(decToBcd(nMinute));
    Wire.write(decToBcd(nHour)); // If you want 12 hour am/pm you need to set
    Wire.endTransmission();
    g_tsNow.nTime = nMinute + nHour * 60;
}

/** @brief  Set date of the RTC
*   @param  nDow Current day of week. Sunday = 1
*   @param  nDay Current day of month
*   @param  nMonth Current month. January = 1
*   @param  nYear Current 2-digit year. 2014 = 14
*/
void setDate(unsigned int nDow, unsigned int nDay, unsigned int nMonth, unsigned int nYear)
{
    Wire.beginTransmission(DS1307_I2C_ADDRESS);
    Wire.write(3); //Set cursor to DoW register
    Wire.write(decToBcd(nDow));
    Wire.write(decToBcd(nDay));
    Wire.write(decToBcd(nMonth));
    Wire.write(decToBcd(nYear));
    Wire.endTransmission();
    g_tsNow.nDay = 1 << (nDow - 1);
}

/** @brief  Convert normal decimal numbers to binary coded decimal
*   @param  nValue Value to convert
*   @return <i>byte</i> BCD encoded byte
*   @note   Supports 2 digit numbers
*/
byte decToBcd(byte nValue)
{
    return ((nValue / 10 * 16) + (nValue % 10));
}

/** @brief  Convert binary coded decimal to normal decimal numbers
*   @param  nValue Value to convert
*   @return <i>byte</i> Decimal value
*   @note   Supports 2 digit numbers
*/
byte bcdToDec(byte nValue)
{
    return ((nValue / 16 * 10) + (nValue % 16));
}

/** @brief  Process all pending events and find next event time
*   @note   g_tsNextEventTime is updated with the next scheduled event or 0 if no more events this week
*/
void ProcessEvents()
{
    g_tsNextEvent.nTime = 0xFFFF;

    for(unsigned int nEvent = 0; nEvent < g_nEventQuant; nEvent++)
    {
        if(g_events[nEvent].nTime == g_tsNow.nTime && g_events[nEvent].nDay & g_tsNow.nDay)
        {
            //Set zone temperature set-point
            g_zones[g_events[nEvent].nZone].nSetpoint = g_events[nEvent].nValue;
        }

        //Find next scheduled event
        if(g_events[nEvent].nDay & g_tsNow.nDay && g_events[nEvent].nTime > g_tsNow.nTime && g_events[nEvent].nTime < g_tsNextEvent.nTime)
        {
            g_tsNextEvent.nTime = g_events[nEvent].nTime;
            g_tsNextEvent.nDay = g_tsNow.nDay;
        }
    }
    if(g_tsNextEvent.nTime == 0xFFFF)
    {
        //No more events today
        g_tsNextEvent.nTime = 0;
        g_tsNextEvent.nDay = g_tsNextEvent.nDay << 1;
        if(g_tsNextEvent.nDay > 127)
            g_tsNextEvent.nDay = 1; //wrap round to Sunday if reached end of Saturday
    }
    //!@todo remove this debug output
    Serial.print("Next event: ");
    Serial.print(g_tsNextEvent.nTime);
    Serial.print(" on ");
    Serial.print(g_tsNextEvent.nDay);
}

void DeleteEvent(byte nEvent)
{
    //Deleting event so shift all others
    for(g_nEventQuant = nEvent; g_nEventQuant < MAX_EVENTS - 1; g_nEventQuant++)
    {
        g_events[g_nEventQuant].nDay = g_events[g_nEventQuant + 1].nDay;
        if(0 == g_events[g_nEventQuant].nDay)
            break;
        g_events[g_nEventQuant].nTime = g_events[g_nEventQuant + 1].nTime;
        g_events[g_nEventQuant].nZone = g_events[g_nEventQuant + 1].nZone;
        g_events[g_nEventQuant].nValue = g_events[g_nEventQuant + 1].nValue;
    }
    Serial.print("Deleted event ");
    Serial.print(nEvent);
    Serial.print(". Quantity of events=");
    Serial.println(g_nEventQuant);
    return;
}
