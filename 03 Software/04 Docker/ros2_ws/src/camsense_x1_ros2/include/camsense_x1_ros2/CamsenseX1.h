# pragma once
#include <boost/asio.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

namespace Camsense
{
enum State {SYNC, DATA};
#define SAMPLES_PER_REV 400

class CamsenseX1
{

public:
    const uint DATA_LEN = SAMPLES_PER_REV;              // Samples per scan
    const float ANGLE_INCREMENT = SAMPLES_PER_REV/360.;  // Angle in degrees between 2 samples
    const float MIN_RANGE = 0.08;                       // Minimum distance
    const float MAX_RANGE = 8.;                         // Maximum distance
    float distances[SAMPLES_PER_REV];                   // Stores distances for most recent scan in m (clockwise)
    uint8_t qualities[SAMPLES_PER_REV];                 // Stores qualities for most recent scan (clockwise)
    

    /* Constructor
        
        @param devname Serial device name
        @param offsetAngle Sensor offset in degrees
    */
    CamsenseX1(const std::string& devname, float offsetAngle=16.):
    io(), serial(io,devname), OFFSET_ANGLE(offsetAngle)
    {   
        serial.set_option(boost::asio::serial_port_base::baud_rate(this->BAUDRATE));
    }

    /* Reads the next data fragment
        @return true, if a complete package was completed
    
    */
    bool readNext()
    {
        if (readState == SYNC)
        {   
            uint8_t res;
            boost::asio::read(serial,boost::asio::buffer(&res,1));
            switch(stateCounter)
            {
                case 0:
                    if (res == 0x55) stateCounter++; 
                    break;
                case 1:
                    
                    if (res == 0xAA) stateCounter++;
                    else if (res != 0x55) stateCounter = 0; 
                    break;
                case 2:
                    if (res == 0x03) stateCounter++; 
                    else stateCounter = 0; 
                    break;
                case 3:
                    if (res == 0x08) readState = DATA;
                    stateCounter = 0; 
                    break;
            }
        }
        else if (readState == DATA)
        {   
            uint8_t data[32];
            boost::asio::read(serial,boost::asio::buffer(data,32));
            parsePackage(data);
            readState = SYNC;
            return true;
        }
        return false;
    }

    /* Reads the next data package
        BLOCKING - about 250Hz
        @return true, if reading was sucessful
    
    */
    bool readPackage()
    {
        while(!readNext()){}
        return true;
    }

    /* Reads the next scan
        BLOCKING - about 5Hz
        @return true, if reading was sucessful
    
    */
    bool readScan()
    {
        for(uint pkg_cnt = 0; pkg_cnt < PACKAGES_PER_REV; pkg_cnt++)
            readPackage();
            
        return true;
    }

    /* Parses a data package
        Algorithm according to https://github.com/Vidicon/camsense-X1
        @param data 32-byte data array 
    */
    void parsePackage(uint8_t *data)
    {
        this->hz = (float)conc2bytes(data) / 3840.0; // 3840.0 = (64 * 60)
        this->startAngle = conc2bytes(data+2) / 64.0 - 640.0;
        this->endAngle = conc2bytes(data+28) / 64.0 - 640.0;
        
        float step = ((this->endAngle > this->startAngle) ? (this->endAngle - this->startAngle): (this->endAngle - (this->startAngle - 360))) / 8;

        // Align flat edge to be the front
        float sampleAngle = this->startAngle + this->OFFSET_ANGLE + 180;
        for(int i = 0; i < 8; i++) // for each of the 8 samples
        {      
            // Map 360° to 400 samples, round and keep in range [0, 399]
            int idx = (int)std::round(sampleAngle * this->ANGLE_INCREMENT) % this->DATA_LEN ; 

            uint8_t quality = data[6+(i*3)];
            
            if(quality == 0)
                this->distances[idx] = 0;
            else
                this->distances[idx] = conc2bytes(data+(4+(i*3)))*0.001;

            this->qualities[idx] = quality;
            sampleAngle += step;
            
        }
    }

    

    /* Sets the offset angle of the sensor
        @param offset angle in degrees
    */
    inline void setOffsetAngle(float offset){this->OFFSET_ANGLE = offset;}

    /* Returns the current time difference between two sample points.
        @return Time increment in s
    */
    inline float getTimeIncrement(){return 1/(hz*DATA_LEN);}

    /* Returns the current rotation speed in hz.
        @return Speed in Hz
    */
    inline float getSpeed(){return hz;}

private:

    const uint BAUDRATE = 115200;
    const uint PACKAGES_PER_REV = 50;
    float OFFSET_ANGLE;

    State readState = SYNC;
    uint stateCounter = 0;
    float hz = 0;
    float startAngle = 0;
    float endAngle = 0;

    boost::asio::io_service io;
    boost::asio::serial_port serial;

    inline uint16_t conc2bytes(uint8_t *data){ return (uint16_t)(data[1] << 8 | data[0]);}
};


} // namespace Camsense