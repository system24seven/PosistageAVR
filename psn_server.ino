#include <avr_stl.h>



// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
        mpuInterrupt = true;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/**
   The MIT License (MIT)

   Copyright (c) 2014 VYV Corporation

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
 **/
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

#include "psn_lib.hpp"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <cmath>
#include <iostream>
#include <list>
#include <cstring>
#include <string>


byte mac[] = {
        0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(192, 168, 2, 25);
//IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 0, 0, 0);

unsigned int UDPport = 56565;// local port to listen for UDP packets

IPAddress UDPServer(236, 10, 10, 10); // destination device server

EthernetUDP Udp;

::std::basic_string <char> bufval;
byte sendBuffer1[] = {0x90, 0x14, 0x22};
byte sendBuffer2[] = {0x80, 0x14, 0x00};


::psn::psn_encoder psn_encoder( "ShadowControls PSN Server" );

::psn::tracker_map trackers;

uint64_t timestamp = 0;
int i = 0;

void setup() {

        ///IMU
        // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);


        // load and configure the DM
        devStatus = mpu.dmpInitialize();

        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
                // Calibration Time: generate offsets and calibrate our MPU6050
                mpu.CalibrateAccel(6);
                mpu.CalibrateGyro(6);
                mpu.PrintActiveOffsets();
                // turn on the DMP, now that it's ready
                mpu.setDMPEnabled(true);

                // enable Arduino interrupt detection
                attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                mpuIntStatus = mpu.getIntStatus();

                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                dmpReady = true;

                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
        }






        ///IMU

        //Ethernet.init(10);

        //Ethernet.begin(mac, ip);
        // Open serial communications and wait for port to open:

        //Serial.begin(9600);
        Ethernet.begin(mac, ip);
        Udp.beginMulticast(UDPServer, UDPport);
        //Udp.begin(UDPport);






        trackers[ i ] = ::psn::tracker( i++, "Tracker" );

}

void loop(  )
{
        // if programming failed, don't try to do anything
        if (!dmpReady) return;

        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
                if (mpuInterrupt && fifoCount < packetSize) {
                        // try to get out of the infinite loop
                        fifoCount = mpu.getFIFOCount();
                }
                trackers[ i ].set_pos( ::psn::float3( 1.0f, 1.0f, 1.0f ) );
                trackers[ i ].set_speed( ::psn::float3( 0, 0, 0 )  );
                trackers[ i ].set_ori( ::psn::float3( -ypr[1] * 180/M_PI +180, ypr[2] * 180/M_PI  +180, -ypr[0] * 180/M_PI +180) );
                //trackers[ i ].set_accel( ::psn::float3( ::psn::float3( 0 , 0 , 0 )  ) ) ;
                //trackers[ i ].set_target_pos( ::psn::float3( 0 , 0 , 0 ) ) ;
                //trackers[ i ].set_status( i / 10.0f ) ;
                trackers[ i ].set_timestamp( timestamp );

                ::std::list< ::std::string > data_packets = psn_encoder.encode_data( trackers, timestamp );


                for ( auto it = data_packets.begin(); it != data_packets.end(); ++it )
                {
                        // Uncomment these lines if you want to simulate a packet drop now and then
                        /*static uint64_t packet_drop = 0 ;
                           if ( packet_drop++ % 100 != 0 )*/
                        bufval = data_packets.front();
                        //Serial.println(bufval.length());
                        // send two packets to the Multicast address
                        Udp.beginPacket(UDPServer, UDPport);
                        Udp.write(bufval.c_str(), bufval.length() );
                        Udp.endPacket();
                        // socket_server.send_message( ::psn::DEFAULT_UDP_MULTICAST_ADDR , ::psn::DEFAULT_UDP_PORT , *it ) ;
                        ;
                }
                // Send Info
                if ( timestamp % 1000 == 0 ) // transmit info at 1 Hz approx.
                {
                        ::std::list< ::std::string > info_packets = psn_encoder.encode_info( trackers, timestamp );

                        for ( auto it = info_packets.begin(); it != info_packets.end(); ++it )
                                // socket_server.send_message( ::psn::DEFAULT_UDP_MULTICAST_ADDR , ::psn::DEFAULT_UDP_PORT , *it ) ;
                                bufval = info_packets.front();
                        Udp.beginPacket(UDPServer, UDPport);
                        Udp.write(bufval.c_str(), bufval.length());
                        Udp.endPacket();
                }

                timestamp++;




        }

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize) {
                //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
                // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        }
        // check for overflow (this should never happen unless our code is too inefficient)
        else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask

                // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

                // read a packet from FIFO
                while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
                        mpu.getFIFOBytes(fifoBuffer, packetSize);
                        // track FIFO count here in case there is > 1 packet available
                        // (this lets us immediately read more without waiting for an interrupt)
                        fifoCount -= packetSize;
                }

                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                // Update trackers
        }
}
