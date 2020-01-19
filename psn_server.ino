#include <avr_stl.h>

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

::psn::psn_encoder psn_encoder( "ShadowControls PSN Server" ) ;

::psn::tracker_map trackers ;


// Planets orbit in days
float orbits[ 10 ] = { 1.0f , 88.0f , 224.7f , 365.2f , 687.0f , 4332.0f , 10760.0f , 30700.0f , 60200.0f , 90600.0f } ;
// Planets distance from sun in million km x 100
float dist_from_sun[ 10 ] = { 0.0f , 0.58f , 1.08f , 1.50f , 2.28f , 7.78f , 14.29f , 28.71f , 45.04f , 59.13f } ;

uint64_t timestamp = 0 ;

void setup() {

  //Ethernet.init(10);

  //Ethernet.begin(mac, ip);
  // Open serial communications and wait for port to open:

  //Serial.begin(9600);
  Ethernet.begin(mac, ip);
  Udp.beginMulticast(UDPServer, UDPport);
  //Udp.begin(UDPport);



  int i = 0 ;

  trackers[ i ] = ::psn::tracker( i++ , "Sun" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Mercury" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Venus" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Earth" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Mars" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Jupiter" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Saturn" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Uranus" ) ;
  //trackers[ i ] = ::psn::tracker( i++ , "Neptune" ) ;

}

void loop(  )
{

  // Update trackers
  for ( int i = 1 ; i < 2 ; ++i ) // do not update the sun
  {
    trackers[ i ].set_pos( ::psn::float3( 1.0f , 1.0f , 1.0f ) ) ;
    trackers[ i ].set_speed( ::psn::float3( 0, 0 , 0 )  ) ;
    trackers[ i ].set_ori( ::psn::float3( 0 , 1.0f , 0 ) ) ;
    //trackers[ i ].set_accel( ::psn::float3( ::psn::float3( 0 , 0 , 0 )  ) ) ;
    //trackers[ i ].set_target_pos( ::psn::float3( 0 , 0 , 0 ) ) ;
    //trackers[ i ].set_status( i / 10.0f ) ;
    trackers[ i ].set_timestamp( timestamp ) ;
  }

  /*// send two packets to the Multicast address (in this case Midi note on and note off every sec)
    Udp.beginPacket(UDPServer, UDPport);
    Udp.write(sendBuffer1[0]);
    Udp.write(sendBuffer1[1]);
    Udp.write(sendBuffer1[2]);
    Udp.endPacket();
    // Serial.println("Send packet1");
    delay(1000);
    Udp.beginPacket(UDPServer, UDPport);
    Udp.write(sendBuffer2[0]);
    Udp.write(sendBuffer2[1]);
    Udp.write(sendBuffer2[2]);
    Udp.endPacket();
    // Serial.println("Send packet2");
    delay(1000);
    // Send data*/
  if ( timestamp % 16 == 0 ) // transmit data at 60 Hz approx.
  {
    ::std::list< ::std::string > data_packets = psn_encoder.encode_data( trackers , timestamp ) ;


    for ( auto it = data_packets.begin() ; it != data_packets.end() ; ++it )
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
  }

  // Send Info
  if ( timestamp % 1000 == 0 ) // transmit info at 1 Hz approx.
  {
    ::std::list< ::std::string > info_packets = psn_encoder.encode_info( trackers , timestamp ) ;

    for ( auto it = info_packets.begin() ; it != info_packets.end() ; ++it )
      // socket_server.send_message( ::psn::DEFAULT_UDP_MULTICAST_ADDR , ::psn::DEFAULT_UDP_PORT , *it ) ;
      bufval = info_packets.front();
    Udp.beginPacket(UDPServer, UDPport);
    Udp.write(bufval.c_str(), bufval.length());
    Udp.endPacket();
  }

  timestamp++ ;

}
