
#include "psn_lib.hpp"

//Declare Ethernet values
//MAC Address
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(192, 168, 2, 25);
IPAddress subnet(255, 0, 0, 0);

unsigned int UDPport = 56565;//PSN Sender Port

IPAddress UDPServer(236, 10, 10, 10); //Multicast PSN Address
//Make Ethernet object
EthernetUDP Udp;

::std::basic_string <char> bufval; //for manipulating PSN Send Buffer

//Declare PSN Server Title
::psn::psn_encoder psn_encoder( "ShadowControls PSN Server" ) ;

//Declare Trackers
::psn::tracker_map trackers ;

uint64_t timestamp = 0 ; //holds packet timestamp
int d = 0;

void setup() {

  //Start Ethernet
  Ethernet.begin(mac, ip);
  //Begin UDP Multicast
  Udp.beginMulticast(UDPServer, UDPport);

  //Declare Trackers ::psn::tracker( <tracker id> , <tracker name> ) ;
  trackers[ 0 ] = ::psn::tracker( 0, "Sun" ) ;
}

void loop(  )
{

  // Update trackers
  for ( int i = 0 ; i < 1 ; ++i ) // do not update the sun
  {
    d = timestamp >> 8;
    trackers[ i ].set_pos( ::psn::float3( 5.0f*sin(d), 0 , 1.0f ) ) ;
    trackers[ i ].set_speed( ::psn::float3( 0, 0 , 0 )  ) ;
    trackers[ i ].set_ori( ::psn::float3( 0 , 1.0f , 0 ) ) ;
    //trackers[ i ].set_accel( ::psn::float3( ::psn::float3( 0 , 0 , 0 )  ) ) ;
    //trackers[ i ].set_target_pos( ::psn::float3( 0 , 0 , 0 ) ) ;
    //trackers[ i ].set_status( i / 10.0f ) ;
    trackers[ i ].set_timestamp( timestamp ) ;
  }
  
  // Send Data
  if ( timestamp % 8 == 0 ) // transmit data at 60 Hz approx.
  {
    ::std::list< ::std::string > data_packets = psn_encoder.encode_data( trackers , timestamp ) ; //Get encoded packets from PSN library


    for ( auto it = data_packets.begin() ; it != data_packets.end() ; ++it )
    {
      // Uncomment these lines if you want to simulate a packet drop now and then
      /*static uint64_t packet_drop = 0 ;
        if ( packet_drop++ % 100 != 0 )*/
      bufval = data_packets.front();
      // send two packets to the Multicast address
      Udp.beginPacket(UDPServer, UDPport);
      Udp.write(bufval.c_str(), bufval.length() );
      Udp.endPacket();
    }
  }

  // Send Info
  if ( timestamp % 1000 == 0 ) // transmit info at 1 Hz approx.
  {
    ::std::list< ::std::string > info_packets = psn_encoder.encode_info( trackers , timestamp ) ; //Get encoded packets from PSN library

    for ( auto it = info_packets.begin() ; it != info_packets.end() ; ++it )
    {
      // socket_server.send_message( ::psn::DEFAULT_UDP_MULTICAST_ADDR , ::psn::DEFAULT_UDP_PORT , *it ) ;
      bufval = info_packets.front();
      Udp.beginPacket(UDPServer, UDPport);
      Udp.write(bufval.c_str(), bufval.length());
      Udp.endPacket();
    }
  }

  timestamp++ ;

}
