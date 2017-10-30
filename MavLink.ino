#define SerialUSB Serial
#define Serial1 Serial


//Sets the output rate of the following messages. 0 = Disabled
#define MAV_RATE_STREAM_RAW_SENSORS 0 //--> Seems to Enable RAW_IMU ( #27 ) / SCALED_PRESSURE ( #29 ) / SCALED_IMU2 ( #116 )
#define MAV_RATE_STREAM_EXTRA1 10 //--> Seems to Enable ATTITUDE ( #30 )
#define MAV_RATE_STREAM_EXTRA2 1 //--> Seems to Enable VFR_HUD ( #74 )
#define MAV_RATE_STREAM_EXTENDED_STATUS 0 //--> Seems to increase the Verbose or details of STATUSTEXT ( #253 )??? 
#define MAV_RATE_STREAM_RAW_CONTROLLER 0 // --> No Tested yet
#define MAV_RATE_STREAM_POSITION 1 //--> No Tested yet
#define MAV_RATE_STREAM_RC_CHANNELS 0 // --> No Tested yet

//Needed for Mavlink:
uint8_t buf[128];
uint8_t buf_len;


/***********************************************/

int sysid = 0x77;                   ///< ID 20 for this "airplane"
int compid = 0xBE; // 190;     ///< The component sending the message is the IMU, it could be also a Linux process
int type = 6;   //

// Define the system type, in this case an airplane
uint8_t system_type = 6;
uint8_t autopilot_type = 8;

uint8_t system_mode = 0; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = 3; ///< System ready for flight
// Initialize the required buffers
mavlink_message_t msg;
mavlink_status_t status1;


/**********************************************************/

void heartBeat(void) {
  uint8_t buf2[128];
  uint8_t buf_len2;
  mavlink_message_t msg_temp;
  mavlink_msg_heartbeat_pack(sysid, compid, &msg_temp, type, autopilot_type, system_mode, custom_mode, system_state);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf2, &msg_temp);
  Serial.write(buf2, len);
  Serial.println("@"); //<---Just to ID the message, you can delete this.
}

/*********************************************************
  Usually, the autopilot will not send out anything over the
  telemetry port so you need to enable the feeds, use this only
  if you are NOT sniffing the MavLink data
***********************************************************/

void start_feeds(void) {

  uint8_t buf2[128];
  uint8_t buf_len2;
  uint16_t len;
  mavlink_message_t msg_feed;

  uint8_t target_component = 1;
  uint8_t target_sysid = 1;


  if (MAV_RATE_STREAM_RAW_SENSORS >= 1) {
    delay(100);
    mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg_feed, target_sysid, target_component, MAV_DATA_STREAM_RAW_SENSORS, MAV_RATE_STREAM_RAW_SENSORS, 1);
    len = mavlink_msg_to_send_buffer(buf2, &msg_feed);
    Serial.write(buf2, len);
  }

  if (MAV_RATE_STREAM_EXTRA1 >= 1) {
    Serial.println("Starting MAV_RATE_STREAM_EXTRA1");
    delay(100);
    mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg_feed, target_sysid, target_component, MAV_DATA_STREAM_EXTRA1, MAV_RATE_STREAM_EXTRA1, 1);
    len = mavlink_msg_to_send_buffer(buf2, &msg_feed);
    Serial.write(buf2, len);
  }

  if (MAV_RATE_STREAM_EXTRA2 >= 1) {
    delay(100);
    mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg_feed, target_sysid, target_component, MAV_DATA_STREAM_EXTRA2, MAV_RATE_STREAM_EXTRA2, 1);
    len = mavlink_msg_to_send_buffer(buf2, &msg_feed);
    Serial.write(buf2, len);
  }

  if (MAV_RATE_STREAM_EXTENDED_STATUS >= 1) {
    delay(100);
    mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg_feed, target_sysid, target_component, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_RATE_STREAM_EXTENDED_STATUS, 1);
    len = mavlink_msg_to_send_buffer(buf2, &msg_feed);
    Serial.write(buf2, len);
  }


  if (MAV_RATE_STREAM_RAW_CONTROLLER >= 1) {
    delay(100);
    mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg_feed, target_sysid, target_component, MAV_DATA_STREAM_RAW_CONTROLLER, MAV_RATE_STREAM_RAW_CONTROLLER, 1);
    len = mavlink_msg_to_send_buffer(buf2, &msg_feed);
    Serial.write(buf2, len);
  }

  if (MAV_RATE_STREAM_POSITION >= 1) {
    delay(100);
    mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg_feed, target_sysid, target_component, MAV_DATA_STREAM_POSITION, MAV_RATE_STREAM_POSITION, 1);
    len = mavlink_msg_to_send_buffer(buf2, &msg_feed);
    Serial.write(buf2, len);
  }

  if (MAV_RATE_STREAM_RC_CHANNELS >= 1) {
    delay(100);
    mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg_feed, target_sysid, target_component, MAV_DATA_STREAM_RC_CHANNELS, MAV_RATE_STREAM_RC_CHANNELS, 1);
    len = mavlink_msg_to_send_buffer(buf2, &msg_feed);
    Serial.write(buf2, len);
  }

}

/********************************************************************/

void comm_receive() {

  while (Serial.available() > 0 )
  {
    uint8_t c = Serial.read(); //Getting one byte from the MavLink stream.
    //Serial.print(" 0x"); Serial.print(c,HEX);
    // Try to get a new message
    if (mavlink_parse_char(1, c, &msg, &status1)) {
      // Handle message
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT: //#0
          {
            //SerialUSB.print("Received Heartbeat ---> ");

            mavlink_heartbeat_t test;
            mavlink_msg_heartbeat_decode(&msg, &test);
            //SerialUSB.print(" Autopilot: "); SerialUSB.print(test.autopilot); //implementation example is inside //c_library_v1/common/mavlink_msg_heartbeat.h
            //SerialUSB.print(" Type: "); SerialUSB.println(test.type);
          }
          break;
        case MAVLINK_MSG_ID_RAW_IMU: //#27
          {
            //For more info: https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_raw_imu.h
            /**
              @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
              @param xacc X acceleration (raw)
              @param yacc Y acceleration (raw)
              @param zacc Z acceleration (raw)
              @param xgyro Angular speed around X axis (raw)
              @param ygyro Angular speed around Y axis (raw)
              @param zgyro Angular speed around Z axis (raw)
              @param xmag X Magnetic field (raw)
              @param ymag Y Magnetic field (raw)
              @param zmag Z Magnetic field (raw)
            */
            //Example:
            //Serial.println(mavlink_msg_raw_imu_get_xacc(&msg));
          }
          break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE: //#29
          {
            //For more info: https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_scaled_pressure.h
            /**
              @param time_boot_ms Timestamp (milliseconds since system boot)
              @param press_abs Absolute pressure (hectopascal)
              @param press_diff Differential pressure 1 (hectopascal)
              @param temperature Temperature measurement (0.01 degrees celsius)
            */
            //Example:
            //Serial.println(mavlink_msg_scaled_pressure_get_press_abs(&msg));
          }
          break;
        case MAVLINK_MSG_ID_ATTITUDE: //#30
          {
            //For more info: https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_attitude.h
            /*
              @param time_boot_ms Timestamp (milliseconds since system boot)
              @param roll Roll angle (rad, -pi..+pi)
              @param pitch Pitch angle (rad, -pi..+pi)
              @param yaw Yaw angle (rad, -pi..+pi)
              @param rollspeed Roll angular speed (rad/s)
              @param pitchspeed Pitch angular speed (rad/s)
              @param yawspeed Yaw angular speed (rad/s)
            * */
            //SerialUSB.print(" Pitch ---> "); SerialUSB.println(mavlink_msg_attitude_get_pitch(&msg) * 57.2958f);
            //SerialUSB.print(" Roll ---> "); SerialUSB.println(mavlink_msg_attitude_get_roll(&msg) * 57.2958f);
            //SerialUSB.print(" Yaw ---> "); SerialUSB.println(mavlink_msg_attitude_get_yaw(&msg) * 57.2958f);
            Global_Pitch = (float)mavlink_msg_attitude_get_pitch(&msg) * 57.2958f;
            Global_Roll = (float)mavlink_msg_attitude_get_roll(&msg) * 57.2958f;
            Global_True_Heading = (float)mavlink_msg_attitude_get_yaw(&msg) * 57.2958f;
          }
          break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: //#33
          {
            //For more info: https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_global_position_int.h
            /*
              @param time_boot_ms Timestamp (milliseconds since system boot)
              @param lat Latitude, expressed as degrees * 1E7
              @param lon Longitude, expressed as degrees * 1E7
              @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
              @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
              @param vx Ground X Speed (Latitude, positive north), expressed as m/s * 100
              @param vy Ground Y Speed (Longitude, positive east), expressed as m/s * 100
              @param vz Ground Z Speed (Altitude, positive down), expressed as m/s * 100
              @param hdg Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
            */

            //SerialUSB.print(" Lat ---> "); SerialUSB.println(mavlink_msg_global_position_int_get_lat(&msg));
            //SerialUSB.print(" Lon ---> "); SerialUSB.println(mavlink_msg_global_position_int_get_lon(&msg));
            Global_Lat = (float)mavlink_msg_global_position_int_get_lat(&msg) / (float)100.00;
            Global_Lon = (float)mavlink_msg_global_position_int_get_lon(&msg) / (float)100.00;
            Global_Alt = (float)mavlink_msg_global_position_int_get_alt(&msg) / (float)1000.00;
            Global_Ground_Track = (float)mavlink_msg_global_position_int_get_hdg(&msg) / (float)100.00;
          }
          break;

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: //#66
          {

            SerialUSB.print("REQUEST_DATA_STREAM ---> "); //SerialUSB.println(mavlink_msg_attitude_get_pitch(&msg));
          }
          break;
        case MAVLINK_MSG_ID_VFR_HUD: //#74
          {
            //For more info: https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_vfr_hud.h
            /**
              @param airspeed Current airspeed in m/s
              @param groundspeed Current ground speed in m/s
              @param heading Current heading in degrees, in compass units (0..360, 0=north)
              @param throttle Current throttle setting in integer percent, 0 to 100
              @param alt Current altitude (MSL), in meters
              @param climb Current climb rate in meters/second
            */
            //Example:
            //Serial.println(mavlink_msg_vfr_hud_get_airspeed(&msg));

            Global_Ground_Speed = (float)mavlink_msg_vfr_hud_get_groundspeed(&msg);

          }
          break;
        case MAVLINK_MSG_ID_SCALED_IMU2: //#116
          {
            //For more info: https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_scaled_imu2.h
            /**
              @param time_boot_ms Timestamp (milliseconds since system boot)
              @param xacc X acceleration (mg)
              @param yacc Y acceleration (mg)
              @param zacc Z acceleration (mg)
              @param xgyro Angular speed around X axis (millirad /sec)
              @param ygyro Angular speed around Y axis (millirad /sec)
              @param zgyro Angular speed around Z axis (millirad /sec)
              @param xmag X Magnetic field (milli tesla)
              @param ymag Y Magnetic field (milli tesla)
              @param zmag Z Magnetic field (milli tesla)
            */
            //Example:
            //Serial.println(mavlink_msg_scaled_imu2_get_xacc(&msg));
          }
          break;
        case MAVLINK_MSG_ID_STATUSTEXT: //#253
          {

            char statusText[50];
            mavlink_msg_statustext_get_text(&msg, statusText);

            SerialUSB.print("STATUS ---> "); SerialUSB.write(statusText); SerialUSB.println("");

          }
          break;
        default:
          SerialUSB.print("Received Not Supported Msg ID --> "); SerialUSB.println(msg.msgid);
          break;
      }
    }

    // And get the next one
  }
}
