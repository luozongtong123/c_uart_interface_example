/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <thread>
#include <iostream>
#include "autopilot_interface.h"

using namespace std;
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t get_time_msec()
{
#ifdef WIN32
#include <windows.h>
    SYSTEMTIME _time_stamp;
    GetLocalTime(&_time_stamp);
    return _time_stamp.wHour * 3600 * 1000 + _time_stamp.wMinute * 60 * 1000 +
           _time_stamp.wSecond * 1000 + _time_stamp.wMilliseconds;
#else
#include <sys/time.h>
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec * 1000 + _time_stamp.tv_usec / 1000;
#endif
}

// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x = x;
    sp.y = y;
    sp.z = z;

    printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);
}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.vx = vx;
    sp.vy = vy;
    sp.vz = vz;

    //printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

    // NOT IMPLEMENTED
    fprintf(stderr, "set_acceleration doesn't work yet \n");
    throw 1;

    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.afx = ax;
    sp.afy = ay;
    sp.afz = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp.yaw = yaw;

    printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp.yaw_rate = yaw_rate;
}

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::Autopilot_Interface(Serial_Port *serial_port_)
{
    // initialize attributes
    write_count = 0;

    reading_status = 0;   // whether the read thread is running
    writing_status = 0;   // whether the write thread is running
    control_status = 0;   // whether the autopilot is in offboard control mode
    time_to_exit = false; // flag to signal thread exit

    read_tid = 0;  // read thread id
    write_tid = 0; // write thread id

    system_id = 0;    // system id
    autopilot_id = 0; // autopilot component id
    companion_id = 0; // companion computer component id

    current_messages.sysid = system_id;
    current_messages.compid = autopilot_id;

    serial_port = serial_port_; // serial port management object
}

Autopilot_Interface::~Autopilot_Interface()
{
}

// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
    current_setpoint = setpoint;
}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_messages()
{
    bool success;              // receive success flag
    bool received_all = false; // receive only one message
    Time_Stamps this_timestamps;

    // Blocking wait for new data
    while (!received_all and !time_to_exit)
    {
        // ----------------------------------------------------------------------
        //   READ MESSAGE
        // ----------------------------------------------------------------------
        mavlink_message_t message;
        success = serial_port->read_message(message);

        // ----------------------------------------------------------------------
        //   HANDLE MESSAGE
        // ----------------------------------------------------------------------
        if (success)
        {

            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages.sysid = message.sysid;
            current_messages.compid = message.compid;

            // Handle Message ID
            switch (message.msgid)
            {

            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                // cout << "MAVLINK_MSG_ID_HEARTBEAT" << endl;
                mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                current_messages.time_stamps.heartbeat = get_time_msec();
                // printf("timestamp:%d\n", current_messages.time_stamps.heartbeat);
                this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                // printf("type:%d, autopilot:%d, base_mode:%d, custom_mode:%D, system_status:%d, mavlink_version:%d \n",
                // 	current_messages.heartbeat.type, current_messages.heartbeat.autopilot,
                // 	current_messages.heartbeat.base_mode, current_messages.heartbeat.custom_mode,
                // 	current_messages.heartbeat.system_status, current_messages.heartbeat.mavlink_version);
                // cout << "type:" << current_messages.heartbeat.type << "autopilot:" <<
                // current_messages.heartbeat.autopilot << "base_mode:" << current_messages.heartbeat.base_mode <<
                // "custom_mode:" << current_messages.heartbeat.custom_mode << "system_status:" <<
                // current_messages.heartbeat.system_status << "mavlink_version:" <<
                // current_messages.heartbeat.mavlink_version << endl;
                break;
            }

            case MAVLINK_MSG_ID_SYS_STATUS:
            {
                //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                current_messages.time_stamps.sys_status = get_time_msec();
                this_timestamps.sys_status = current_messages.time_stamps.sys_status;
                break;
            }

            case MAVLINK_MSG_ID_BATTERY_STATUS:
            {
                //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                current_messages.time_stamps.battery_status = get_time_msec();
                this_timestamps.battery_status = current_messages.time_stamps.battery_status;
                break;
            }

            case MAVLINK_MSG_ID_RADIO_STATUS:
            {
                //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                current_messages.time_stamps.radio_status = get_time_msec();
                this_timestamps.radio_status = current_messages.time_stamps.radio_status;
                break;
            }

            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            {
                //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                current_messages.time_stamps.local_position_ned = get_time_msec();
                this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
                break;
            }

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            {
                //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                current_messages.time_stamps.global_position_int = get_time_msec();
                this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
                break;
            }

            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            {
                //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
                current_messages.time_stamps.position_target_local_ned = get_time_msec();
                this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
                break;
            }

            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
            {
                //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
                current_messages.time_stamps.position_target_global_int = get_time_msec();
                this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
                break;
            }

            case MAVLINK_MSG_ID_HIGHRES_IMU:
            {
                //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                current_messages.time_stamps.highres_imu = get_time_msec();
                this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
                break;
            }

            case MAVLINK_MSG_ID_ATTITUDE:
            {
                //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                current_messages.time_stamps.attitude = get_time_msec();
                this_timestamps.attitude = current_messages.time_stamps.attitude;
                break;
            }

            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            {
                //printf("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW\n");
                mavlink_msg_servo_output_raw_decode(&message, &(current_messages.servo_output_raw));
                current_messages.time_stamps.servo_output_raw = get_time_msec();
                this_timestamps.servo_output_raw = current_messages.time_stamps.servo_output_raw;
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_ACK:
            {
                //printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
                mavlink_msg_command_ack_decode(&message, &(current_messages.command_ack));
                current_messages.time_stamps.command_ack = get_time_msec();
                this_timestamps.command_ack = current_messages.time_stamps.command_ack;
                printf("Command_ACK, command:%d, result:%d. \n", current_messages.command_ack.command,
                       current_messages.command_ack.result);
                break;
            }
            case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
            {
                //printf("MAVLINK_MSG_ID_NAMED_VALUE_FLOA\n");
                mavlink_msg_named_value_float_decode(&message, &(current_messages.named_value_float));
                current_messages.time_stamps.named_value_float = get_time_msec();
                this_timestamps.named_value_float = current_messages.time_stamps.named_value_float;
                // printf(current_messages.named_value_float.name);
                // printf(": %f \n", current_messages.named_value_float.value);
                break;
            }
            case MAVLINK_MSG_ID_VFR_HUD:
            {
                // printf("MAVLINK_MSG_ID_VFR_HUD\n");
                mavlink_msg_vfr_hud_decode(&message, &(current_messages.vfr_hud));
                current_messages.time_stamps.vfr_hud = get_time_msec();
                this_timestamps.vfr_hud = current_messages.time_stamps.vfr_hud;
                // printf("heading:%d\n", current_messages.vfr_hud.heading);
                break;
            }
            case MAVLINK_MSG_ID_POWER_STATUS:
            {
                // printf("MAVLINK_MSG_ID_POWER_STATUS\n");
                mavlink_msg_power_status_decode(&message, &(current_messages.power_status));
                current_messages.time_stamps.power_status = get_time_msec();
                this_timestamps.power_status = current_messages.time_stamps.power_status;
                // printf("Vcc(5V rail voltage in mV):%d, Vservo(servo rail voltage in mV):%d, "
                // "power supply status flags:%d.\n", current_messages.power_status.Vcc,
                // current_messages.power_status.Vservo, current_messages.power_status.flags);
                break;
            }
            case MAVLINK_MSG_ID_SYSTEM_TIME:
            {
                // printf("MAVLINK_MSG_ID_SYSTEM_TIME\n");
                mavlink_msg_system_time_decode(&message, &(current_messages.system_time));
                current_messages.time_stamps.system_time = get_time_msec();
                this_timestamps.system_time = current_messages.time_stamps.system_time;
                break;
            }
            case MAVLINK_MSG_ID_MISSION_CURRENT:
            {
                // printf("MAVLINK_MSG_ID_MISSION_CURRENT\n");
                mavlink_msg_mission_current_decode(&message, &(current_messages.mission_current));
                current_messages.time_stamps.mission_current = get_time_msec();
                this_timestamps.mission_current = current_messages.time_stamps.mission_current;
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT:
            {
                // printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
                mavlink_msg_gps_raw_int_decode(&message, &(current_messages.gps_raw_int));
                current_messages.time_stamps.gps_raw_int = get_time_msec();
                this_timestamps.gps_raw_int = current_messages.time_stamps.gps_raw_int;
                break;
            }
            case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
            {
                // printf("MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT\n");
                mavlink_msg_nav_controller_output_decode(&message, &(current_messages.nav_controller_output));
                current_messages.time_stamps.nav_controller_output = get_time_msec();
                this_timestamps.nav_controller_output = current_messages.time_stamps.nav_controller_output;
                break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS:
            {
                // printf("MAVLINK_MSG_ID_RC_CHANNELS\n");
                mavlink_msg_rc_channels_decode(&message, &(current_messages.rc_channels));
                current_messages.time_stamps.rc_channels = get_time_msec();
                this_timestamps.rc_channels = current_messages.time_stamps.rc_channels;
                break;
            }
            case MAVLINK_MSG_ID_VIBRATION:
            {
                // printf("MAVLINK_MSG_ID_VIBRATION\n");
                mavlink_msg_vibration_decode(&message, &(current_messages.vibration));
                current_messages.time_stamps.vibration = get_time_msec();
                this_timestamps.vibration = current_messages.time_stamps.vibration;
                break;
            }
            case MAVLINK_MSG_ID_RAW_IMU:
            {
                // printf("MAVLINK_MSG_ID_RAW_IMU\n");
                mavlink_msg_raw_imu_decode(&message, &(current_messages.raw_imu));
                current_messages.time_stamps.raw_imu = get_time_msec();
                this_timestamps.raw_imu = current_messages.time_stamps.raw_imu;
                break;
            }
            case MAVLINK_MSG_ID_SCALED_PRESSURE:
            {
                // printf("MAVLINK_MSG_ID_SCALED_PRESSURE\n");
                mavlink_msg_scaled_pressure_decode(&message, &(current_messages.scaled_pressure));
                current_messages.time_stamps.scaled_pressure = get_time_msec();
                this_timestamps.scaled_pressure = current_messages.time_stamps.scaled_pressure;
                break;
            }
            case MAVLINK_MSG_ID_SCALED_IMU2:
            {
                // printf("MAVLINK_MSG_ID_SCALED_IMU2\n");
                mavlink_msg_scaled_imu2_decode(&message, &(current_messages.scaled_imu2));
                current_messages.time_stamps.scaled_imu2 = get_time_msec();
                this_timestamps.scaled_imu2 = current_messages.time_stamps.scaled_imu2;
                break;
            }
            case MAVLINK_MSG_ID_SCALED_PRESSURE2:
            {
                // printf("MAVLINK_MSG_ID_SCALED_PRESSURE2\n");
                mavlink_msg_scaled_pressure2_decode(&message, &(current_messages.scaled_pressure2));
                current_messages.time_stamps.scaled_pressure2 = get_time_msec();
                this_timestamps.scaled_pressure2 = current_messages.time_stamps.scaled_pressure2;
                break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            {
                // printf("MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
                mavlink_msg_rc_channels_raw_decode(&message, &(current_messages.rc_channels_raw));
                current_messages.time_stamps.rc_channels_raw = get_time_msec();
                this_timestamps.scaled_pressure2 = current_messages.time_stamps.rc_channels_raw;
                break;
            }
            case MAVLINK_MSG_ID_STATUSTEXT:
            {
                cout << "MAVLINK_MSG_ID_STATUSTEXT" << endl;
                mavlink_msg_statustext_decode(&message, &(current_messages.statustext));
                current_messages.time_stamps.statustext = get_time_msec();
                this_timestamps.statustext = current_messages.time_stamps.statustext;
                cout << "severity:" << current_messages.statustext.severity << ",text:" << current_messages.statustext.text << endl;
                // printf("severity:%d", current_messages.statustext.severity);
                // printf(current_messages.statustext.text);
                // printf("\n");
                break;
            }
            case MAVLINK_MSG_ID_PARAM_VALUE:
            {
                cout << "MAVLINK_MSG_ID_PARAM_VALUE" << endl;
                mavlink_msg_param_value_decode(&message, &(current_messages.param_value));
                current_messages.time_stamps.param_value = get_time_msec();
                this_timestamps.param_value = current_messages.time_stamps.param_value;
                cout << "param_id:" << current_messages.param_value.param_id << ", param_value:" << current_messages.param_value.param_value << ", param_type:" << current_messages.param_value.param_type << ", param_count:" << current_messages.param_value.param_count << ", param_index:" << current_messages.param_value.param_index << endl;
                break;
            }
            default:
            {
                printf("Warning, did not handle message id %i\n", message.msgid);
                break;
            }

            } // end: switch msgid

        } // end: if read message

        // Check for receipt of all items
        received_all =
            this_timestamps.heartbeat &&
            //				this_timestamps.battery_status             &&
            //				this_timestamps.radio_status               &&
            //				this_timestamps.local_position_ned         &&
            //				this_timestamps.global_position_int        &&
            //				this_timestamps.position_target_local_ned  &&
            //				this_timestamps.position_target_global_int &&
            //				this_timestamps.highres_imu                &&
            //				this_timestamps.attitude                   &&
            this_timestamps.sys_status;

        // give the write thread time to use the port
        if (writing_status > false)
        {
            usleep(100); // look for components of batches at 10kHz
        }

    } // end: while not received all

    return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int Autopilot_Interface::write_message(mavlink_message_t message)
{
    // do the write
    int len = serial_port->write_message(message);

    // book keep
    write_count++;

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_setpoint()
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    // pull from position target
    mavlink_set_position_target_local_ned_t sp = current_setpoint;

    // double check some system parameters
    if (not sp.time_boot_ms)
        sp.time_boot_ms = (uint32_t)get_time_msec();
    sp.target_system = system_id;
    sp.target_component = autopilot_id;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if (len <= 0)
        // fprintf(stderr, "WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
        cout << "WARNING: could not send POSITION_TARGET_LOCAL_NED" << endl;
    //	else
    //		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

    return;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_heartbeat(mavlink_heartbeat_t hb)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_heartbeat_encode(system_id, companion_id, &message, &hb);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);
    // cout << "heartbeat msg wrote!" << endl;

    // check the write
    if (len <= 0)
        // fprintf(stderr, "WARNING: could not send heartbeat \n");
        cout << "WARNING: could not send heartbeat" << endl;
    //	else
    //		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

    return;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::do_set_servo(float servo_no, float pwm)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    mavlink_command_long_t cmd_long;
    cmd_long.target_system = system_id;
    cmd_long.target_component = companion_id;
    cmd_long.command = MAV_CMD_DO_SET_SERVO;
    cmd_long.confirmation = 0;
    cmd_long.param1 = servo_no;
    cmd_long.param2 = pwm;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &cmd_long);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);
    cout << "do_set_servo msg wrote!" << endl;

    // check the write
    if (len <= 0)
        // fprintf(stderr, "WARNING: could not send heartbeat \n");
        cout << "WARNING: could not send heartbeat" << endl;
    //	else
    //
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::do_motor_test(float motor_no, float pwm)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    mavlink_command_long_t cmd_long;
    cmd_long.target_system = system_id;
    cmd_long.target_component = companion_id;
    cmd_long.command = MAV_CMD_DO_MOTOR_TEST;
    cmd_long.confirmation = 0;
    cmd_long.param1 = motor_no - 1;
    cmd_long.param2 = MOTOR_TEST_THROTTLE_PWM;
    cmd_long.param3 = pwm;
    cmd_long.param4 = 1;
    cmd_long.param5 = 1;
    cmd_long.param6 = MOTOR_TEST_ORDER_DEFAULT;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &cmd_long);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);
    cout << "do_motor_test msg wrote!" << endl;

    // check the write
    if (len <= 0)
        // fprintf(stderr, "WARNING: could not send heartbeat \n");
        cout << "WARNING: could not send heartbeat" << endl;
    //	else
    //
}


void Autopilot_Interface::request_param_list(void)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    mavlink_param_ext_request_list_t param_list;
    param_list.target_system = system_id;
    param_list.target_component = companion_id;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_param_ext_request_list_encode(system_id, companion_id, &message, &param_list);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);
    cout << "request_param_list msg wrote!" << endl;

    // check the write
    if (len <= 0)
        // fprintf(stderr, "WARNING: could not send heartbeat \n");
        cout << "WARNING: could not send heartbeat" << endl;
    //	else
    //

}

// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void Autopilot_Interface::enable_offboard_control()
{
    // Should only send this command once
    if (control_status == false)
    {
        printf("ENABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

        // Sends the command to go off-board
        int success = toggle_offboard_control(true);

        // Check the command was written
        if (success)
            control_status = true;
        else
        {
            fprintf(stderr, "Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if not offboard_status
}

// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void Autopilot_Interface::disable_offboard_control()
{

    // Should only send this command once
    if (control_status == true)
    {
        printf("DISABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

        // Sends the command to stop off-board
        int success = toggle_offboard_control(false);

        // Check the command was written
        if (success)
            control_status = false;
        else
        {
            fprintf(stderr, "Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if offboard_status
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int Autopilot_Interface::toggle_offboard_control(bool flag)
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = {0};
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation = true;
    com.param1 = (float)flag; // flag >0.5 => start, <0.5 => stop

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    int len = serial_port->write_message(message);

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   ARM the vehicle
// ------------------------------------------------------------------------------
void Autopilot_Interface::vehicle_arm()
{
    // Prepare command for off-board mode
    mavlink_command_long_t cmd = {0};
    cmd.target_system = system_id;
    cmd.target_component = autopilot_id;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 1.0;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &cmd);

    // Send the message
    int len = serial_port->write_message(message);

    if (0 > len)
    {
        //
    }
}

// ------------------------------------------------------------------------------
//   DISARM the vehicle
// ------------------------------------------------------------------------------
void Autopilot_Interface::vehicle_disarm()
{
    // Prepare command for off-board mode
    mavlink_command_long_t cmd = {0};
    cmd.target_system = system_id;
    cmd.target_component = autopilot_id;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 0;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &cmd);

    // Send the message
    int len = serial_port->write_message(message);

    if (0 > len)
    {
        //
    }
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void Autopilot_Interface::start()
{
    int result;

    // --------------------------------------------------------------------------
    //   CHECK SERIAL PORT
    // --------------------------------------------------------------------------

    if (serial_port->status != 1) // SERIAL_PORT_OPEN
    {
        fprintf(stderr, "ERROR: serial port not open\n");
        throw 1;
    }

    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------

    cout << "START READ THREAD." << endl;

    std::thread autopilot_interface_read_thread(&Autopilot_Interface::start_read_thread, this);
    autopilot_interface_read_thread.detach();
    // result = pthread_create(&read_tid, NULL, &start_autopilot_interface_read_thread, this);
    // if (result)
    // 	throw result;

    // now we're reading messages
    cout << endl;

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------

    cout << "CHECK FOR MESSAGES." << endl;

    while (not current_messages.sysid)
    {
        if (time_to_exit)
            return;
        usleep(500000); // check at 2Hz
    }

    cout << "Found!" << endl;

    // now we know autopilot is sending messages
    cout << endl;

    // --------------------------------------------------------------------------
    //   GET SYSTEM and COMPONENT IDs
    // --------------------------------------------------------------------------

    // This comes from the heartbeat, which in theory should only come from
    // the autopilot we're directly connected to it.  If there is more than one
    // vehicle then we can't expect to discover id's like this.
    // In which case set the id's manually.

    // System ID
    if (not system_id)
    {
        system_id = current_messages.sysid;
        cout << "GOT VEHICLE SYSTEM ID: " << system_id << "." << endl;
    }

    // Component ID
    if (not autopilot_id)
    {
        autopilot_id = current_messages.compid;
        cout << "GOT AUTOPILOT COMPONENT ID: " << autopilot_id << endl
             << endl;
    }

    // // no LOCAL_POSITION_NED msg for ArduSub
    // // --------------------------------------------------------------------------
    // //   GET INITIAL POSITION
    // // --------------------------------------------------------------------------

    // // Wait for initial position ned
    // while (not(current_messages.time_stamps.local_position_ned &&
    // 		   current_messages.time_stamps.attitude))
    // {
    // 	if (time_to_exit)
    // 		return;
    // 	usleep(500000);
    // }

    // // copy initial position ned
    // Mavlink_Messages local_data = current_messages;
    // initial_position.x = local_data.local_position_ned.x;
    // initial_position.y = local_data.local_position_ned.y;
    // initial_position.z = local_data.local_position_ned.z;
    // initial_position.vx = local_data.local_position_ned.vx;
    // initial_position.vy = local_data.local_position_ned.vy;
    // initial_position.vz = local_data.local_position_ned.vz;
    // initial_position.yaw = local_data.attitude.yaw;
    // initial_position.yaw_rate = local_data.attitude.yawspeed;

    // printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
    // printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
    // printf("\n");

    // // we need this before starting the write thread

    // --------------------------------------------------------------------------
    //   HEARTBEAT WRITE THREAD
    // --------------------------------------------------------------------------
    cout << "START WRITE HEARTBEAT THREAD." << endl;

    std::thread autopilot_interface_write_heartbeat_thread(&Autopilot_Interface::start_write_heartbeat_thread, this);
    autopilot_interface_write_heartbeat_thread.detach();

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    cout << "START WRITE THREAD." << endl;

    std::thread autopilot_interface_write_thread(&Autopilot_Interface::start_write_thread, this);
    autopilot_interface_write_thread.detach();
    // result = pthread_create(&write_tid, NULL, &start_autopilot_interface_write_thread, this);
    // if (result)
    // 	throw result;

    // wait for it to be started
    while (not writing_status)
        usleep(100000); // 10Hz

    // now we're streaming setpoint commands
    printf("\n");

    // Done!
    return;
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void Autopilot_Interface::stop()
{
    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------
    cout << "CLOSE THREADS " << endl;

    // signal exit
    time_to_exit = true;

    // wait for exit
    // pthread_join(read_tid, NULL);
    // pthread_join(write_tid, NULL);
    // autopilot_interface_read_thread.join();
    // autopilot_interface_read_thread.join();

    // now the read and write threads are closed
    printf("\n");

    // still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_read_thread()
{

    if (reading_status != 0)
    {
        fprintf(stderr, "read thread already running\n");
        return;
    }
    else
    {
        read_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Start Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_write_thread(void)
{
    if (writing_status == true)
    {
        fprintf(stderr, "write thread already running\n");
        return;
    }

    else
    {
        write_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Start Write heartbeat Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_write_heartbeat_thread(void)
{
    if (heartbeat_writing_status == true)
    {
        fprintf(stderr, "write heartbeat thread already running\n");
        return;
    }

    else
    {
        write_heartbeat_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void Autopilot_Interface::handle_quit(int sig)
{

    disable_offboard_control();

    try
    {
        stop();
    }
    catch (int error)
    {
        fprintf(stderr, "Warning, could not stop autopilot interface\n");
    }
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_thread()
{
    reading_status = true;

    while (!time_to_exit)
    {
        read_messages();
        usleep(100000); // Read batches at 10Hz
    }

    reading_status = false;

    return;
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_thread(void)
{
    // signal startup
    writing_status = 2;

    // prepare an initial setpoint, just stay put
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
                   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.vx = 0.0;
    sp.vy = 0.0;
    sp.vz = 0.0;
    sp.yaw_rate = 0.0;

    // set position target
    current_setpoint = sp;

    // write a message and signal writing
    write_setpoint();
    writing_status = true;

    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while (!time_to_exit)
    {
        usleep(250000); // Stream at 4Hz
        write_setpoint();
    }

    // signal end
    writing_status = false;

    return;
}

// ------------------------------------------------------------------------------
//   Write heartbeat Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_heartbeat_thread(void)
{
    // signal startup
    heartbeat_writing_status = 2;

    mavlink_heartbeat_t hb;
    hb.type = current_messages.heartbeat.type;
    hb.autopilot = current_messages.heartbeat.autopilot;
    hb.base_mode = current_messages.heartbeat.base_mode;
    hb.custom_mode = current_messages.heartbeat.custom_mode;
    hb.system_status = current_messages.heartbeat.system_status;
    hb.mavlink_version = current_messages.heartbeat.mavlink_version;

    // write a message and signal writing
    write_heartbeat(hb);
    heartbeat_writing_status = true;

    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while (!time_to_exit)
    {
        usleep(500000); // Stream at 4Hz
        write_heartbeat(hb);
    }

    // signal end
    heartbeat_writing_status = false;

    return;
}

// End Autopilot_Interface

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void *start_autopilot_interface_read_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_read_thread();

    // done!
    return NULL;
}

void *start_autopilot_interface_write_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_write_thread();

    // done!
    return NULL;
}

void *start_autopilot_interface_write_heartbeat_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_write_heartbeat_thread();

    // done!
    return NULL;
}

#ifdef WIN32
#include <windows.h>

void usleep(long usec)
{
    // HANDLE timer;
    // LARGE_INTEGER ft;

    // ft.QuadPart = -(10*usec); // Convert to 100 nanosecond interval, negative value indicates relative time

    // timer = CreateWaitableTimer(NULL, TRUE, NULL);
    // SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
    // WaitForSingleObject(timer, INFINITE);
    // CloseHandle(timer);
    DWORD dwMilliseconds;
    dwMilliseconds = DWORD(usec / 1000);
    if (0 == dwMilliseconds)
    {
        Sleep(1);
    }
    else
    {
        Sleep(dwMilliseconds);
    }
}
#endif
