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
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top(int argc, char **argv)
{

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------

    // Default input arguments
#ifdef __APPLE__
    char *uart_name = (char *)"/dev/tty.usbmodem1";
#elif WIN32
    char *uart_name = (char *)"COM4";
#else
    char *uart_name = (char *)"/dev/ttyUSB0";
#endif
    int baudrate = 57600;

    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate);

    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */
    Serial_Port serial_port(uart_name, baudrate);
    // serial_port.debug = true;

    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Autopilot_Interface autopilot_interface(&serial_port);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit = &serial_port;
    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT, quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();
    autopilot_interface.start();

    // --------------------------------------------------------------------------
    //   RUN COMMANDS
    // --------------------------------------------------------------------------

    /*
     * Now we can implement the algorithm we want on top of the autopilot interface
     */
    commands(autopilot_interface);

    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    autopilot_interface.stop();
    serial_port.stop();

    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;
}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void commands(Autopilot_Interface &api)
{

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------

    // Command UNKNOWN/UNSUPPORTED for AuduSub
    // api.enable_offboard_control();
    // usleep(100); // give some time to let it sink in

    // now the autopilot is accepting setpoint commands

    // --------------------------------------------------------------------------
    //   SEND OFFBOARD COMMANDS
    // --------------------------------------------------------------------------
    // cout << "SEND OFFBOARD COMMANDS" << endl;

    // initialize command data strtuctures
    // mavlink_set_position_target_local_ned_t sp;
    // mavlink_set_position_target_local_ned_t ip = api.initial_position;

    // autopilot_interface.h provides some helper functions to build the command

    // Example 1 - Set Velocity
    // can not set velocity for ArduSub
    //	set_velocity( -1.0       , // [m/s]
    //				  -1.0       , // [m/s]
    //				   0.0       , // [m/s]
    //				   sp        );

    // Example 2 - Set Position
    // can not set Position for ArduSub
    // set_position(ip.x - 5.0, // [m]
    // 			 ip.y - 5.0, // [m]
    // 			 ip.z,		 // [m]
    // 			 sp);

    // Example 1.2 - Append Yaw Command
    // set_yaw(ip.yaw, // [rad]
    // 		sp);

    // SEND THE COMMAND
    // no setpoint for ArduSub
    // api.update_setpoint(sp);
    // NOW pixhawk will try to move

    // Wait for 8 seconds, check position
    // no position for ArduSub
    // for (int i = 0; i < 8; i++)
    // {
    // 	mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
    // 	printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
    // 	sleep(1);
    // }
    // api.request_param_list();
    // usleep(long(1000000));



    api.vehicle_arm();
    usleep(long(1000000));
    // api.do_motor_test(0, 1700.0);
    api.do_set_servo(1.0, 1501.0);
    // usleep(long(10000));
    // api.do_motor_test(1.0, 1700.0);
    // usleep(long(10000));
    // api.do_motor_test(2.0, 1502.0);
    // usleep(long(10000));
    // api.do_motor_test(3.0, 1503.0);
    // usleep(long(10000));
    // api.do_motor_test(4.0, 1504.0);
    // usleep(long(10000));
    // api.do_motor_test(5.0, 1505.0);
    // usleep(long(10000));
    // api.do_motor_test(6.0, 1506.0);
    // usleep(long(10000));
    // api.do_motor_test(7.0, 1507.0);
    // usleep(long(10000));
    api.do_motor_test(1, 1700);
    usleep(long(10000));
    //api.vehicle_disarm();
    usleep(long(1000000));
    // Wait for 8 seconds

    for (int i = 0; i < 8; i++)
    {
        cout << "wait for " << 8 - i << " second." << endl;
        sleep(1);
    }

    cout << endl;
    // --------------------------------------------------------------------------
    //   STOP OFFBOARD MODE
    // --------------------------------------------------------------------------

    // Command UNKNOWN/UNSUPPORTED for AuduSub
    // api.disable_offboard_control();

    // now pixhawk isn't listening to setpoint commands

    // --------------------------------------------------------------------------
    //   GET A MESSAGE
    // --------------------------------------------------------------------------
    cout << "READ SOME MESSAGES " << endl;

    // copy current messages
    Mavlink_Messages messages = api.current_messages;

    // // no local position for ArduSub
    // // local position in ned frame
    // mavlink_local_position_ned_t pos = messages.local_position_ned;
    // printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
    // printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z);

    // // no hires imu for ArduSub
    // // hires imu
    // mavlink_highres_imu_t imu = messages.highres_imu;
    // printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
    // printf("    ap time:     %llu \n", imu.time_usec);
    // printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
    // printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
    // printf("    mag  (NED):  % f % f % f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
    // printf("    baro:        %f (mBar) \n", imu.abs_pressure);
    // printf("    altitude:    %f (m) \n", imu.pressure_alt);
    // printf("    temperature: %f C \n", imu.temperature);

    // servo output raw
    mavlink_servo_output_raw_t servo_output = messages.servo_output_raw;
    cout << "Got message SERVO_OUTPUT_RAW " << endl;
    cout << "    port(set of 8 outputs = 1 port):  " << servo_output.port << endl;
    cout << "    servo1_raw:                       " << servo_output.servo1_raw << endl;
    cout << "    servo2_raw:                       " << servo_output.servo2_raw << endl;
    cout << "    servo3_raw:                       " << servo_output.servo3_raw << endl;
    cout << "    servo4_raw:                       " << servo_output.servo4_raw << endl;
    cout << "    servo5_raw:                       " << servo_output.servo5_raw << endl;
    cout << "    servo6_raw:                       " << servo_output.servo6_raw << endl;
    cout << "    servo7_raw:                       " << servo_output.servo7_raw << endl;
    cout << "    servo8_raw:                       " << servo_output.servo8_raw << endl << endl;
    cout << "    servo9_raw:                       " << servo_output.servo9_raw << endl;
    cout << "    servo10_raw:                      " << servo_output.servo10_raw << endl;
    cout << "    servo11_raw:                      " << servo_output.servo11_raw << endl;
    cout << "    servo12_raw:                      " << servo_output.servo12_raw << endl;
    cout << "    servo13_raw:                      " << servo_output.servo13_raw << endl;
    cout << "    servo14_raw:                      " << servo_output.servo14_raw << endl << endl;
    api.vehicle_disarm();
    // --------------------------------------------------------------------------
    //   END OF COMMANDS
    // --------------------------------------------------------------------------

    return;
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

    // Read input arguments
    for (int i = 1; i < argc; i++)
    { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)
        {
            if (argc > i + 1)
            {
                uart_name = argv[i + 1];
            }
            else
            {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0)
        {
            if (argc > i + 1)
            {
                baudrate = atoi(argv[i + 1]);
            }
            else
            {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    // end: for each input argument

    // Done!
    return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler(int sig)
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try
    {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error)
    {
    }

    // serial port
    try
    {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error)
    {
    }

    // end program here
    exit(0);
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // This program uses throw, wrap one big try/catch here
    try
    {
        int result = top(argc, argv);
        return result;
    }

    catch (int error)
    {
        fprintf(stderr, "mavlink_control threw exception %i \n", error);
        return error;
    }
}
