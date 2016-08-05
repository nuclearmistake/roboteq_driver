/*!
 * \file mdc2250/mdc2250.h
 * \author William Woodall <wjwwood@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 William Woodall
 *
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides an interface for the Roboteq MDC2250 Motor Controller.
 * 
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on Serial: https://github.com/wjwwood/serial
 * 
 */

#ifndef MDC2250_H
#define MDC2250_H

// Standard Library Headers
#include <string>
#include <sstream>

// Boost Headers
#include <boost/function.hpp>

#define SERIAL_LISTENER_DEBUG 0

// Serial Headers
#include <serial/serial.h>
#include <serial_utils/serial_listener.h>

namespace mdc2250 {

namespace constants {

typedef enum
{
	openloop=1,
	closedloop_speed=2,
	closedloop_position=3
} COMMAND_MODE;

typedef enum
{
	unused=0,
	feedback=1,
	command=2
} ENCODER_USAGE;

}

/*!
 * This function type describes the prototype for the logging callbacks.
 * 
 * The function takes a std::string reference and returns nothing.  It is 
 * called from the library when a logging message occurs.  This 
 * allows the library user to hook into this and integrate it with their own 
 * logging system.  It can be set with any of the set<log level>Handler 
 * functions.
 * 
 * \see SerialListener::setInfoHandler, SerialListener::setDebugHandler, 
 * SerialListener::setWarningHandler
 */
typedef boost::function<void(const std::string&)> LoggingCallback;

/*!
 * This function type describes the prototype for the exception callback.
 * 
 * The function takes a std::exception reference and returns nothing.  It is 
 * called from the library when an exception occurs in a library thread.
 * This exposes these exceptions to the user so they can to error handling.
 * 
 * \see SerialListener::setExceptionHandler
 */
typedef boost::function<void(const std::exception&)> ExceptionCallback;

/*!
 * Represents an MDC2250 Device and provides and interface to it.
 */
class MDC2250 {
public:
  /*!
   * Constructs the MDC2250 object.
   */
  MDC2250(bool debug_mode = false);
  virtual ~MDC2250();

  bool isConnected();

  /*!
   * Connects to the MDC2250 motor controller given a serial port.
   * 
   * \param port Defines which serial port to connect to in serial mode.
   * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
   * 
   * \param watchdog_time size_t that defines how long the motor controller's
   * watchdog timer should be set to in milliseconds.  A value of 0 will 
   * deisable the watchdog timer completely.  If you do not disable the 
   * watchdog timer then you will have to send motor commands periodically to 
   * prevent the motors from stopping.  Defaults to 1000 ms.
   * 
   * \param echo bool true for the echo to be on and false for it to be off.
   * The echo provides a sortof checksum on commands, but is not necessary and 
   * turning it off can improve performance.  Defaults to on.
   * 
   * \throws ConnectionFailedException connection attempt failed.
   * \throws UnknownErrorCodeException unknown error code returned.
   */
  void connect(std::string port,
               size_t watchdog_time = 1000,
               bool echo = true);

  /*!
   * Disconnects from the MDC2250 motor controller given a serial port.
   */
  void disconnect();

  /*!
   * Takes a std::string query, a Comparator to match the response, a 
   * std::string response for storing the response, a std::string for storing 
   * the reason of a failure, and returns true for success and false for a 
   * failure.
   * 
   * \param query string to send to the mdc2250 (no return carriage needed)
   * \param comparator a comparator function for matching the command 
   * response.
   * \param response response from the device, matched by the comparator.
   * \param failure_reason the reason for a failure, empty if there was no 
   * failure.
   * 
   * \return bool true for success, false for failure.
   */
  bool issueQuery(const std::string &query,
                  serial_utils::ComparatorType comparator,
                  std::string &response,
                  std::string &failure_reason);

  /*!
   * Takes a std::string command, a std::string for storing the reason of a 
   * failure, and returns true for success and false for a failure.
   * 
   * \param query string to send to the mdc2250 (no return carriage needed)
   * \param failure_reason the reason for a failure, empty if there was no 
   * failure.
   * 
   * \return bool true for success, false for failure.
   */
  bool issueCommand(const std::string &command, std::string &failure_reason);

  /*!
   * Sends an ASCII QRY to the controller to check for its presence.
   * 
   * \return bool true for success, false for failure.
   */
  bool ping();

  /*!
   * Resets the controller, this is done on connection.
   */
  void reset();

  /*!
   * Sets the watchdog timer timeout time in milliseconds.
   * 
   * \params timeout size_t time in milliseconds for the timeout.  A value of 
   * 0 will disable the watchdog timer.
   */
  void setWatchdog(size_t timeout);

  /*!
   * Sets the command echoing on or off.
   * 
   * \params state bool true will enable echo, false will disable it.
   */
  void setEcho(bool state);

  /*!
   * Sets the emergency stop, motors won't move until estop is cleared.
   */
  void estop();

  /*!
   * Clears the emergency stop, motors won't move until estop is cleared.
   */
  void clearEstop();

  /*!
   * Returns the estop status, true for estopped, false otherwise.
   * 
   * \returns bool true means it is estopped, false means it is not
   */
  bool isEstopped() {
    // this->detect_emergency_stop_();
    return this->estop_;
  }

  /*!
   * Sets the Telemetry string, rate, and callback.
   * 
   * This allows you to set an order of queries to be preiodically sent
   * automatically by the motor controller and then provided to a callback.
   * 
   * Example: my_mdc2250.setTelemetry("CR,V,CR,A", 25, my_callback);
   *          // This will return CR then wait 25 ms, then V, and so on
   *          //  calling the callback each time
   * 
   * Automatic telemetry will be interrupted by several actions, activating
   * the estop, changing the echo state, or making an arbitrary query.  If you
   * wish to deliberately stop the automatic telemetry, then you can make a
   * call to setTelemetry with an empty string for the telemetry_queries.
   * 
   * \params telemetry_queries std::string that describes the telemetry order 
   * and is a series of queries separated by comma.
   * 
   * \params period size_t period in milliseconds between each telemetry 
   * element being sent by the motor controller.
   * 
   * \params callback serial_utils::DataCallback function to be called when 
   * new telemetry data has arrived.
   */
  void setTelemetry(std::string telemetry_queries,
                    size_t period,
                    serial_utils::DataCallback callback);

  /*!
   * Commands a given motor to a given motor effort.
   * 
   * From the datasheet:
   * <pre>
   *    The G command is used to set the speed or position of a single motor.
   *    The commands are given in values from -1000 to +1000 and represent a
   *    power level in open-loop speed mode, desired speed in percent of max 
   *    RPM in the closed loop speed mode, or a desired relative position in 
   *    the closed-loop position mode.
   *    
   *    Syntax: !G [nn] mm
   *    
   *    Where: nn = Motor Channel. May be omitted in single channel
   *                controllers
   *           mm = command value in +/-1000 range
   *    Examples: G 1 500: set motor1 to 500
   *              G 2 600: set motor2 to 600
   * </pre>
   * 
   * \param motor_index size_t motor channel index 1 or 2
   * \param motor_effort ssize_t value between -1000 and 1000, defaults to 0
   * 
   * \return bool true for success, false for failure.
   * 
   * \throws std::invalid_argument
   */
  void commandMotor(size_t motor_index, ssize_t motor_effort = 0);

  /*!
   * Commands both motors given their respective efforts.
   * 
   * From the datasheet:
   * <pre>
   *    The M command is used to set the speed or position of one or two 
   *    motors at once. The command can include 1 or 2 parameters to set the 
   *    speed of one or both motors from sin- gle command. The commands are 
   *    given in values from -1000 to +1000 and represent a power level in 
   *    open-loop speed mode, desired speed in percent of max RPM in the 
   *    closed-loop speed mode, or a desired relative position in the 
   *    closed-loop position mode. If only one parameter is sent, the value is 
   *    applied to channel 1. When two parameters follow the runtime command, 
   *    they apply to the first and the second channel.
   *    
   *    Syntax: !M nn [mm]
   *    
   *    Where: nn, mm = command value in +/-1000 range
   *    
   *    Examples: !M 500: set motor1 to 500
   *              !M 500 600: set motor1 to 500 and motor2 to 600
   *              !M 0 600: stop motor1 and set motor2 to 600
   *              !M 0 0: stop both motors
   * </pre>
   * 
   * \param motor1_effort ssize_t value between -1000 and 1000, defaults to 0
   * \param motor2_effort ssize_t value between -1000 and 1000, defaults to 0
   * 
   * \return bool true for success, false for failure.
   * 
   * \throws std::invalid_argument
   */
  void commandMotors(ssize_t motor1_effort = 0, ssize_t motor2_effort = 0);

  /*!
   * Sets the function to be called when an info logging message occurs.
   * 
   * This allows you to hook into the message reporting of the library and use
   * your own logging facilities.
   * 
   * The provided function must follow this prototype:
   * <pre>
   *    void yourInfoCallback(const std::string &msg)
   * </pre>
   * Here is an example:
   * <pre>
   *    void yourInfoCallback(const std::string &msg) {
   *        std::cout << "MDC2250 Info: " << msg << std::endl;
   *    }
   * </pre>
   * And the resulting call to make it the callback:
   * <pre>
   *    Object my_object;
   *    my_object.setInfoCallback(yourInfoCallback);
   * </pre>
   * Alternatively you can use a class method as a callback using boost::bind:
   * <pre>
   *    #include <boost/bind.hpp>
   *    
   *    #include "object.h"
   *    
   *    class MyClass
   *    {
   *    public:
   *     MyClass () {
   *      my_object.setInfoHandler(
   *          boost::bind(&MyClass::handleInfo, this, _1));
   *     }
   *    
   *     void handleInfo(const std::string &msg) {
   *       std::cout << "MyClass Info: " << msg << std::endl;
   *     }
   *    
   *    private:
   *     Object object;
   *    };
   * </pre>
   * 
   * \param info_handler A function pointer to the callback to handle new 
   * Info messages.
   * 
   * \see serial_utils::LoggingCallback
   */
  void setInfoHandler(LoggingCallback info_handler) {
    this->info = info_handler;
  }

  /*!
   * Sets the function to be called when an exception occurs internally.
   * 
   * This allows you to hook into the exceptions that occur in threads inside
   * the library.
   * 
   * \param exception_handler A function pointer to the callback to handle new 
   * interal exceptions.
   * 
   * \see mdc2250::ExceptionCallback, MDC2250::setInfoHandler
   */
  void
  setExceptionHandler (ExceptionCallback exception_handler) {
    this->handle_exc = exception_handler;
    // Reset the listener's exception handler
    this->listener_.setExceptionHandler(this->handle_exc);
  }

  void setOperatingMode (int  channel, constants::COMMAND_MODE cm);

  void setMaxRPMValue(int channel, int rpmThatCorrespondsTo1000Effort);

  void setEncoderPulsesPerRotation(int channel, int pulses);

  void setEncoderUsage(int channel, constants::ENCODER_USAGE eu, bool mot1, bool mot2);

  void commitConfig();

private:
  // Implementation of _issueCommand, used by issueQuery too
  bool _issueCommand(const std::string &command, std::string &failure_reason,
                     const std::string &cmd_type);
  // Function to setup commonly used, persistent filters
  void setupFilters();
  // Detects the motor controller's echo state
  void detect_echo_();
  // Detects the motor controller's estop state
  void detect_emergency_stop_();

  // Time to wait for responses from the device
  long cmd_time;

  // Exception callback handle
  ExceptionCallback handle_exc;
  LoggingCallback info;

  // Serial port name
  std::string port_;

  // Device Info
  std::string device_string_;
  std::string control_unit_;
  std::string controller_model_;

  // Serial port and listener
  serial::Serial                serial_port_;
  serial_utils::SerialListener listener_;

  // Fitlers
  serial_utils::BufferedFilterPtr ack_filter;
  serial_utils::BufferedFilterPtr nak_filter;
  serial_utils::BufferedFilterPtr ping_filter;
  std::vector<serial_utils::FilterPtr> telemetry_filters_;

  // Connection state
  bool connected_;

  // Echo setting state
  bool echo_;

  // Estop state
  bool estop_;

  // Debug mode
  bool debug_mode_;
};

/*!
 * Exception called when a connection to the mdc2250 fails for some reason.
 */
class ConnectionFailedException : public std::exception {
  const std::string e_what_;
  int error_type_;
public:
  ConnectionFailedException(const std::string &e_what, int error_type = 0)
  : e_what_(e_what), error_type_(error_type) {}
  ~ConnectionFailedException() throw() {}

  int error_type() {return error_type_;}

  virtual const char * what() const throw() {
    std::stringstream ss;
    ss << "Connecting to the MDC2250: " << this->e_what_;
    return ss.str().c_str();
  }
};

/*!
 * Exception called when a command or query failed.
 */
class CommandFailedException : public std::exception {
  const std::string command_, e_what_;
  int error_type_;
public:
  CommandFailedException(const std::string &command,
                         const std::string &e_what, int error_type = 0)
  : command_(command), e_what_(e_what), error_type_(error_type) {}
  ~CommandFailedException() throw() {}

  int error_type() {return error_type_;}

  virtual const char * what() const throw() {
    std::stringstream ss;
    ss << "Command " << this->command_ << " failed: " << this->e_what_;
    return ss.str().c_str();
  }
};

}

#endif
