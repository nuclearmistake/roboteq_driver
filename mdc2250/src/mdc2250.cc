#include "mdc2250/mdc2250.h"

#include <iostream>
#include <algorithm>
#include <cstdio>

/***** Inline Functions *****/

namespace mdc2250_ {

inline void unparsedMessages(const std::string &token) {
  std::cout << "Unparsed token(" << token.length() << "): ";
  std::cout << token << std::endl;
}

inline void defaultInfoCallback(const std::string &msg) {
  std::cout << "MDC2250 Info: " << msg << std::endl;
}

inline void defaultExceptionCallback(const std::exception &error) {
  std::cerr << "MDC2250 Unhandled Exception: " << error.what();
  std::cerr << std::endl;
  throw(error);
}

inline void printHex(char * data, int length) {
    for(int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
    }
    printf("\n");
}

}

using namespace mdc2250;
using namespace mdc2250_;
using namespace serial;
using namespace serial::utils;

// Tokenizes on carriage return or ACK (\x06)
inline void tokenizer(const std::string &data,
                      std::vector<TokenPtr> &tokens)
{
  // Find the number of \x06 ASCII ACK's
  size_t number_of_acks =
    (size_t) std::count(data.begin(), data.end(), '\x06');
  // Create tokens for each of the acks
  for(size_t i = 0; i < number_of_acks; ++i) {
    tokens.push_back(TokenPtr( new std::string("\x06") ));
  }
  // Split on \r and ack
  typedef std::vector<std::string> find_vector_type;
  find_vector_type t;
  boost::split(t, data, boost::is_any_of("\r\x06"));
  for (find_vector_type::iterator it = t.begin(); it != t.end(); it++) {
    tokens.push_back(TokenPtr( new std::string(*it) ));
  }
}

/***** MDC2250 Class Functions *****/

MDC2250::MDC2250(bool debug_mode) : listener_(1) {
  // Set default callbacks
  this->handle_exc = defaultExceptionCallback;
  this->info = defaultInfoCallback;
  cmd_time = 200; // Default to 15 ms
  this->debug_mode_ = debug_mode;
  if (this->debug_mode_) {
    this->listener_.setDefaultHandler(unparsedMessages);
  }
  this->listener_.setTokenizer(tokenizer);
  this->listener_.setExceptionHandler(this->handle_exc);
  this->connected_ = false;
  this->echo_ = false;
  this->estop_ = false;
}

MDC2250::~MDC2250() {
  if (this->connected_) {
    this->disconnect();
  }
}

bool MDC2250::isConnected() {
	return this->connected_;
}

void MDC2250::connect(std::string port, size_t watchdog_time, bool echo) {
  // Set the port
  this->port_ = port;

  try {
    // Setup and open serial port
    this->info("Set up port");
    this->serial_port_.setPort(port_);
    this->serial_port_.setBaudrate(115200);
    this->info("Set timeout");
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    this->serial_port_.setTimeout(to);
    this->info("Opening...");
    this->serial_port_.open();
    this->info("Opened");

    // Setup filters
    this->info("Set up filters");
    this->setupFilters();

    // Setup and start serial listener
    this->info("Listening...");
    listener_.startListening(this->serial_port_);
  } catch (std::exception &e) {
    throw(ConnectionFailedException(e.what()));
  }

  // Reset the controller to ensure clean setup
  //this->reset();
  this->connected_ = true;

  // Ping the controller for presence
  if (!this->ping()) {
    this->connected_ = false;
    // We didn't receive a ping from the device
    throw(ConnectionFailedException("Failed to get a response "
                                    "from ping.",1));
  }
  else
    this->info("Ping accomplished");

  // Make sure we aren't etopped
  this->detect_emergency_stop_();

  // Set watchdog and echo
  this->setEcho(echo);
  this->setWatchdog(watchdog_time);

  // Get the device version
  {
    this->info("Querying device version");
    std::string res, fail_why;
    if (!this->issueQuery("?$1E",
                          SerialListener::startsWith("$1E="),
                          res,
                          fail_why))
    {
      this->connected_ = false;
      throw(ConnectionFailedException(fail_why));
    }
    // Store the response
    device_string_ = res.substr(4, res.length()-4);
  }

  // Get the control unit type and controller model
  {
    std::string res, fail_why;
    if (!this->issueQuery("?TRN",
                          SerialListener::startsWith("TRN="),
                          res,
                          fail_why))
    {
      this->connected_ = false;
      throw(ConnectionFailedException(fail_why));
    }
    // Parse the response
    size_t trn = res.find("TRN=");
    size_t colon = res.find(":");
    if (trn == std::string::npos || colon == std::string::npos) {
      std::stringstream ss;
      ss << "Invalid ?TRN query response: " << res;
      this->connected_ = false;
      throw(ConnectionFailedException(ss.str()));
    }
    // Report device info
    trn += 4;
    control_unit_ = res.substr(trn, colon-(trn));
    controller_model_ = res.substr(colon+1, res.length()-colon);
  }

  std::stringstream ss;
  ss << "Connected to device " << device_string_ << " with control unit ";
  ss << control_unit_ << " and controller model ";
  ss << controller_model_ << ".";
  this->info(ss.str());
}

void MDC2250::disconnect() {
  if (this->connected_ == false) {
    return;
  }
  // E-stop
  if (this->serial_port_.isOpen()) {
    this->serial_port_.write("!EX\r");
  }
  this->listener_.stopListening();
  this->connected_ = false;
}

bool MDC2250::issueQuery(const std::string &query,
                         serial::utils::ComparatorType comparator,
                         std::string &response, std::string &failure_reason)
{
  // BufferedFilter for response
  BufferedFilterPtr r = this->listener_.createBufferedFilter(comparator);
  // Issue command
  if (!this->_issueCommand(query,failure_reason,"query"))
    return false;
  // If that succeeded, get the response
  response = r->wait(cmd_time);
  if (response == "") {
    // This means we didn't get a response
    std::stringstream error;
    error << "Failed to receive a response for query " << query << ".";
    failure_reason = error.str();
    return false;
  }
  return true;
}

bool MDC2250::issueCommand(const std::string &command,
                           std::string &failure_reason)
{
  if (!this->_issueCommand(command,failure_reason,"command"))
    return false;
  if (ack_filter->wait(cmd_time) == "") {
    // This means we didn't get an ack ('+')
    if (nak_filter->wait(cmd_time) == "") {
      // This means we didn't get a nak either ('-')
      std::stringstream error;
      error << "Command " << command << " received a non-acknowledgement";
      error << " ('-'), which means there was an error with the command";
      error << ".";
      failure_reason = error.str();
      return false;
    }
    std::stringstream error;
    error << "Failed to receive any acknowledgement from the device ";
    error << "for command " << command << ".";
    failure_reason = error.str();
    return false;
  }
  return true;
}

bool MDC2250::ping() {
  this->serial_port_.write("\x05");
  // If the wait command == "", then no response was heard
  std::string temp = this->ping_filter->wait(cmd_time);
  return !temp.empty();
}

void
MDC2250::reset() {
  BufferedFilterPtr fid_filt =
    this->listener_.createBufferedFilter(SerialListener::startsWith("FID="));
  this->serial_port_.write("%RESET 321654987\r");
  fid_filt->wait(2000);
}

void
MDC2250::setWatchdog(size_t timeout) {
  // Create command
  std::stringstream ss;
  ss << "^RWD " << timeout;
  // Issue command
  std::string fail_why;
  if (!this->issueCommand(ss.str(), fail_why)) {
    // Something went wrong
    throw(CommandFailedException("setWatchdog", fail_why));
  }
}

void
MDC2250::setEcho(bool state) {
  // Create command
  std::stringstream ss;
  ss << "^ECHOF ";
  if (state) {
    ss << "0";
  } else {
    ss << "1";
  }
  // Issue command
  std::string fail_why;
  if (!this->issueCommand(ss.str(), fail_why)) {
    // Something went wrong
    throw(CommandFailedException("setEcho", fail_why));
  }
  // Get the controller's state
  this->detect_echo_();
}

void
MDC2250::estop() {
  // Issue Command
  std::string fail_why;
  if (!this->issueCommand("!EX", fail_why)) {
    // Something went wrong
    throw(CommandFailedException("estop", fail_why));
  }
  // Get the resulting state
  this->detect_emergency_stop_();
}

void MDC2250::clearEstop() {
  // Issue Command
  std::string fail_why;
  if (!this->issueCommand("!MG", fail_why)) {
    // Something went wrong
    throw(CommandFailedException("estop", fail_why));
  }
  // Get the resulting state
  this->detect_emergency_stop_();
}

void
MDC2250::setTelemetry(std::string telemetry_queries,
                      size_t period,
                      serial::utils::DataCallback callback)
{
  // Stop the current telemetry if it is running
  std::string fail_why;
  if (!this->_issueCommand("# C", fail_why, "query history")) {
    // Something went wrong
    throw(CommandFailedException("setTelemetry", fail_why));
  }
  // Validate the parameters
  std::vector<std::string> queries;
  boost::split(queries, telemetry_queries, boost::is_any_of(","));
  if (queries.size() == 0) {
    // There were no queries parsed out
    std::stringstream ss;
    ss << "In setTelemetry, telemetry_queries must be a string of ";
    ss << "queries separated by commas, given: " << telemetry_queries;
    throw(std::invalid_argument(ss.str()));
  }
  if (period == 0) {
    // period must be greater than 0
    std::stringstream ss;
    ss << "In setTelemetry, period must be greater than 0, given: ";
    ss << period;
    throw(std::invalid_argument(ss.str()));
  }
  // Remove old filters
  {
    std::vector<serial::utils::FilterPtr>::iterator i;
    for (i = telemetry_filters_.begin(); i != telemetry_filters_.end(); i++)
    {
      this->listener_.removeFilter((*i));
    }
    telemetry_filters_.clear();
  }
  // Run each query once, in order
  std::vector<std::string>::iterator it;
  for (it = queries.begin(); it != queries.end(); ++it) {
    // Wait to ensure we don't overload the mc
    this->listener_.sleep(100);
    // Issue the query once
    std::string cmd, match, res, fail_why;
    cmd = "?"+(*it);
    match = (*it)+"=";
    if (!issueQuery(cmd, SerialListener::startsWith(match), res, fail_why)) {
      // Something went wrong
      std::vector<serial::utils::FilterPtr>::iterator i;
      for (i = telemetry_filters_.begin(); i != telemetry_filters_.end(); i++)
      {
        this->listener_.removeFilter((*i));
      }
      telemetry_filters_.clear();
      this->serial_port_.write("# C\r");
      throw(CommandFailedException("setTelemetry", fail_why));
    }
  }
  // Now make filters
  std::vector<std::string> key;
  for (it = queries.begin(); it != queries.end(); ++it) {
    // Make sure there are no duplicate filters
    if (std::find(key.begin(), key.end(), (*it)) == key.end()) {
      // Not a filter for it yet
      key.push_back((*it));
      // Add a fitler for it
      telemetry_filters_.push_back(
        listener_.createFilter(SerialListener::startsWith((*it)), callback));
    }
  }
  // Now that all of the queries have run once and filters have been made
  // Call the automatic telemetry sending
  std::stringstream ss;
  ss << "# " << period;
  if (!this->_issueCommand(ss.str(), fail_why, "query history")) {
    // Something went wrong
    throw(CommandFailedException("setTelemetry", fail_why));
  }
}

void
MDC2250::commandMotor(size_t motor_index, ssize_t motor_effort) {
  // Validate the parameters
  if (motor_index != 1 && motor_index != 2) {
    // Invalid motor index, must be 1 or 2
    std::stringstream ss;
    ss << "In commandMotor, motor_index must be 1 or 2, given: ";
    ss << motor_index;
    throw(std::invalid_argument(ss.str()));
  }
  if (motor_effort < -1000 || motor_effort > 1000) {
    // Invalid motor effort, must be between -1000 and 1000 inclusive
    std::stringstream ss;
    ss << "In commandMotor, motor_effort must be between -1000 and ";
    ss << "1000 inclusively, given: " << motor_effort;
    throw(std::invalid_argument(ss.str()));
  }
  // Build the command
  std::stringstream ss;
  ss << "!G " << motor_index << " " << motor_effort;
  // Issue the command
  std::string fail_why;
  if (!this->issueCommand(ss.str(), fail_why)) {
    // Something went wrong
    throw(CommandFailedException("commandMotor", fail_why));
  }
}

void
MDC2250::commandMotors(ssize_t motor1_effort, ssize_t motor2_effort) {
  // Validate parameters
  if (motor1_effort < -1000 || motor1_effort > 1000) {
    // motor1_effort is not valid
    std::stringstream ss;
    ss << "In commandMotors, motor1_effort must be between -1000 and ";
    ss << "1000 inclusively, given: " << motor1_effort;
    throw(std::invalid_argument(ss.str()));
  }
  if (motor2_effort < -1000 || motor2_effort > 1000) {
    // motor2_effort is not valid
    std::stringstream ss;
    ss << "In commandMotors, motor2_effort must be between -1000 and ";
    ss << "1000 inclusively, given: " << motor2_effort;
    throw(std::invalid_argument(ss.str()));
  }
  // Build the command
  std::stringstream ss;
  ss << "!M " << motor1_effort << " " << motor2_effort;
  // Issue the command
  std::string fail_why;
  if (!this->issueCommand(ss.str(), fail_why)) {
    // Something went wrong
    throw(CommandFailedException("commandMotors", fail_why));
  }
}

bool MDC2250::_issueCommand(const std::string &command,
                            std::string &failure_reason,
                            const std::string &cmd_type)
{
  if (!this->connected_) {
    failure_reason = "Not connected.";
    return false;
  }
  if (this->echo_) {
    // BufferedFilter for echo of command
    BufferedFilterPtr e =
      this->listener_.createBufferedFilter(SerialListener::exactly(command));
    // Send the command
    this->serial_port_.write(command+"\r");
    // Wait for the echo of the command
    if (e->wait(cmd_time).empty()) {
      // This means we didn't see it
      std::stringstream error;
      error << "Failed to get " << command << " " << cmd_type << " echo.";
      failure_reason = error.str();
      return false;
    }
  } else {
    // Send the command
    this->serial_port_.write(command+"\r");
  }
  return true;
}

void MDC2250::setupFilters() {
  this->ack_filter =
    this->listener_.createBufferedFilter(SerialListener::exactly("+"));
  this->nak_filter =
    this->listener_.createBufferedFilter(SerialListener::exactly("-"));
  this->ping_filter =
    this->listener_.createBufferedFilter(SerialListener::exactly("\x06"));
}

void MDC2250::detect_echo_() {
  BufferedFilterPtr echo_setting_filt =
  this->listener_.createBufferedFilter(SerialListener::startsWith("ECHOF="));
  this->serial_port_.write("~ECHOF\r");
  std::string echo_setting_res = echo_setting_filt->wait(cmd_time);
  if (echo_setting_res.empty()) {
    // Something went wrong
    throw(CommandFailedException("detect_echo_", "No echo state response."));
  }
  if (echo_setting_res.find('0') != std::string::npos) {
    this->echo_ = true;
    this->info("Echo is enabled.");
  } else {
    this->echo_ = false;
    this->info("Echo is disabled.");
  }
}

void MDC2250::detect_emergency_stop_() {
  BufferedFilterPtr estop_filt =
    this->listener_.createBufferedFilter(SerialListener::startsWith("FF="));
  this->serial_port_.write("?FF\r");
  std::string estop_res = estop_filt->wait(cmd_time);
  if (estop_res.empty()) {
    // Something went wrong
    throw(CommandFailedException("detect_echo_", "No echo state response."));
  }
  if (estop_res.find("16") != std::string::npos) {
    this->estop_ = true;
    this->info("Estop is enabled.");
  } else {
    this->estop_ = false;
    this->info("Estop is disabled.");
  }
}
