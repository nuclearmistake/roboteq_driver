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

#ifndef MDC2250_DECODE_H
#define MDC2250_DECODE_H

// Standard Library Headers
#include <string>
#include <cstdlib>

// Boost headers
#include <boost/algorithm/string.hpp>

namespace mdc2250 {

namespace queries {
  /* 
   * This is an enumeration of the possible types of response from queries.
   * 
   * These are listed in the order that they appear in the manual, starting on 
   * page 99.
   */
  typedef enum {
    motor_amps,
    analog_input,
    battery_amps,
    brushless_motor_speed_rpm,
    brushless_motor_speed_percent,
    encoder_count_absolute,
    brushless_encoder_count_absolute,
    brushless_encoder_count_relative,
    internal_analog,
    internal_pulse,
    internal_serial,
    encoder_count_relative,
    digital_inputs,
    individual_digital_inputs,
    digital_output_status,
    closed_loop_error,
    feedback_in,
    fault_flag,
    firmware_id,
    status_flag,
    lock_status,
    motor_command_applied,
    motor_power_output_applied,
    pulse_input,
    encoder_speed_rpm,
    encoder_speed_relative,
    temperature,
    read_time,
    control_unit_type_and_controller_model,
    volts,
    user_variable,
    unknown,
    any_query
  } QueryType;
} // queries namespace

bool
starts_with(const std::string str, const std::string prefix) {
  return str.substr(0,prefix.length()) == prefix;
}

queries::QueryType
detect_response_type(const std::string &raw) {
  using namespace queries;
  if (raw.empty()) {
    std::cerr << "In detect_response_type: Got an empty string." << std::endl;
    return unknown;
  }
  switch(raw[0]) {
    case 'A':
      if (starts_with(raw, "A=")) return motor_amps;
      if (starts_with(raw, "AI=")) return analog_input;
      break;
    case 'B':
      if (starts_with(raw, "BA=")) return battery_amps;
      if (starts_with(raw, "BS=")) return brushless_motor_speed_rpm;
      if (starts_with(raw, "BSR=")) return brushless_motor_speed_percent;
      break;
    case 'C':
      if (starts_with(raw, "C=")) return encoder_count_absolute;
      if (starts_with(raw, "CB=")) return brushless_encoder_count_absolute;
      if (starts_with(raw, "CBR=")) return brushless_encoder_count_relative;
      if (starts_with(raw, "CIA=")) return internal_analog;
      if (starts_with(raw, "CIP=")) return internal_pulse;
      if (starts_with(raw, "CIS=")) return internal_serial;
      if (starts_with(raw, "CR=")) return encoder_count_relative;
      break;
    case 'D':
      if (starts_with(raw, "D=")) return digital_inputs;
      if (starts_with(raw, "DI=")) return individual_digital_inputs;
      if (starts_with(raw, "DO=")) return digital_output_status;
      break;
    case 'E':
      if (starts_with(raw, "E=")) return closed_loop_error;
      break;
    case 'F':
      if (starts_with(raw, "F=")) return feedback_in;
      if (starts_with(raw, "FF=")) return fault_flag;
      if (starts_with(raw, "FID=")) return firmware_id;
      if (starts_with(raw, "FS=")) return status_flag;
      break;
    case 'L':
      if (starts_with(raw, "LK=")) return lock_status;
      break;
    case 'M':
      if (starts_with(raw, "M=")) return motor_command_applied;
      break;
    case 'P':
      if (starts_with(raw, "P=")) return motor_power_output_applied;
      if (starts_with(raw, "PI=")) return pulse_input;
      break;
    case 'S':
      if (starts_with(raw, "S=")) return encoder_speed_rpm;
      if (starts_with(raw, "SR=")) return encoder_speed_relative;
      break;
    case 'T':
      if (starts_with(raw, "T=")) return temperature;
      if (starts_with(raw, "TM=")) return read_time;
      if (starts_with(raw, "TRN=")) {
        return control_unit_type_and_controller_model;
      }
      break;
    case 'V':
      if (starts_with(raw, "V=")) return volts;
      if (starts_with(raw, "VAR=")) return user_variable;
      break;
    default:
      break;
  }
  return unknown;
}

/*!
 * Returns the corresponding std::string given a QueryType.
 */
std::string
response_type_to_string(queries::QueryType res) {
  using namespace queries;
  switch (res) {
    case motor_amps: return "motor_amps";
    case analog_input: return "analog_input";
    case battery_amps: return "battery_amps";
    case brushless_motor_speed_rpm: return "brushless_motor_speed_rpm";
    case brushless_motor_speed_percent:
      return "brushless_motor_speed_percent";
    case encoder_count_absolute: return "encoder_count_absolute";
    case brushless_encoder_count_absolute:
      return "brushless_encoder_count_absolute";
    case brushless_encoder_count_relative:
      return "brushless_encoder_count_relative";
    case internal_analog: return "internal_analog";
    case internal_pulse: return "internal_pulse";
    case internal_serial: return "internal_serial";
    case encoder_count_relative: return "encoder_count_relative";
    case digital_inputs: return "digital_inputs";
    case individual_digital_inputs: return "individual_digital_inputs";
    case digital_output_status: return "digital_output_status";
    case closed_loop_error: return "closed_loop_error";
    case feedback_in: return "feedback_in";
    case fault_flag: return "fault_flag";
    case firmware_id: return "firmware_id";
    case status_flag: return "status_flag";
    case lock_status: return "lock_status";
    case motor_command_applied: return "motor_command_applied";
    case motor_power_output_applied: return "motor_power_output_applied";
    case pulse_input: return "pulse_input";
    case encoder_speed_rpm: return "encoder_speed_rpm";
    case encoder_speed_relative: return "encoder_speed_relative";
    case temperature: return "temperature";
    case read_time: return "read_time";
    case control_unit_type_and_controller_model:
      return "control_unit_type_and_controller_model";
    case volts: return "volts";
    case user_variable: return "user_variable";
    case unknown: return "unknown";
    default: break;
  }
  return "unknown";
}

/*!
 * Exception called when a problem occurs while parsing some mdc2250 response.
 */
class DecodingException : public std::exception {
  const std::string e_what_;
  const std::string raw_;
  const queries::QueryType res_;
public:
  DecodingException(const std::string &e_what = "",
                    const std::string &raw = "",
                    queries::QueryType res = queries::unknown)
  : e_what_(e_what), raw_(raw), res_(res) {}
  ~DecodingException() throw() {}

  virtual const char * what() const throw() {
    std::stringstream ss;
    ss << "Failed to decode `" << this->raw_ << "` as a ";
    ss << response_type_to_string(this->res_) << ": " << this->e_what_;
    return ss.str().c_str();
  }
};

/*
 * Decodes any response from the MDC2250 into a list of longs.
 * 
 * \params raw The raw data from the motor controller.
 * \params channels A std::vector of longs that can vary in length based on 
 * the response and motor controller.  Many will contain 1 or 2, but some have 
 * many (like Read Analog Inputs).  See the mdc2250 manual for more 
 * information reguarding query responses.
 * 
 * \returns size_t The number of elements in channels.
 * 
 * \throws mdc2250::DecodingException
 */
size_t
decode_generic_response(const std::string &raw, std::vector<long> &channels) {
  queries::QueryType res = detect_response_type(raw);
  if (res == queries::unknown) {
    throw(DecodingException("unknown response type", raw, res));
  }
  std::vector<std::string> strs;
  boost::split(strs, raw, boost::is_any_of("=:"));
  if (strs.size() < 2) {
    throw(DecodingException("the format is invalid", raw, res));
  }
  strs.erase(strs.begin()); // Erase the stuff before the '='
  std::vector<std::string>::iterator it;
  for (it = strs.begin(); it != strs.end(); ++it) {
    channels.push_back(atol((*it).c_str()));
  }
  return channels.size();
}

} // mdc2250 namespace

#endif
