#include <iostream>

#include <mdc2250/mdc2250.h>
#include <mdc2250/decode.h>

void telemetry_callback(const std::string &telemetry) {
  std::cout << "Got telemetry: " << telemetry << std::endl;
}

int run(int argc, char** argv) {
  mdc2250::MDC2250 my_mdc2250(true);
  if (argc > 1)
        my_mdc2250.connect(argv[1]);
  else
        my_mdc2250.connect("/dev/ttyACM0");
  // Disable echo
  my_mdc2250.setEcho(true);
  // Disable watchdog
  my_mdc2250.setWatchdog(10000);

  // Setup telemetry
  size_t period = 25;
  my_mdc2250.setTelemetry("C,V,C,A", period, telemetry_callback);
  my_mdc2250.setTelemetry("C,V,C,A", period, telemetry_callback);

  // Move both motors for 4 seconds
  my_mdc2250.commandMotor(1, 1000);
  my_mdc2250.commandMotor(2, -1000);
  boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
  // Stop both motors for 1 second
  my_mdc2250.commandMotor(1);
  my_mdc2250.commandMotor(2, 0);
  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
  // Move both motors for 1 second, but estop (they shouldn't move)
  my_mdc2250.commandMotors(-1000, 1000);
  std::cout << "E-stopping!" << std::endl;
  my_mdc2250.estop();
  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
  // Stop both motors for 1 second, and clear the estop
  my_mdc2250.commandMotors();
  my_mdc2250.commandMotors(0); // Same thing
  my_mdc2250.clearEstop();
  // Have to redo telemetry after an estop
  my_mdc2250.setTelemetry("C,V,C,A", period, telemetry_callback);
  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
  // Move both motors for 4 seconds
  my_mdc2250.commandMotors(-1000, 1000);
  boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
  // Stop both motors
  my_mdc2250.commandMotors();

  return 0;
}

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    return 1;
  }
}
