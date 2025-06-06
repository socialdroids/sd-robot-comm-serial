/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <serial/serial.h>
#include "feedback.pb.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
	}
}

void print_usage()
{
	cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

int run(int argc, char **argv)
{
  if(argc < 2) {
	  print_usage();
    return 0;
  }

  // Argument 1 is the serial port or enumerate flag
  string port(argv[1]);

  if( port == "-e" ) {
	  enumerate_ports();
	  return 0;
  }
  else if( argc < 3 ) {
	  print_usage();
	  return 1;
  }

  // Argument 2 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[2], "%lu", &baud);
#else
  sscanf(argv[2], "%lu", &baud);
#endif

  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  // Get the Test string
  int count = 0;
  string test_string;
  if (argc == 4) {
    test_string = argv[3];
  } else {
    test_string = "Testing.";
  }

  using std::chrono::high_resolution_clock;
  using std::chrono::duration_cast;
  using std::chrono::duration;
  using std::chrono::milliseconds;
  using std::chrono::microseconds;
  using std::chrono::nanoseconds;


  // // Test the timeout, there should be 1 second between prints
  // cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
  // while (count < 10) {
  //   size_t bytes_wrote = my_serial.write(test_string);
  //
  //   string result = my_serial.read(test_string.length()+1);
  //
  //   cout << "Iteration: " << count << ", Bytes written: ";
  //   cout << bytes_wrote << ", Bytes read: ";
  //   cout << result.length() << ", String read: " << result << endl;
  //
  //   count += 1;
  // }

  // Test the timeout at 250ms
  my_serial.setTimeout(10, 1, 0, 1, 0);
  count = 0;

  auto t1 = high_resolution_clock::now();
  auto tPrint = high_resolution_clock::now();
  auto tLoop = high_resolution_clock::now();

  int i = 0, packetsReceived = 0, packetSize = 0;
  float packetFreq = 0, loopFreq[100];
  string result;
  FeedbackMessage message;
  while (true) {
    // size_t bytes_wrote = my_serial.write(test_string);
    tLoop = high_resolution_clock::now();
    if (my_serial.available() > 0)
    {
      // cout << my_serial.available() << "\n";
      packetsReceived++;
      // packetSize += my_serial.available();
      // result = my_serial.read(my_serial.available());
      result = "";
      packetSize = my_serial.readline(result, 65536, "¬");
      cout << "Data = ";
      for (int n = 0; n < packetSize; ++n) {
        unsigned int num = result.c_str()[n];
        cout << std::hex << num << std::dec << ", ";
      }
      cout << "\n";
      bool ok = message.ParseFromArray(result.c_str(), packetSize);
      // cout << "Iteration: " << count << ", Bytes written: ";
      // cout << bytes_wrote << ", Bytes read: ";
      // cout << "Bytes read: " << result.length();
      packetFreq = 1e9/(duration_cast<nanoseconds>(high_resolution_clock::now() - t1)).count();
      cout << "Packet Size " << packetSize << ", OK? " << ok;
      if (ok)
      {
        if (message.bumpers_size() > 0)
        {
            cout << " | Bumper = " << message.bumpers(0).status() << ", ID = " << message.bumpers(0).id();
        }
        if (message.bumpers_size() > 1)
        {
            cout << " | Bumper = " << message.bumpers(1).status() << ", ID = " << message.bumpers(1).id() ;
        }
        if (message.has_pose()) 
        {
            cout << " | Pose (x,y,z) = " << message.pose().x() << ", " << message.pose().y() << ", " << message.pose().z();
        }
      }
      cout << " | Packet F = " << packetFreq << " Hz \n";

      // count += 1;
      t1 = high_resolution_clock::now();
    }
    if ((duration_cast<milliseconds>(high_resolution_clock::now() - tPrint)).count() > 3000)
    {
      // cout << "Packet F = " << packetFreq << " Hz\n";
      float freqValue = 0;
      for (int n = 0; n < 100; n++)
      {
        freqValue += loopFreq[n];
      }
      freqValue /= 100;
      cout //<< "String read: " << result << "/" << packetSize << ", Packet F = " << packetFreq 
           << " Hz, Loop F = " << freqValue << " Hz, Packets in 3s = " 
           << packetsReceived << ", Packets/s = " << packetsReceived/3.0
           << "\n";
      packetsReceived = 0;
      // packetSize = 0;

      tPrint = high_resolution_clock::now();
    }
      
    loopFreq[i++] = 1e9/(duration_cast<nanoseconds>(high_resolution_clock::now() - tLoop)).count();
    if (i >= 100)
    {
      i = 0;
    }
  }

  // Test the timeout at 250ms, but asking exactly for what was written
  // count = 0;
  // cout << "Timeout == 250ms, asking for exactly what was written." << endl;
  // while (count < 10) {
  //   size_t bytes_wrote = my_serial.write(test_string);
  //
  //   string result = my_serial.read(test_string.length());
  //
  //   cout << "Iteration: " << count << ", Bytes written: ";
  //   cout << bytes_wrote << ", Bytes read: ";
  //   cout << result.length() << ", String read: " << result << endl;
  //
  //   count += 1;
  // }
  //
  // // Test the timeout at 250ms, but asking for 1 less than what was written
  // count = 0;
  // cout << "Timeout == 250ms, asking for 1 less than was written." << endl;
  // while (count < 10) {
  //   size_t bytes_wrote = my_serial.write(test_string);
  //
  //   string result = my_serial.read(test_string.length()-1);
  //
  //   cout << "Iteration: " << count << ", Bytes written: ";
  //   cout << bytes_wrote << ", Bytes read: ";
  //   cout << result.length() << ", String read: " << result << endl;
  //
  //   count += 1;
  // }

  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
