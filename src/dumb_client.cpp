#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <array>

using namespace std::literals;

int main(int argc, char **argv) {
  Poco::Net::SocketAddress sa("localhost", 1234);
  Poco::Net::DatagramSocket dgs;
  dgs.connect(sa);

  std::string start{"1"};
  std::string stop{"0"};

  dgs.sendTo(start.data(), start.size(), sa);
  std::chrono::time_point<std::chrono::system_clock> begin{
      std::chrono::system_clock::now()};

  char buffer[512];

  while ((begin + std::chrono::seconds(5)) > std::chrono::system_clock::now()) {

      //std::cout << "Received " << n << " bytes" << std::endl;
      int n = dgs.receiveFrom(buffer, sizeof(buffer) - 1, sa);
      std::cout << sa.toString() << ": " << buffer << std::endl << std::endl;
      buffer[0] = '\0';
      std::this_thread::sleep_for(500ms);
  }

  dgs.sendTo(stop.data(), stop.size(), sa);

  return 0;
}