#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <boost/program_options.hpp>

#include <librorc.h>
#include "trorc.hpp"

using namespace trorc;
namespace bpo = boost::program_options;

uint32_t conv_hex_string_to_uint32(const std::string& s)
{
  uint32_t n;
  std::stringstream is;
  if (s.find_first_not_of("0123456789") != std::string::npos) {
    if ((s.length() <= 2) ||
        (s.find_first_not_of("xX0123456789abcdefABCDEF") != std::string::npos) ||
        ((s.substr(0, 2) != std::string("0x")) &&
        (s.substr(0, 2) != std::string("0X")))) 
    throw(std::invalid_argument("Illegal number specified: *" + s));
    is << std::hex;
  }

  is << s;
  is >> n;
  return n;
}

int main(int argc, char** argv)
{
  bpo::variables_map vm;
  bpo::options_description opt_general("Usage: trorc_rw <addr> [<data>]\nOptions");
  bpo::options_description opt_hidden("");
  bpo::options_description opt_all;
  bpo::positional_options_description opt_pos;

  try
  {
    opt_general.add_options()
      ("help,h", "Print this help message")
      ("verbose,v", "Be verbose")
      ;
    opt_hidden.add_options()
      ("addr", bpo::value<std::string>()->required(), "Address (r/w)")
      ("data", bpo::value<std::string>(), "Data (w)")
      ;
    opt_pos.add("addr", 1).add("data", 1);
    opt_all.add(opt_general).add(opt_hidden);

    bpo::store(bpo::command_line_parser(argc, argv).options(opt_all)
                     .positional(opt_pos).run(), vm);

    if (vm.count("help")) {
      std::cout << opt_general << std::endl;
      exit(0);
    }

    bpo::notify(vm);
  }
  catch(bpo::error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << opt_general << std::endl;
    exit(1);
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << ", application will now exit" << std::endl;
    exit(2);
  }

  // Convert (dec<->hex) and check input address and data
  uint32_t addr(0);
  uint32_t data(0);

  try {
    addr = conv_hex_string_to_uint32(vm["addr"].as<std::string>());
    if (vm.count("data"))
      data = conv_hex_string_to_uint32(vm["data"].as<std::string>());
  }
  catch (std::invalid_argument& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    exit(1);
  }

  // Device stuff
  int device_number(0);
  int bar_number(1);
  std::unique_ptr<Device> trorc;
  std::unique_ptr<Bar>    bar;

  try {
    trorc.reset(new Device(device_number));
    bar.reset(new Bar(*trorc, bar_number));
  } catch (int e) {
    std::cerr << "ERROR: Failed to initialize T-RORC: " << librorc::errMsg(e)
              << std::endl;
    exit(1);
  }

  // Do something
  if (vm.count("data")) {
    uint32_t v = bar->writeRb(addr, data);
    if (vm.count("verbose")) {
       std::cout << "Writing 0x" << std::hex << data << " [" << std::dec << data << "] to address 0x"
                 << std::hex << addr << "." << std::dec << std::endl;
    }
    std::cout << std::hex << "0x" << v << std::endl;
  }
  else {
    uint32_t v = bar->read(addr);
    std::cout << std::hex << "0x" << v << std::endl;
  }
   

  return 0;
}
