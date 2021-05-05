#include "simple_log.hpp"

const std::string Log::levelNames_[] = {
  "DEBUG",
  "DEBUG",
  "SYS",
  "INFO",
  "WARN",
  "ERROR",
  "FATAL"
};

const std::string Log::levelColors_[] = {
  "\033[0;97m\033[4m",
  "\033[0;97m",
  "\033[0;36m  ",
  "\033[0;32m ",
  "\033[0;33m ",
  "\033[0;31m",
  "\033[1;31m"
};

std::mutex Log::mtx_;
Log::level_t Log::targetLogLevel_(Log::lsys);
bool Log::includeTs_(true);
