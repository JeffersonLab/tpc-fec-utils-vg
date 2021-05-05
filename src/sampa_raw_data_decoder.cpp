// read a file into memory
#include <iostream> // std::cout
#include <fstream>  // std::std::ifstream
#include <stdint.h>
#include <iomanip>
#include <vector>
#include <cmath>
#include <boost/program_options.hpp>

#include "git_info.hpp"

namespace bpo = boost::program_options;

class SampaDecoder
{
  std::string input_file_name_;
  std::string output_file_prefix_;
  uint32_t verbosity_;
  uint32_t num_timebins_;

  // SYNC positions
  const int sync_unknown_ = {-1};
  int sync_low_;
  int sync_high_;
  uint32_t channel_offset_low_ = {0};
  uint32_t channel_offset_high_ = {16};

  int file_size_;

  // vector for SAMPA half-words (5-bit)
  std::vector<uint32_t> sampa_stream_low_;
  std::vector<uint32_t> sampa_stream_high_;
  // array of vectors for the SAMPA data. Each array corresponds to one channel,
  // each entry in the vector corresponds to one time-bin
  std::vector<uint32_t> sampaData[32];

  // statistics
  double mean[32];
  double sdv[32];

public:
  /**
   * Construcor
   */
  SampaDecoder(const std::string& input_file_name, uint32_t num_timebins = 0, uint32_t verbosity = 0) :
      input_file_name_(input_file_name),
      output_file_prefix_(input_file_name_.substr(0, input_file_name_.rfind('.'))),
      verbosity_(verbosity),
      num_timebins_(num_timebins),
      sync_low_(sync_unknown_),
      sync_high_(sync_unknown_),
      file_size_(-1)
    {}

  /**
   * Exception class for decoder error
   */
  class Error : public std::runtime_error {
    public:
      Error(const std::string& what_arg) : std::runtime_error("Decoding error:" + what_arg) {}
  };

  void setOutputFilePrefix(const std::string& s) { output_file_prefix_ = s; }

  int readDataFromFile();

  inline void getSync(bool throw_if_not_found = false)
    {
      sync_low_ = findSync(sampa_stream_low_, 0);
      sync_high_ = findSync(sampa_stream_high_, 0);

      if (throw_if_not_found && (sync_low_ == sync_unknown_))
        throw Error("Unable to detect sync position in low stream");
      if (throw_if_not_found && (sync_high_ == sync_unknown_))
        throw Error("Unable to detect sync position in high stream");

      if (verbosity_) {
        std::cout << "SYNC Stream Low  : " << sync_low_ << std::endl;
        std::cout << "SYNC Stream High : " << sync_high_ << std::endl;
      }
    }

  inline void getAdcValues()
    {
      extractAdcValues(sampa_stream_low_, sync_low_ + 1, channel_offset_low_);
      extractAdcValues(sampa_stream_high_, sync_high_ + 1, channel_offset_high_);
    }

  void calcStats();

  /** Write ADC values to output stream
   */
  void writeAdcValues(std::ostream &os);

  /** Write ADC values to output file
   */
  void writeAdcValues() {
      std::ofstream fout(output_file_prefix_ + "_adc.txt");
      writeAdcValues(fout);
      fout.close();
    }

  /** Write statistics to output stream (normal or in json format)
   */
  void writeStats(std::ostream &os, bool json = false);

  /** Write statistics to output file (normal or in json format)
   */
  void writeStats(bool json = false) {
      std::ofstream fout(output_file_prefix_ + "_stats." + (json ? "json" : "txt"));
      writeStats(fout, json);
      fout.close();
    }

private:
  // GBTx frame data helper struct
  #pragma pack(push)
  #pragma pack(1)
  struct gbt_frame_t {
      uint32_t w3;
      uint32_t w2;
      uint32_t w1;
      uint32_t w0;
    };
  #pragma pack(pop)

  int findSync(const std::vector<uint32_t> &data, int startIndex) const;
  void extractAdcValues(const std::vector<uint32_t> &data, int startPos, uint32_t channel_offset);

  inline uint32_t bit(uint32_t d, uint8_t src, uint8_t trg) const { return ((d & (1u << src)) >> src) << trg; }

  inline void getHalfWords(const gbt_frame_t &gf)
    {
      // extract the 4 halfwords for the higher data stream and insert them into the stream_high vector
      sampa_stream_high_.push_back(bit(gf.w1, 7, 4) | bit(gf.w1, 3, 3) | bit(gf.w0, 31, 2) | bit(gf.w0, 27, 1) | bit(gf.w0, 23, 0));
      sampa_stream_high_.push_back(bit(gf.w1, 6, 4) | bit(gf.w1, 2, 3) | bit(gf.w0, 30, 2) | bit(gf.w0, 26, 1) | bit(gf.w0, 22, 0));
      sampa_stream_high_.push_back(bit(gf.w1, 5, 4) | bit(gf.w1, 1, 3) | bit(gf.w0, 29, 2) | bit(gf.w0, 25, 1) | bit(gf.w0, 21, 0));
      sampa_stream_high_.push_back(bit(gf.w1, 4, 4) | bit(gf.w1, 0, 3) | bit(gf.w0, 28, 2) | bit(gf.w0, 24, 1) | bit(gf.w0, 20, 0));

      // extract the 4 halfwords for the lower data stream and insert them into the stream_low vector
      sampa_stream_low_.push_back(bit(gf.w0, 19, 4) | bit(gf.w0, 15, 3) | bit(gf.w0, 11, 2) | bit(gf.w0, 7, 1) | bit(gf.w0, 3, 0));
      sampa_stream_low_.push_back(bit(gf.w0, 18, 4) | bit(gf.w0, 14, 3) | bit(gf.w0, 10, 2) | bit(gf.w0, 6, 1) | bit(gf.w0, 2, 0));
      sampa_stream_low_.push_back(bit(gf.w0, 17, 4) | bit(gf.w0, 13, 3) | bit(gf.w0, 9, 2) | bit(gf.w0, 5, 1) | bit(gf.w0, 1, 0));
      sampa_stream_low_.push_back(bit(gf.w0, 16, 4) | bit(gf.w0, 12, 3) | bit(gf.w0, 8, 2) | bit(gf.w0, 4, 1) | bit(gf.w0, 0, 0));
    }

};

int SampaDecoder::readDataFromFile()
{
  std::ifstream file;
  file.open(input_file_name_, std::ifstream::binary);
  if (!file.good())
    throw std::runtime_error("Unable to open or access file " + input_file_name_);

  // get length of file in bytes
  file.seekg(0, file.end);
  file_size_ = file.tellg();
  file.seekg(0, file.beg);

  // calculate the limit for reading. Each GBTx Frame contains 16 bytes
  int readLimit = file_size_ / 16;

  // read in the data frame by frame, extract the 5-bit halfwords for
  // the two data streams and store them in the corresponding half-word
  // vectors
  gbt_frame_t gFrame;
  for (int i = 0; i < readLimit; i++) {
    file.read((char *)&gFrame, sizeof(gFrame));
    getHalfWords(gFrame);
  };

  // close the File
  file.close();

  if (verbosity_) {
    std::cout << std::dec
              << "File Name : " << input_file_name_ << "\n"
              << "File size [bytes] : " << file_size_ << "\n"
              << "GBTx frames : " << readLimit << "\n"
              << "SAMPA low stream size  : " << sampa_stream_low_.size() << "\n"
              << "SAMPA high stream size : " << sampa_stream_high_.size() << "\n";
  }

  return 0;
}

/*
  SampaDecoder::findSync
  Loop over the 5-bit half-word stream and search for the SYNC pattern.
  Return the position of the last SYNC pattern value (0x0A) in the stream
*/
int SampaDecoder::findSync(const std::vector<uint32_t> &data, int startIndex) const
{
  // SYNC Pattern. The following sequence in the data is the SYNC pattern.
  // 0 = 0x2B5 and 1 = 0x14A
  const uint32_t SYNC_PATTERN[32] = { 0x015, 0x015, 0x00a, 0x00a, 0x015, 0x015, 0x00a, 0x00a, 0x015, 0x015, 0x00a,
                                      0x00a, 0x015, 0x015, 0x00a, 0x00a, 0x015, 0x015, 0x015, 0x015, 0x00a, 0x00a,
                                      0x00a, 0x00a, 0x015, 0x015, 0x015, 0x015, 0x00A, 0x00A, 0x00A, 0x00A };
  int index = 0;

  for (uint32_t i = 0; i < data.size(); i++) {
    // cout << "Check : " << index << " " << hex << data[i] << " - " << SYNC_PATTERN[index + startIndex] << " ";
    if (data[i] == SYNC_PATTERN[index + startIndex])
      index += 1;
    else
      index = 0;

    if (index == (32 - startIndex))
      return i;
  };
  return sync_unknown_;
}

void SampaDecoder::extractAdcValues(const std::vector<uint32_t> &data, int startPos, uint32_t channel_offset)
{
  uint32_t adc_value = 0x0;

  // calculate the number of full Adc channel entries in the stream (16 channels * 2 HWs)
  int maxSamples = (data.size() - startPos) / 32;

  if (verbosity_)
    std::cout << "Maximum number of samples : " << maxSamples << std::endl;

  // loop over the samples
  for (int numSamples = 0; numSamples < maxSamples; numSamples++) {
    // extract the 16 channels
    int offset = startPos + numSamples * 32;

    for (uint32_t channel = 0; channel < 16; channel++) {
      adc_value = (data[offset + channel * 2 + 1] << 5) + data[offset + channel * 2];
      sampaData[channel + channel_offset].push_back(adc_value);
    };
  };
}

void SampaDecoder::calcStats()
{
  double m = 0;
  double M2 = 0;
  double variance = 0;

  // check if the number of TimeBins doesn't exceed the amount of decoded
  // timebins
  uint32_t sampleLimit = num_timebins_;
  if (num_timebins_ > sampaData[0].size() || num_timebins_ == 0) {
    sampleLimit = sampaData[0].size();
  }

  for (int channel = 0; channel < 32; channel++) {
    m = 0;
    M2 = 0;
    variance = 0;
    for (uint32_t sample = 0; sample < sampleLimit; sample++) {
      double delta = sampaData[channel][sample] - m;
      m += delta / (sample + 1);
      M2 += delta * (sampaData[channel][sample] - m);
      variance = M2 / (sample + 1);
    };
    mean[channel] = m;
    sdv[channel] = sqrt(variance);

    if (verbosity_)
      std::cout << "[" << std::setw(2) << channel << "] : "
                << std::setw(8) << std::setprecision(4) << std::fixed << mean[channel] << "   "
                << std::setw(6) << std::setprecision(4) << std::fixed << sdv[channel]
                << "\n";
  };
}

void SampaDecoder::writeStats(std::ostream &os, bool json)
{
  os << (json ? "{\n" : "")
     << (json ? "\"input_file_name\": " : "File name : ") << "\"" << input_file_name_ << "\"" << (json ? ",\n" : "\n")
     << (json ? "\"input_file_size\": " : "File size : ") << file_size_ << (json ? ",\n" : "\n")
     << (json ? "\"sync_low\": " : "SYNC Low : ") << sync_low_ << (json ? ",\n" : "\n")
     << (json ? "\"sync_high\": " : "SYNC High : ") << sync_high_ << (json ? ",\n" : "\n");

  if (json) {
    os << "\"mean\": [";
    for (int channel = 0; channel < 32; channel++)
      os << std::setw(8) << std::setprecision(4) << std::fixed << mean[channel] << (channel == 31 ? "]" : ", ");

    os << ",\n" << "\"stdev\": [";

    for (int channel = 0; channel < 32; channel++)
      os << std::setw(6) << std::setprecision(4) << std::fixed << sdv[channel] << (channel == 31 ? "]" : ", ");
  }
  else {
    for (int channel = 0; channel < 32; channel++) {
      os << "[ CHA " << std::setw(2) << channel << "] : "
         << std::setw(8) << std::setprecision(4) << std::fixed << mean[channel] << "   "
         << std::setw(6) << std::setprecision(4) << std::fixed << sdv[channel] << std::endl;
    };
  };

  os << (json ? "\n}" : "") << "\n";
}

void SampaDecoder::writeAdcValues(std::ostream &os)
{
  // check if the number of TimeBins doesn't exceed the amount of decoded
  // timebins
  uint32_t sampleLimit = num_timebins_;
  if (num_timebins_ > sampaData[0].size() || num_timebins_ == 0) {
    sampleLimit = sampaData[0].size();
  }

  for (uint32_t sample = 0; sample < sampleLimit; sample++) {
    for (int channel = 0; channel < 32; channel++)
      os << std::dec << std::setw(4) << std::right << sampaData[channel][sample] << " ";
    os << std::endl;
  };
}

int main(int argc, char **argv)
{
  bpo::variables_map vm;
  bpo::options_description opt_general("Usage:\n  " + std::string(argv[0]) + " <cmds/options>\n"
                                       "  Tool will decode the GBTx data for SAMPA 0\n"
                                       "Commands / Options");
  bpo::options_description opt_hidden("");
  bpo::options_description opt_all;
  bpo::positional_options_description opt_pos;

  try
  {
    auto add_option = opt_general.add_options();
    add_option("help,h", "Print this help message");
    add_option("verbose,v", "Be verbose");
    add_option("version", "Print version information");
    add_option("input-file,i", bpo::value<std::string>()->required(), "Specifies input file.");
    add_option("output-prefix,o", bpo::value<std::string>(), "Specify output file prefix (defaults to (dirname+basename) of input-file)");
    add_option("samples,s", bpo::value<uint32_t>()->default_value(0), "Number of samples to decode [0 = complete file]");
    add_option("json", "Output results as json (if applicable)");

    opt_all.add(opt_general).add(opt_hidden);
    bpo::store(bpo::command_line_parser(argc, argv).options(opt_all).positional(opt_pos).run(), vm);

    if (vm.count("help") || argc == 1) {
      std::cout << opt_general << std::endl;
      exit(0);
    }

    if (vm.count("version")) {
      std::cout << GitInfo();
      exit(0);
    }

    bpo::notify(vm);
  }
  catch (bpo::error &e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << opt_general << std::endl;
    exit(1);
  }
  catch (std::exception &e)
  {
    std::cerr << e.what() << ", application will now exit" << std::endl;
    exit(2);
  }

  // Instantiate the SampaDecoder
  SampaDecoder sDec(
      vm["input-file"].as<std::string>(),
      vm["samples"].as<uint32_t>(),
      vm.count("verbose")
    );

  if (vm.count("output-file"))
    sDec.setOutputFilePrefix(vm["output-file"].as<std::string>());

  try {
    // read the input data from file
    sDec.readDataFromFile();

    // check for the SYNC pattern
    sDec.getSync();

    // extract the Adc values
    sDec.getAdcValues();

    // write Adc values to file
    sDec.writeAdcValues();

    // calculate the mean and sdv for each Adc channel
    sDec.calcStats();
    sDec.writeStats(true);

    if (vm.count("json"))
      sDec.writeStats(std::cout, true);
    else {
      std::cout << "-----------------------------------\n"
                << "          Stats                    \n"
                << "-----------------------------------\n";
      sDec.writeStats(std::cout);
    }
  }
  catch (const SampaDecoder::Error& e) {
    std::cout << e.what() << std::endl;
    exit(10);
  }
  catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    exit(100);
  }
  catch (...) {
    std::cerr << "ERROR: Unknown error" << std::endl;
    exit(1000);
  }

  return 0;
}
