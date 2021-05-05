#pragma once

#include <string>
#include <iostream>
#include <ostream>
#include <vector>
#include <boost/program_options.hpp>
#include <boost/fusion/sequence.hpp>
#include <boost/fusion/include/sequence.hpp>

#include "factory.hpp"
#include "git_info.hpp"

namespace bpo = boost::program_options;

namespace common {

/** Common command line options and factory
 *    to somewhat unify command-line options across the different binaries
 *
 *  We could use macros to generate these, but explicit is a bit easier to grep
 */
class CommonOptions
{
  private:
    inline bool getFlag(const std::string& key)
      {
        return vm_.count(key) > 0;
      }

  protected:
    bpo::variables_map vm_;

  public:
    /** Get reference to variables map
     */
    inline auto& getVariablesMap() const { return vm_; }

    /* Help */
    inline auto addHelp(bpo::options_description& od)
      {
        return od.add_options()("help,h", "Print this help message");
      }
    inline auto getHelpFlag() { return getFlag("help"); }

    /* Verbosity */
    inline auto addVerbose(bpo::options_description& od)
      {
        return od.add_options()("verbose,v", "Be verbose");
      }
    inline auto getVerboseFlag() { return getFlag("verbose"); }

    /* Version */
    inline auto addVersion(bpo::options_description& od)
      {
        return od.add_options()("version", "Display version information");
      }
    inline auto getVersionFlag() { return getFlag("version"); }

    /* Card Id */
    inline auto addId(bpo::options_description& od)
      {
        return od.add_options()("id", bpo::value<std::string>()->default_value(common::OptionDefaultsFactory::id),
                                (std::string("Specify card id ") + common::OptionDefaultsFactory::id_help).c_str());
      }
    inline auto getId() { return vm_["id"].as<std::string>(); }

    /* Bar */
    inline auto addBar(bpo::options_description& od)
      {
        return od.add_options()("bar,b", bpo::value<cl_uint_t>()->default_value(common::OptionDefaultsFactory::bar),
                                "Specify the base address register");
      }
    inline auto getBar() { return vm_["bar"].as<cl_uint_t>()(); }

    /* FEC mask */
    inline auto addFecMask(bpo::options_description& od)
      {
        return od.add_options()("mask,m", bpo::value<cl_uint_t>()->default_value(0x1),
                                "Mask defining on which FECs (i.e., SCAs) to execute all following operations");
      }
    inline auto getFecMask() { return vm_["mask"].as<cl_uint_t>()(); }
};


/** Class to conveniently handle command line options and avoid boiler plate code
 *    in several command-line utilities
 *
 *  @note FIXME Add support for config file
 *  @note FIXME Add support for positional options
 */
class CommandLineOptions : public CommonOptions
{
  private:
#if 0
    // Would be cool to store info/definition of default options in tuple and just add them
    //   to the options_descriptions in a generic way...

    /** applyTuple implementation
     *    (from https://cpppatterns.com/patterns/apply-tuple-to-function.html)
     */
    template<typename F, typename T, size_t ...S >
    decltype(auto) applyTupleImpl(F&& fn, T&& t, std::index_sequence<S...>)
      {
        return std::forward<F>(fn)(std::get<S>(std::forward<T>(t))...);
      }

    /** Apply content of a tuple t as function parameters to function fn
     */
    template<typename F, typename T>
    decltype(auto) applyTuple(F&& fn, T&& t)
      {
        std::size_t constexpr tSize
          = std::tuple_size<typename std::remove_reference<T>::type>::value;
        return applyTupleImpl(std::forward<F>(fn),
                              std::forward<T>(t),
                              std::make_index_sequence<tSize>());
      }

    /** Pop first implementation */
    template <typename T, std::size_t ... S>
    auto popFirstTupleImpl(const T& tuple, std::index_sequence<S...>)
      {
        return std::make_tuple(std::get<1 + S>(tuple)...);
      }

    /** Pop first element from tuple
     */
    template <typename T>
    auto popFirst(const T& tuple)
      {
        return popFirstTupleImpl(tuple, std::make_index_sequence<std::tuple_size<T>::value - 1>());
      }
#endif

    /** Add option description
     *  @param od Options description container to add to
     *  @return Reference to this object (for chaining)
     */
    template<typename ...R>
    auto& add(bpo::options_description& od, R && ...r)
      {
        od.add_options()(std::forward<R>(r)...);
        return *this;
      }

    /** Get all options_description of all options that can be specified on the command line
     *    (generic, cmdline, hidden)
     */
    inline auto getOptionDescCmdLine() const
      {
        bpo::options_description opt_cmdline_all(opt_generic_);
        return opt_cmdline_all.add(opt_cmdline_).add(opt_hidden_);
      }

    /** Get all options_description of all options that can be specified in a config file
     *    (generic, cmdline, hidden)
     */
    inline auto getOptionDescCfgFile() const
      {
        bpo::options_description opt_file_all(opt_generic_);
        return opt_file_all.add(opt_file_).add(opt_hidden_);
      }

  protected:
    // Option descriptions
    bpo::options_description opt_generic_;                /** Generic options (given on command line or in config file)*/
    bpo::options_description opt_cmdline_;                /** Options specified on only command line */
    bpo::options_description opt_file_;                   /** Options given only in configuration file */
    bpo::options_description opt_hidden_;                 /** Options on command line or file that should be hidden */
    bpo::positional_options_description opt_positional_;  /** Positional options */

  public:
    CommandLineOptions(const std::string& help_head_text = "", bool add_common_options = true) :
        opt_generic_(help_head_text)
      {
        if (add_common_options) {
          /** Add default options common to all programs) */
          addHelp(opt_cmdline_);
          addVersion(opt_cmdline_);
          addVerbose(opt_cmdline_);
          addId(opt_generic_);
          addBar(opt_generic_);
          addFecMask(opt_generic_);
        }
      }

    /** Add generic option (command line and config file)
     *  @param r Option description as the one passed to boost::program_options::add_options()
     *  @return Reference to this CommandLineOptions for chaining
     */
    template<typename ...R>
    inline auto& add(R && ...r) { return add(opt_generic_, std::forward<R>(r)...); }

    /** Add hidden option (command line and config file)
     *  @param r Option description as the one passed to boost::program_options::add_options()
     *  @return Reference to this CommandLineOptions for chaining
     */
    template<typename ...R>
    inline auto& addHidden(R && ...r) { return add(opt_hidden_, std::forward<R>(r)...); }

    /** Add command line option
     *  @param r Option description as the one passed to boost::program_options::add_options()
     *  @return Reference to this CommandLineOptions for chaining
     */
    template<typename ...R>
    inline auto& addCmdLine(R && ...r) { return add(opt_cmdline_, std::forward<R>(r)...); }

    /** Add configuration file option
     *  @param r Option description as the one passed to boost::program_options::add_options()
     *  @return Reference to this CommandLineOptions for chaining
     */
    template<typename ...R>
    inline auto& addFile(R && ...r) { return add(opt_file_, std::forward<R>(r)...); }

    /** Process options from command line
     *  @param argc Argument count
     *  @param argv Argument values
     *  @param os_msg Output stream for regular messages
     *  @param os_err Output stream for error messages
     *  @return Filed boost variables map
     */
    inline auto& Process(int argc, char** argv, std::ostream& os_msg = std::cout, std::ostream& os_err = std::cerr)
      {
        try {
          bpo::store(bpo::command_line_parser(argc, argv)
                       .options(getOptionDescCmdLine())
                       .positional(opt_positional_).run(), vm_);

          if (getHelpFlag() || argc == 1) {
            os_msg << getOptionDescCmdLine();
            exit(0);
          }

          if (getVersionFlag()) {
            os_msg << GitInfo();
            exit(0);
          }

          bpo::notify(vm_);
        }
        catch(bpo::error& e) {
          os_err << "ERROR: " << e.what() << "\n\n"
                 << getOptionDescCmdLine() << std::endl;
          exit(1);

        }
        catch(std::exception& e)
        {
          os_err << e.what() << ", application will now exit" << std::endl;
          exit(2);
        }

        return vm_;
      }
};

} // namespace common
