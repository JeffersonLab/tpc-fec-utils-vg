#include <iostream>
#include <iomanip>
#include <mutex>
#include <chrono>
#include <cstdint>

//------------------------------------------------------------------------------
/** Trivial logger class
 *   for logging to stream from multiple threads
 */
class Log
{
  public:
    /** Log levels
     */
    enum level_t : int32_t {
      ldbgl = 0,
      ldbg,
      lsys,
      linfo,
      lwarn,
      lerror,
      lfatal
    };

    /** Logging specialization for different log levels
     *
     *  Resulting call order via log base templates is, e.g.
     *    lwarn -> log(lwarn, ...) -> log(...) -> ... -> log()
     */
    template<typename ...R>
    static void dbgl(R && ...r) { log(ldbgl, std::forward<R>(r)...); }
    template<typename ...R>
    static void dbg(R && ...r) { log(ldbg, std::forward<R>(r)...); }
    template<typename ...R>
    static void sys(R && ...r) { log(lsys, std::forward<R>(r)...); }
    template<typename ...R>
    static void info(R && ...r) { log(linfo, std::forward<R>(r)...); }
    template<typename ...R>
    static void warn(R && ...r) { log(lwarn, std::forward<R>(r)...); }
    template<typename ...R>
    static void error(R && ...r) { log(lerror, std::forward<R>(r)...); }
    template<typename ...R>
    static void fatal(R && ...r) { log(lfatal, std::forward<R>(r)...); }

    /** Change log level
     *  @param l New log level
     */
    static void setLevel(level_t l) { targetLogLevel_ = l; }

    /** Include timestamp in log output
     */
    static void setIncludeTimestamp(bool enable = true) { includeTs_ = enable; }

    static std::string levelName(level_t l) { return levelNames_[l]; }

  private:
    static std::mutex mtx_;
    static level_t targetLogLevel_;
    static bool includeTs_;

    /** Logging base template
     */
    static void log() { std::cout << "\n"; }

    template<typename ...R>
    static void log(level_t lvl, R && ...r)
      {
        if (lvl >= targetLogLevel_) {
          if (includeTs_) {
            auto ts = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            char tbuf[64];
            std::strftime(tbuf, 64, "%Y-%m-%d %H:%M:%S", std::localtime(&ts));
            std::lock_guard<std::mutex> g(mtx_);
            std::cout << std::setw(20) << tbuf << "  [" << lvl << "] :";
          }
          else {
            std::lock_guard<std::mutex> g(mtx_);
            std::cout << "[" << lvl << "] :";
          }
          Log::log(std::forward<R>(r)...);
        }
      }

    template<typename F, typename ...R>
    static void log(F && f, R && ...r)
      {
        std::cout << " " << std::forward<F>(f);
        Log::log(std::forward<R>(r)...);
      }

  protected:
    static const std::string levelColors_[];
    static const std::string levelNames_[];

    friend std::ostream& operator<< (std::ostream& s, const Log::level_t l)
    {
      s << Log::levelColors_[l] << Log::levelNames_[l] << "\033[;39m";
      return s;
    }
};
