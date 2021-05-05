#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdint>
#include <chrono>

#include "hdlc.hpp"

namespace common {

/** Interface class for GBT SCA communication
 *  @see HdlcSwTrorc
 *  @see HdlcCernMe
 */
class HdlcCore
{
  private:
    static const std::string hdlcPayloadErrors_[]; /**< HDLC payload reply frame error codes, SCA manual v8.0 p14 */

  protected:
    const uint8_t transceive_max_retries_{50};
    const std::chrono::milliseconds transceive_timeout_{100};

    bool    request_trid_auto_; /**< Option for automatic transaction ID handling */
    uint8_t request_trid_;      /**< Next valid transaction ID */

    uint32_t stat_total_requests_; /**< Number of total HDLC request frames sent */

    bool print_hdlc_dbg_;       /**< Print HDLC debug messages */

    /** Adjust request trid\n
     *  Valid trids are in range 0x1 .. 0xfe
     *  @see GBT SCA manual v8.0 p13f
     */
    void adjustRequestTrid(HdlcEcFramePayload& r)
      {
        r.trid = (request_trid_auto_ ? request_trid_ : r.trid);
        request_trid_ = (request_trid_ % 0xfe) + 1;
      }

    /** GBT SCA communication via HDLC frames
     *  @param request_trid_auto Enable automatic handling of transaction ID in HDLC frame payload
     *                           (User-specified transaction ID will be overwritten)
     */
    HdlcCore(bool request_trid_auto = false, bool print_hdlc_dbg = true) :
        request_trid_auto_(request_trid_auto),
        request_trid_(1),
        stat_total_requests_(0),
        print_hdlc_dbg_(print_hdlc_dbg)
      {}

    /** SCA request frame dispatch
     *  @note Internal use only
     *  @param r Payload of the request frame
     *  @see sendFrame
     */
    virtual void sendFrame_(HdlcEcFramePayload& r) const = 0;

    /** Send SCA request frame\n
     *  Intercept user-specified transaction ID and replace with automatically
     *    generated ID, if class 'request_trid_auto_' option enabled.
     *    Then call self->sendFrame_ for actual transmission of the frame
     *  @param r Payload of the request frame
     */
    void sendFrame(HdlcEcFramePayload& r);

  public:
    /** Error and return codes
     */
    enum return_codes_t : uint32_t {
      success = 0x0,
      error = 0xffffffff
    };

    /** Print core status
     */
    friend std::ostream& operator<< (std::ostream& os, const HdlcCore& r) {
      return r.printStatus(os);
    }

    /** Print status information
     */
    virtual std::ostream& printStatus(std::ostream& os) const
      {
        os << "-> " << __func__ << " not implemented\n";
        return os;
      }

    /** Translate HDLC frame error code to string
     *  @param error_code Error code of the HDLC payload reply frame
     *  @return String indicated all active error conditions
     */
    std::string getPayloadErrorMessage(uint8_t error_code) const;

    /** Transmit SCA command (do not expect and wait for reply).
     *   i.e. blocking wait until send is possible and send request
     *
     *  Primarily intended for cases where command can be received by SCA, but
     *    reply will not make it back, e.g. if GBTx pattern generators are enabled
     *
     *  @param request Payload of the request frame
     *  @see sendFrame
     */
    virtual void transmitCommand(HdlcEcFramePayload& request) = 0;

    /** Execute SCA command cycle (transmit request and get reply).
     *   i.e. send request and (blocking) wait for reply frame with
     *   corresponding transaction ID
     *  @note This function does neither look at analyze nor interpret the error field of the reply frame,
     *          it only monitors the correctness of the received transaction ID
     *  @param request Payload of the request frame
     *  @param reply   Payload of the reply frame
     *  @return 0 if reply with correct transaction ID was received, non-zero otherwise
     *  @see getFrame
     *  @see sendFrame
     */
    virtual uint32_t executeCommand(HdlcEcFramePayload& request, HdlcEcFramePayload& reply) = 0;

    /** Send supervisory level command "CONNECT" to SCA (see SCA manual p.11)
     */
    virtual void sendSvlConnect() const = 0;

    /** Send supervisory level command "TEST" to SCA (see SCA manual p.11)
     */
    virtual void sendSvlTest() const = 0;

    /** Send supervisory level command "RESET" to SCA (see SCA manual p.11)
     */
    virtual void sendSvlReset() const = 0;

    /** Reset SCA core itself
     */
    virtual void rst() const = 0;

    /** Clear counters & statistics in the SCA core, if available
     */
    virtual void clr() const {};

    /** Set up special functions of the SCA core, if available
     */
    virtual void init() const {};
};

} // namepsace common
