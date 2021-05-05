#pragma once

#include <map>
#include <string>
#include <iostream>
#include <iomanip>
#include <cstdint>

#include "gbt_sca.hpp"

/** Utility class to help distinguish different FEC revisions
 *
 *    Since no other methods of distinguishing FECs are available,
 *    it is based on a lookup of SCA revision IDs read back
 */
class FecRevision
{
  public:
    /** The various FEC revisions
     */
    enum fec_rev_t : uint8_t {REV_UNKNOWN = 0, REV_0, REV_1, REV_1A, REV_2};   // Make sure to adjust fecNames_ if changing anything here

    /** The various SAMPA revision */
    enum sampa_rev_t : uint8_t {SAMPA_REV_UNKNOWN = 0, SAMPA_REV_1, SAMPA_REV_2, SAMPA_REV_3, SAMPA_REV_4};

    /** Tuple holding FEC & SAMPA revision information */
    using rev_tuple_t = std::tuple<uint32_t, fec_rev_t, sampa_rev_t>;

    /** Extract FEC ID as stored in database from rev_tuple_t
     *  @param r Tuple with revision information
     *  @return Stored FEC ID
     */
    static inline uint32_t getFecDbId(const rev_tuple_t& r) { return std::get<0>(r); }

    /** Extract FEC revision as stored in database from rev_tuple_t
     *  @param r Tuple with revision information
     *  @return FEC revision identifier
     */
    static inline fec_rev_t getFecRev(const rev_tuple_t& r) { return std::get<1>(r); }

    /** Extract SAMPA revision as stored in database from rev_tuple_t
     *  @param r Tuple with revision information
     *  @return SAMPA revision identifier
     */
    static inline sampa_rev_t getSampaRev(const rev_tuple_t& r) { return std::get<2>(r); }

    /** Perform lookup of FEC & SAMPA revision based on SCA ID
     *  @return Tuple with FEC revision information if SCA ID can be matched. Returns tuple with invalid information if
     *          SCA ID cannot be matched
     */
    static const rev_tuple_t get(const uint32_t sca_id);

    /** Print known FEC revisions and revision identifiers
     */
    static std::ostream& printMap(std::ostream& os = std::cout);

    /** Resolve FEC revision identifier to revision name string
     *
     * @param r FEC revision identifier
     * @return Resolved FEC revision name string
     */
    static const std::string& getFecName(const fec_rev_t& r) { return fecNames_[r]; }

    /** Resolve SAMPA revision to revision name string
     *  @param r SAMPA revision identifier
     *  @return Resolved SAMPA revision name string
     */
    static const std::string getSampaName(const sampa_rev_t& r)
      { return std::string("v" + std::to_string(static_cast<uint32_t>(r))); }

  private:
    /** rev_map_t content */
    using rev_map_t = std::map<uint32_t, rev_tuple_t>;
    using rev_map_pair_t = std::pair<uint32_t, rev_tuple_t>;

    /** Access to rev_map_t content */
    static inline uint32_t getScaDbId(const rev_map_pair_t r) { return r.first; }
    static inline uint32_t getFecDbId(const rev_map_pair_t r) { return getFecDbId(r.second); }
    static inline fec_rev_t getFecRev(const rev_map_pair_t r) { return getFecRev(r.second); }
    static inline sampa_rev_t getSampaRev(const rev_map_pair_t r) { return getSampaRev(r.second); }

    /** Map of known/relevant SCA IDs and tuples with info on the FEC (id, revision, SAMPA rev)
     */
    static const rev_map_t rev_map_;

    static const std::string fecNames_[];
};
