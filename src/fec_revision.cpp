#include "fec_revision.hpp"

//------------------------------------------------------------------------------
const FecRevision::rev_map_t
FecRevision::rev_map_ = {
  // Invalid tuple
  {gbt::ScaId::SCA_ID_INVALID, std::make_tuple(0x0, REV_UNKNOWN, SAMPA_REV_UNKNOWN)},
  //
  {0x005f, std::make_tuple(0x801, REV_1A, SAMPA_REV_2)},
  {0x0060, std::make_tuple(0x803, REV_1A, SAMPA_REV_2)},
  {0x0061, std::make_tuple(0x702, REV_1 , SAMPA_REV_2)}, // -> check SAMPA rev
  {0x0062, std::make_tuple(0x700, REV_1 , SAMPA_REV_2)}, // -> check SAMPA rev
  {0x0063, std::make_tuple(0x800, REV_1A, SAMPA_REV_2)},
  {0x0064, std::make_tuple(0x809, REV_1A, SAMPA_REV_4)},
  {0x0066, std::make_tuple(0x802, REV_1A, SAMPA_REV_2)},
  {0x0067, std::make_tuple(0x701, REV_1 , SAMPA_REV_2)},
  {0x0068, std::make_tuple(0x703, REV_1 , SAMPA_REV_2)},
  {0x2839, std::make_tuple(0x808, REV_1A, SAMPA_REV_4)},
  {0x283b, std::make_tuple(0x80b, REV_1A, SAMPA_REV_4)},
  {0x2852, std::make_tuple(0x804, REV_1A, SAMPA_REV_3)},
  {0x2853, std::make_tuple(0x807, REV_1A, SAMPA_REV_3)},
  {0x286b, std::make_tuple(0x806, REV_1A, SAMPA_REV_3)},
  {0x2882, std::make_tuple(0x805, REV_1A, SAMPA_REV_3)},
  {0x28bf, std::make_tuple(0x80a, REV_1A, SAMPA_REV_2)}, // -> check SAMPA rev
  //
  {0x28ea, std::make_tuple(0x854, REV_1A, SAMPA_REV_4)}, // New FECs from ORNL, Oct2018
  {0x558c, std::make_tuple(0x850, REV_1A, SAMPA_REV_4)},
  {0x5611, std::make_tuple(0x851, REV_1A, SAMPA_REV_4)},
  {0x5629, std::make_tuple(0x852, REV_1A, SAMPA_REV_4)},
  //
  {0x283a, std::make_tuple(0x832, REV_1A, SAMPA_REV_4)}, // Next batch from ORNL, Oct2018
  {0x28c0, std::make_tuple(0x831, REV_1A, SAMPA_REV_4)},
  {0x28ac, std::make_tuple(0x833, REV_1A, SAMPA_REV_4)},
  {0x28d6, std::make_tuple(0x835, REV_1A, SAMPA_REV_4)},
  {0x28d2, std::make_tuple(0x836, REV_1A, SAMPA_REV_4)},
  // Dummy tuple
  {gbt::ScaId::SCA_ID_DUMMY_V1, std::make_tuple(0xfff, REV_0, SAMPA_REV_2)}    // 0xffffff
};


// These names need to corresponds to the definition of fec_rev_t
const std::string
FecRevision::fecNames_[] = { "revUnknown", "rev0", "rev1", "rev1a", "rev2" };


std::ostream&
FecRevision::printMap(std::ostream& os)
{
  os << "   SCA ID     FEC ID / FEC revision       SAMPA\n"
     << "------------------------------------------------\n";

  for (auto const& x: rev_map_) {
    os << "  0x" << std::hex << std::setfill('0') << std::setw(6) << getScaDbId(x)
       << " -> 0x" << std::setw(3) << std::right << getFecDbId(x)
       << "  / " << std::setfill(' ') << std::setw(12) << std::left << getFecName(getFecRev(x))
       << std::right << " (" << std::dec << static_cast<uint32_t>(getFecRev(x)) << ")"
       << " -> " << getSampaName(getSampaRev(x))
       << std::right << " (" << std::dec << static_cast<uint32_t>(getSampaRev(x)) << ")"
       << "\n";
  }

  return os;
}


const FecRevision::rev_tuple_t
FecRevision::get(const uint32_t sca_id)
{
  rev_map_t::const_iterator it = rev_map_.find(sca_id);
  if (it != rev_map_.end())
    return it->second;
  //
  // Return the first tuple in the map with invalid info as default
  // TODO: Once final revision is known, we only look up the non-final tuples
  //         and otherwise return the newest value tuple as default
  return get(gbt::ScaId::SCA_ID_INVALID);
}
