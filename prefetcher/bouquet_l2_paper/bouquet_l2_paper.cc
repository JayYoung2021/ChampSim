#include <map>

#include "cache.h"
#include "ooo_cpu.h"
#include <unordered_map>

#define NUM_IP_TABLE_L2_ENTRIES 1024
#define NUM_IP_INDEX_BITS 10
#define NUM_IP_TAG_BITS 6
#define S_TYPE 1    // global stream (GS)
#define CS_TYPE 2   // constant stride (CS)
#define CPLX_TYPE 3 // complex stride (CPLX)
#define NL_TYPE 4   // next line (NL)

// #define SIG_DEBUG_PRINT_L2
#ifdef SIG_DEBUG_PRINT_L2
#define SIG_DP(x) x
#else
#define SIG_DP(x)
#endif

namespace
{
constexpr uint64_t STATISTICS_INSTR_LIMIT_PER_DEGREE = 4096;
constexpr uint64_t BEST_DEGREE_INSTR_LIMIT = 4194304;

constexpr int degrees[] = {0, 1, 2, 3, 4, 6, 8, 12, 16};
constexpr int MAX_DEGREE_INDEX = sizeof(degrees) / sizeof(degrees[0]) - 1;

constexpr int INITIAL_DEGREE_INDEX = 0; // -> 2

enum struct State : char {
  STATISTICS,
  BEST_DEGREE,
};

class PrivateData
{
public:
  uint64_t last_num_retired;
  uint64_t last_cycle;

  int degree_index;
  int cache_operate_cnt;

  std::map<int, double> ipcs; // ipcs[i] is the IPC of degree indexed by i

  State state;

  PrivateData() : last_num_retired(0), last_cycle(0), degree_index(INITIAL_DEGREE_INDEX), cache_operate_cnt(0), ipcs(), state(State::STATISTICS) {}
};
std::unordered_map<CACHE*, PrivateData> private_data;
} // namespace

class IP_TRACKER
{
public:
  uint64_t ip_tag;
  uint16_t ip_valid;
  uint32_t pref_type; // prefetch class type
  int stride;         // last stride sent by metadata

  IP_TRACKER()
  {
    ip_tag = 0;
    ip_valid = 0;
    pref_type = 0;
    stride = 0;
  };
};

uint32_t spec_nl_l2[NUM_CPUS] = {0};
IP_TRACKER trackers[NUM_CPUS][NUM_IP_TABLE_L2_ENTRIES];

int decode_stride(uint32_t metadata)
{
  int stride = 0;
  if (metadata & 0b1000000)
    stride = -1 * (metadata & 0b111111);
  else
    stride = metadata & 0b111111;

  return stride;
}

void CACHE::prefetcher_initialize() {}

uint32_t CACHE::prefetcher_cache_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, uint8_t type, uint32_t metadata_in)
{
  uint64_t cl_addr = addr >> LOG2_BLOCK_SIZE;
  int prefetch_degree = degrees[::private_data[this].degree_index];
  int64_t stride = decode_stride(metadata_in);
  uint32_t pref_type = metadata_in & 0xF00;
  uint16_t ip_tag = (ip >> NUM_IP_INDEX_BITS) & ((1 << NUM_IP_TAG_BITS) - 1);

  // calculate the index bit
  int index = ip & ((1 << NUM_IP_INDEX_BITS) - 1);
  if (trackers[cpu][index].ip_tag != ip_tag) { // new/conflict IP
    if (trackers[cpu][index].ip_valid == 0) {  // if valid bit is zero, update with latest IP info
      trackers[cpu][index].ip_tag = ip_tag;
      trackers[cpu][index].pref_type = pref_type;
      trackers[cpu][index].stride = stride;
    } else {
      trackers[cpu][index].ip_valid = 0; // otherwise, reset valid bit and leave the previous IP as it is
    }

    // issue a next line prefetch upon encountering new IP
    uint64_t pf_address = ((addr >> LOG2_BLOCK_SIZE) + 1) << LOG2_BLOCK_SIZE;
    prefetch_line(pf_address, true, 0);
    SIG_DP(cout << "1, ");
    return metadata_in;
  } else { // if same IP encountered, set valid bit
    trackers[cpu][index].ip_valid = 1;
  }

  // std::cout << int(type) << " " << int(static_cast<uint8_t>(access_type::PREFETCH)) << std::endl;
  // update the IP table upon receiving metadata from prefetch
  if (type == static_cast<uint8_t>(access_type::PREFETCH)) {
    trackers[cpu][index].pref_type = pref_type;
    trackers[cpu][index].stride = stride;
    spec_nl_l2[cpu] = metadata_in & 0x1000;
  }

  SIG_DP(cout << ip << ", " << cache_hit << ", " << cl_addr << ", "; cout << ", " << stride << "; ";);

  // we are prefetching only for GS, CS and NL classes
  // std::cout << cpu << " " << index << " " << trackers[cpu][index].stride << std::endl;
  if (trackers[cpu][index].stride != 0) {
    if (trackers[cpu][index].pref_type == 0x100 || trackers[cpu][index].pref_type == 0x200) { // GS or CS class
      if (trackers[cpu][index].pref_type == 0x100)
        if (NUM_CPUS == 1)
          prefetch_degree = 4;
      for (int i = 0; i < prefetch_degree; i++) {
        uint64_t pf_address = (cl_addr + (trackers[cpu][index].stride * (i + 1))) << LOG2_BLOCK_SIZE;

        // Check if prefetch address is in same 4 KB page
        if ((pf_address >> LOG2_PAGE_SIZE) != (addr >> LOG2_PAGE_SIZE))
          break;

        prefetch_line(pf_address, true, 0);
        SIG_DP(cout << trackers[cpu][index].stride << ", ");
      }
    } else if (trackers[cpu][index].pref_type == 0x400 && spec_nl_l2[cpu] > 0) {
      uint64_t pf_address = ((addr >> LOG2_BLOCK_SIZE) + 1) << LOG2_BLOCK_SIZE;
      prefetch_line(pf_address, true, 0);

      SIG_DP(cout << "1;");
    }
  }

  SIG_DP(cout << endl);
  return metadata_in;
}

uint32_t CACHE::prefetcher_cache_fill(uint64_t addr, uint32_t set, uint32_t way, uint8_t prefetch, uint64_t evicted_addr, uint32_t metadata_in)
{
  return metadata_in;
}

void CACHE::prefetcher_cycle_operate()
{
  if (::private_data[this].cache_operate_cnt < 10) {
    return;
  }
  ::private_data[this].cache_operate_cnt = 0;

  // Avoid name collisions
  const uint64_t current_cycle_ = this->p_cpu->current_cycle;
  const uint64_t num_retired = this->p_cpu->num_retired;

  PrivateData& self = ::private_data[this];

  const uint64_t num_retired_diff = num_retired - self.last_num_retired;

  if (self.state == State::STATISTICS) {
    if (num_retired_diff < STATISTICS_INSTR_LIMIT_PER_DEGREE) {
      return;
    }

    const uint64_t cycle_diff = current_cycle_ - self.last_cycle;
    self.last_num_retired = num_retired;
    self.last_cycle = current_cycle_;

    const double ipc = static_cast<double>(num_retired_diff) / cycle_diff;
    self.ipcs[self.degree_index] = ipc;
    self.degree_index++;

    if (self.degree_index <= MAX_DEGREE_INDEX) {
      return;
    }

    // Find the best degree
    const auto best_degree_index_iter = std::max_element(self.ipcs.begin(), self.ipcs.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    self.degree_index = best_degree_index_iter->first;
    self.ipcs.clear();

    self.state = State::BEST_DEGREE;
  } else if (self.state == State::BEST_DEGREE) {
    if (num_retired_diff < BEST_DEGREE_INSTR_LIMIT) {
      return;
    }
    self.last_num_retired = num_retired;
    self.last_cycle = current_cycle_;

    self.degree_index = 0;

    self.state = State::STATISTICS;
  }
}

void CACHE::prefetcher_final_stats() {}