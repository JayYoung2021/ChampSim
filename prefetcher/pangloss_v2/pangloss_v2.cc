#include <algorithm>
#include <map>

#include "cache.h"
#include "ooo_cpu.h"
#include <unordered_map>

namespace
{
constexpr uint64_t STATISTICS_INSTR_LIMIT_PER_DEGREE = 8192;
constexpr uint64_t BEST_DEGREE_INSTR_LIMIT = 262144;

constexpr int MIN_DEGREE = 0;
constexpr int MAX_DEGREE = 16;

constexpr int INITIAL_DEGREE = 2;

enum struct State : char {
  STATISTICS,
  BEST_DEGREE,
  LAST_BEST_DEGREE,
};

class PrivateData
{
public:
  uint64_t last_num_retired;
  uint64_t last_cycle;

  int degree;
  int last_best_degree;
  int cache_operate_cnt;

  std::map<int, double> ipcs; // ipcs[i] is the IPC of degree indexed by i

  State state;

  PrivateData() : last_num_retired(0), last_cycle(0), degree(INITIAL_DEGREE), last_best_degree(-1), cache_operate_cnt(0), ipcs(), state(State::STATISTICS) {}
};
std::unordered_map<CACHE*, PrivateData> private_data;
} // namespace

// Delta Cache
#define DC_range_l2c (128) // 128 (-1) possible deltas
#define DC_ways_l2c 16     // Delta cache assoc.
#define DC_LFUmax_l2c 256  // Maximum LFU counter value

int DC_deltanext_l2c[DC_range_l2c][DC_ways_l2c]; // Next deltas
int DC_LFUbits_l2c[DC_range_l2c][DC_ways_l2c];   // Frequency counters

// Page Cache
#define PC_sets_l2c 256    // 256 sets
#define PC_ways_l2c 12     // 12 ways
#define PC_tag_bits_l2c 10 // 10-bit page tags

int PC_ldelta_l2c[PC_sets_l2c][PC_ways_l2c];  // Last deltas
int PC_loffset_l2c[PC_sets_l2c][PC_ways_l2c]; // Last offsets
int PC_ptag_l2c[PC_sets_l2c][PC_ways_l2c];    // Page tag
int PC_NRUbit_l2c[PC_sets_l2c][PC_ways_l2c];  // Not-Recently Used (NRU) bits

// Supplementary function to caclulate the page tag
int get_page_tag_l2c(uint64_t page) { return (page / PC_sets_l2c) & ((1 << PC_tag_bits_l2c) - 1); }

// Prefetcher initialisation
void CACHE::prefetcher_initialize()
{
  printf("Ultra pref. initializing...\n");
  fflush(stdout);

  // Initialise the Delta Cache
  for (int i = 0; i < DC_range_l2c; i++) {
    for (int j = 0; j < DC_ways_l2c; j++) {
      DC_deltanext_l2c[i][j] = 1 + DC_range_l2c / 2; // (Optional (currently not used): fallback to delta=1)
      DC_LFUbits_l2c[i][j] = 0;
    }
  }

  // Note: the Page Cache initialisation is less important, since we are looking for page tag hits of 10-bits
  printf("Ultra pref. initialized\n");
  fflush(stdout);
}

// This function updates the Delta Cache with a new delta transition (delta_from -> delta_next)
void update_DC_l2c(int delta_from, int delta_to)
{

  // Look for hits
  int dhit = 0;
  for (int i = 0; i < DC_ways_l2c; i++) {

    // If there is a hit, increment the respective counter
    if (DC_deltanext_l2c[delta_from][i] == delta_to) {
      DC_LFUbits_l2c[delta_from][i]++;

      // If there is an overflow,
      if (DC_LFUbits_l2c[delta_from][i] == DC_LFUmax_l2c) {
        for (int j = 0; j < DC_ways_l2c; j++) {

          // Decrement all counters in the set (frequency proportions shall remain about the same)
          DC_LFUbits_l2c[delta_from][j] /= 2;
        }
      }
      dhit = 1;
      break;
    }
  }

  // If the delta transition is not in Delta Cache,
  if (dhit == 0) {

    // Evict the least-frequent delta transition in the set
    int min_freq = DC_LFUbits_l2c[delta_from][0];
    int min_freq_way = 0;
    for (int i = 1; i < DC_ways_l2c; i++) {
      if (DC_LFUbits_l2c[delta_from][i] < min_freq) {
        min_freq = DC_LFUbits_l2c[delta_from][i];
        min_freq_way = i;
      }
    }

    // And replace with the current one
    DC_deltanext_l2c[delta_from][min_freq_way] = delta_to;
    DC_LFUbits_l2c[delta_from][min_freq_way] = 1;
  }
}

// This function returns the most probably immidiately next delta based on the current delta
int get_next_best_transition_l2c(int delta)
{

  // Caclulate the sum of the LFU counters for the current set
  int probs_sum = 0;
  for (int i = 0; i < DC_ways_l2c; i++) {
    probs_sum += DC_LFUbits_l2c[delta][i];
  }

  // Find the maximum LFU value
  int max_freq = DC_LFUbits_l2c[delta][0];
  int max_freq_way = 0;
  for (int i = 1; i < DC_ways_l2c; i++) {
    if (DC_LFUbits_l2c[delta][i] > max_freq) {
      max_freq = DC_LFUbits_l2c[delta][i];
      max_freq_way = i;
    }
  }

  // Discard, if it represents a probability lower than 1/3
  if ((float)DC_LFUbits_l2c[delta][max_freq_way] / probs_sum < 1 / 3.0)
    return -1;

  return DC_deltanext_l2c[delta][max_freq_way];
}

uint32_t CACHE::prefetcher_cache_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, uint8_t type, uint32_t metadata_in)
{
  ::private_data[this].cache_operate_cnt += 1;

  unsigned long long int cl_address = addr >> 6; // Cache line address
  unsigned long long int page = cl_address >> 6;
  int page_offset = cl_address & 63;

  // Page Cache lookup: Looking for previous entries of the same page in the Page Cache
  int way = -1;
  for (int i = 0; i < PC_ways_l2c; i++) {
    if (PC_ptag_l2c[page % PC_sets_l2c][i] == get_page_tag_l2c(page)) {
      way = i;
      break;
    }
  }

  int cur_delta = 1 + DC_range_l2c / 2; // (fallback to delta=1, when there is no page match)
  int matched = 0;

  // If there was a Page Cache hit,
  // (and the access does not come from a prefetch miss (small heuristic to avoid expensive prefetch chains))
  if ((way != -1) && !((type == static_cast<uint8_t>(access_type::PREFETCH)) && (cache_hit == 0))) {

    int ldelta_l2c = PC_ldelta_l2c[page % PC_sets_l2c][way]; // Last delta
    int loff_l2c = PC_loffset_l2c[page % PC_sets_l2c][way];  // Last offset

    // Calculate current delta,
    cur_delta = page_offset - loff_l2c + DC_range_l2c / 2;
    matched = 1;

    // And update the Delta cache with the new delta transition
    update_DC_l2c(ldelta_l2c, cur_delta);
  }

  int next_delta = cur_delta;
  uint64_t addr_n = addr; // Next address to prefetch
  int count = 0;

  int degree = std::min(::private_data[this].degree, static_cast<int>(MSHR_SIZE - get_mshr_occupancy() * 2 / 3));
  if ((type == static_cast<uint8_t>(access_type::PREFETCH)) && (cache_hit == 0))
    degree /= 2;

  for (int i_ = 0; i_ < degree && count < degree; i_++) {

    // Find the most probable next delta from the current delta (1 generation)
    int best_delta = get_next_best_transition_l2c(next_delta);

    // Abort if probability less than 1/3
    if (best_delta == -1)
      break;
    {
      // Aggregate all counter values in the set
      int sum = 0;
      for (int j = 0; j < DC_ways_l2c; j++) {
        sum += DC_LFUbits_l2c[next_delta][j];
      }

      // Looking for the top 2 child deltas (no more than 2 having a probability > 1/3)
      int used[DC_ways_l2c] = {0};
      for (int i = 0; i < 2 /*DC_ways_l2c*/; i++) {
        int max_way = -1;
        int max_value = -1;
        for (int j = 0; j < DC_ways_l2c; j++) {
          if ((DC_LFUbits_l2c[next_delta][j] > max_value) && (!used[j])) {
            max_way = j;
            max_value = DC_LFUbits_l2c[next_delta][j];
          }
        }
        if (max_way == -1)
          continue;

        // If the probability is greater than 1/3,
        if ((count < degree) && ((float)DC_LFUbits_l2c[next_delta][max_way] / sum > 1 / 3.0)) {
          used[max_way] = 1;
          uint64_t pf_addr = ((addr_n >> LOG2_BLOCK_SIZE) + (DC_deltanext_l2c[next_delta][max_way] - DC_range_l2c / 2)) << LOG2_BLOCK_SIZE;
          unsigned long long int pf_page = pf_addr >> 12;

          // And it falls in the same page, prefetch block
          if (page == pf_page) {
            prefetch_line(pf_addr, false, metadata_in);
            count++;
          }
        }
      }
    }

    // Update values for moving to the next delta generation based on the top next delta
    next_delta = best_delta;
    uint64_t pf_addr = ((addr_n >> LOG2_BLOCK_SIZE) + (best_delta - DC_range_l2c / 2)) << LOG2_BLOCK_SIZE;
    addr_n = pf_addr;
  }

  // If there was a Page cache miss, evict the Not-Recently used
  if (way == -1) {

    // Look for NRU bit equal to 0
    for (int i = 0; i < PC_ways_l2c; i++) {
      if (PC_NRUbit_l2c[page % PC_sets_l2c][i] == 0) {
        way = i;
        break;
      }
    }

    // If all are equal to 1, flip them
    if (way == -1) {
      way = 0;
      for (int i = 0; i < PC_ways_l2c; i++)
        PC_NRUbit_l2c[page % PC_sets_l2c][i] = 0;
    }
  }

  // Update the respective Page Cache entry
  if (matched)
    PC_ldelta_l2c[page % PC_sets_l2c][way] = cur_delta;
  else
    // If we did not have this entry (or bypassed the delta transition),
    // the delta value is invalid (0 represents delta=-64, which falls in a different page)
    PC_ldelta_l2c[page % PC_sets_l2c][way] = 0;

  PC_loffset_l2c[page % PC_sets_l2c][way] = page_offset;
  PC_ptag_l2c[page % PC_sets_l2c][way] = get_page_tag_l2c(page);
  PC_NRUbit_l2c[page % PC_sets_l2c][way] = 1;

  return metadata_in;
}

// Not used
uint32_t CACHE::prefetcher_cache_fill(uint64_t addr, uint32_t set, uint32_t way, uint8_t prefetch, uint64_t evicted_addr, uint32_t metadata_in)
{
  // unsigned long long int cl_address = addr >> 6;
  // unsigned long long int page = cl_address >> 6;
  // int page_offset = cl_address & 63;
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
    self.ipcs[self.degree] = ipc;

    if (self.last_best_degree == MIN_DEGREE) {
      assert(MIN_DEGREE == 0 && self.degree == 1);
      // nop
    } else if (self.last_best_degree == MAX_DEGREE) {
      assert(MAX_DEGREE > 0 && self.degree == MAX_DEGREE - 1);
      // nop
    } else {
      if (self.last_best_degree != -1) {
        if (self.degree == self.last_best_degree - 1) {
          self.degree = self.last_best_degree + 1;
          return;
        } else if (self.degree == self.last_best_degree + 1) {
          // nop
        } else {
          assert(false && "Invalid degree");
        }
      } else {
        if (self.degree == INITIAL_DEGREE) {
          self.degree = INITIAL_DEGREE - 1;
          return;
        } else if (self.degree == INITIAL_DEGREE - 1) {
          self.degree = INITIAL_DEGREE + 1;
          return;
        } else if (self.degree == INITIAL_DEGREE + 1) {
          // nop
        } else {
          assert(false && "Invalid degree");
        }
      }
    }

    const int best_degree = std::max_element(self.ipcs.begin(), self.ipcs.end(), [](const auto& a, const auto& b) { return a.second < b.second; })->first;

    // for(const auto& [degree_, ipc_] : self.ipcs) {
    //   std::cout << degree_ << " IPC: " << ipc_ << std::endl;
    // }
    // std::cout  << best_degree << std::endl;

    self.degree = best_degree;
    self.ipcs.clear();

    self.state = State::BEST_DEGREE;
  } else if (self.state == State::BEST_DEGREE) {
    if (num_retired_diff < BEST_DEGREE_INSTR_LIMIT - STATISTICS_INSTR_LIMIT_PER_DEGREE) {
      return;
    }
    self.last_num_retired = num_retired;
    self.last_cycle = current_cycle_;

    self.state = State::LAST_BEST_DEGREE;
  } else if (self.state == State::LAST_BEST_DEGREE) {
    if (num_retired_diff < STATISTICS_INSTR_LIMIT_PER_DEGREE) {
      return;
    }

    const uint64_t cycle_diff = current_cycle_ - self.last_cycle;
    const double ipc = static_cast<double>(num_retired_diff) / cycle_diff;
    self.ipcs[self.degree] = ipc;

    self.last_best_degree = self.degree;

    if (self.last_best_degree == MIN_DEGREE) {
      assert(MIN_DEGREE == 0);
      self.degree = 1;
    } else if (self.last_best_degree == MAX_DEGREE) {
      assert(MAX_DEGREE > 0);
      self.degree = MAX_DEGREE - 1;
    } else {
      self.degree = self.last_best_degree - 1;
    }

    self.last_num_retired = num_retired;
    self.last_cycle = current_cycle_;

    self.state = State::STATISTICS;
  }
}

void CACHE::prefetcher_final_stats()
{
  // Print the information stored in Delta Cache
  printf("\n");
  for (int i = 0; i < DC_range_l2c; i++) {
    printf("L2C %d>\t", i - DC_range_l2c / 2);
    for (int j = 0; j < DC_ways_l2c; j++) {
      printf("(%d,%d)\t", DC_deltanext_l2c[i][j] - DC_range_l2c / 2, DC_LFUbits_l2c[i][j]);
    }
    printf("\n");
  }
}
