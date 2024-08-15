#include <iostream>
#include <map>

#include "cache.h"
#include "ooo_cpu.h"
#include <unordered_map>

constexpr bool ADA_DEBUG_PRINT = false;

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

void CACHE::prefetcher_initialize() {}

uint32_t CACHE::prefetcher_cache_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, uint8_t type, uint32_t metadata_in)
{
  ::private_data[this].cache_operate_cnt += 1;

  const int degree = degrees[::private_data[this].degree_index];
  for (int i = 0; i < degree; i++) {
    const uint64_t pf_addr = addr + (i + 1) * BLOCK_SIZE;
    if ((pf_addr >> LOG2_PAGE_SIZE) != (addr >> LOG2_PAGE_SIZE)) {
      break;
    }
    prefetch_line(pf_addr, true, metadata_in);
  }

  if constexpr (ADA_DEBUG_PRINT) {
    const uint64_t current_cycle_ = this->p_cpu->current_cycle;
    const uint64_t num_retired = this->p_cpu->num_retired;
    const double real_time_ipc = static_cast<double>(num_retired - ::private_data[this].last_num_retired) / (current_cycle_ - ::private_data[this].last_cycle);
    std::cout << num_retired << "," << degree << "," << real_time_ipc << std::endl;
  }

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
