#include <algorithm>
#include <iostream>
#include <map>

#include "cache.h"
#include "ooo_cpu.h"
#include <unordered_map>

constexpr bool ADA_DEBUG_PRINT = false;

namespace
{
constexpr uint64_t STATISTICS_INSTR_LIMIT_PER_DEGREE = 5000;
constexpr uint64_t BEST_DEGREE_INSTR_LIMIT = 25000;

constexpr int MIN_DEGREE = 0;
constexpr int MAX_DEGREE = 16;

constexpr int INITIAL_DEGREE = 0;

enum struct State : char {
  STATISTICS,
  BEST_DEGREE,
};

class PrivateData
{
public:
  uint64_t last_num_retired;

  int degree;
  int cache_operate_cnt;

  int num_misses;
  std::map<int, double> mpkis;

  State state;

  PrivateData() : last_num_retired(0), degree(INITIAL_DEGREE), cache_operate_cnt(0), num_misses(0), mpkis(), state(State::STATISTICS) {}
};
std::unordered_map<CACHE*, PrivateData> private_data;
} // namespace

void CACHE::prefetcher_initialize() {}

uint32_t CACHE::prefetcher_cache_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, uint8_t type, uint32_t metadata_in)
{
  ::private_data[this].cache_operate_cnt += 1;

  if (!cache_hit) {
    ::private_data[this].num_misses++;
  }

  const int degree = ::private_data[this].degree;
  for (int i = 0; i < degree; i++) {
    const uint64_t pf_addr = addr + (i + 1) * BLOCK_SIZE;
    if ((pf_addr >> LOG2_PAGE_SIZE) != (addr >> LOG2_PAGE_SIZE)) {
      break;
    }
    prefetch_line(pf_addr, true, metadata_in);
  }

  if constexpr (ADA_DEBUG_PRINT) {
    const uint64_t num_retired = this->p_cpu->num_retired;
    std::cout << num_retired << "," << degree << std::endl;
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
  const uint64_t num_retired = this->p_cpu->num_retired;

  PrivateData& self = ::private_data[this];

  const uint64_t num_retired_diff = num_retired - self.last_num_retired;

  if (self.state == State::STATISTICS) {
    if (num_retired_diff < STATISTICS_INSTR_LIMIT_PER_DEGREE) {
      return;
    }

    self.last_num_retired = num_retired;

    const double mpki = static_cast<double>(self.num_misses) / num_retired_diff * 1000;
    self.mpkis[self.degree] = mpki;
    self.degree++;

    if (self.degree <= MAX_DEGREE) {
      return;
    }

    self.degree = std::min_element(self.mpkis.begin(), self.mpkis.end(), [](const auto& a, const auto& b) { return a.second < b.second; })->first;
    self.mpkis.clear();

    self.state = State::BEST_DEGREE;
  } else if (self.state == State::BEST_DEGREE) {
    if (num_retired_diff < BEST_DEGREE_INSTR_LIMIT) {
      return;
    }
    self.last_num_retired = num_retired;

    self.degree = MIN_DEGREE;
    self.state = State::STATISTICS;
  }
}

void CACHE::prefetcher_final_stats() {}
