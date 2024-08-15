#include <bitset>
#include <map>
#include <vector>

#include "cache.h"

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

namespace
{
constexpr std::size_t REGION_COUNT = 128;
constexpr int MAX_DISTANCE = 256;

struct region_type {
  uint64_t vpn;
  std::bitset<PAGE_SIZE / BLOCK_SIZE> access_map{};
  std::bitset<PAGE_SIZE / BLOCK_SIZE> prefetch_map{};
  uint64_t lru;

  static uint64_t region_lru;

  region_type() : region_type(0) {}
  explicit region_type(uint64_t allocate_vpn) : vpn(allocate_vpn), lru(region_lru++) {}
};
uint64_t region_type::region_lru = 0;

std::map<CACHE*, std::array<region_type, REGION_COUNT>> regions;

auto page_and_offset(uint64_t addr)
{
  auto page_number = addr >> LOG2_PAGE_SIZE;
  auto page_offset = (addr & champsim::msl::bitmask(LOG2_PAGE_SIZE)) >> LOG2_BLOCK_SIZE;
  return std::pair{page_number, page_offset};
}

bool check_cl_access(CACHE* cache, uint64_t v_addr)
{
  auto [vpn, page_offset] = page_and_offset(v_addr);
  auto region = std::find_if(std::begin(regions.at(cache)), std::end(regions.at(cache)), [vpn = vpn](auto x) { return x.vpn == vpn; });

  return (region != std::end(regions.at(cache))) && region->access_map.test(page_offset);
}

bool check_cl_prefetch(CACHE* cache, uint64_t v_addr)
{
  auto [vpn, page_offset] = page_and_offset(v_addr);
  auto region = std::find_if(std::begin(regions.at(cache)), std::end(regions.at(cache)), [vpn = vpn](auto x) { return x.vpn == vpn; });

  return (region != std::end(regions.at(cache))) && region->prefetch_map.test(page_offset);
}

} // anonymous namespace

void CACHE::prefetcher_initialize() { regions.insert_or_assign(this, decltype(regions)::mapped_type{}); }

uint32_t CACHE::prefetcher_cache_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, uint8_t type, uint32_t metadata_in)
{
  auto [current_vpn, page_offset] = ::page_and_offset(addr);
  auto demand_region = std::find_if(std::begin(::regions.at(this)), std::end(::regions.at(this)), [vpn = current_vpn](auto x) { return x.vpn == vpn; });

  if (demand_region == std::end(::regions.at(this))) {
    // not tracking this region yet, so replace the LRU region
    demand_region = std::min_element(std::begin(::regions.at(this)), std::end(::regions.at(this)), [](auto x, auto y) { return x.lru < y.lru; });
    *demand_region = region_type{current_vpn};
    return metadata_in;
  }

  // mark this demand access
  demand_region->access_map.set(page_offset);

  // attempt to prefetch in the positive, then negative direction
  for (auto direction : {1, -1}) {
    for (int i = 1, prefetches_issued = 0; i <= MAX_DISTANCE && prefetches_issued < degrees[::private_data[this].degree_index]; i++) {
      const auto pos_step_addr = addr + direction * (i * (signed)BLOCK_SIZE);
      const auto neg_step_addr = addr - direction * (i * (signed)BLOCK_SIZE);
      const auto neg_2step_addr = addr - direction * (2 * i * (signed)BLOCK_SIZE);

      if (::check_cl_access(this, neg_step_addr) && ::check_cl_access(this, neg_2step_addr) && !::check_cl_access(this, pos_step_addr)
          && !::check_cl_prefetch(this, pos_step_addr)) {
        // found something that we should prefetch
        if ((addr >> LOG2_BLOCK_SIZE) != (pos_step_addr >> LOG2_BLOCK_SIZE)) {
          bool prefetch_success = prefetch_line(pos_step_addr, (get_mshr_occupancy_ratio() < 0.5), metadata_in);
          if (prefetch_success) {
            auto [pf_vpn, pf_page_offset] = ::page_and_offset(pos_step_addr);
            auto pf_region = std::find_if(std::begin(::regions.at(this)), std::end(::regions.at(this)), [vpn = pf_vpn](auto x) { return x.vpn == vpn; });

            if (pf_region == std::end(::regions.at(this))) {
              // we're not currently tracking this region, so allocate a new region so we can mark it
              pf_region = std::min_element(std::begin(::regions.at(this)), std::end(::regions.at(this)), [](auto x, auto y) { return x.lru < y.lru; });
              *pf_region = region_type{pf_vpn};
            }

            pf_region->prefetch_map.set(pf_page_offset);
            prefetches_issued++;
          }
        }
      }
    }
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
