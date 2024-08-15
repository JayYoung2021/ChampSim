#include <bitset>
#include <map>
#include <vector>

#include "cache.h"

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
    for (int i = 1, prefetches_issued = 0; i <= MAX_DISTANCE && prefetches_issued < ::private_data[this].degree; i++) {
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
    
    self.last_num_retired = num_retired;
    self.last_cycle = current_cycle_;
    
    const uint64_t cycle_diff = current_cycle_ - self.last_cycle;
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

void CACHE::prefetcher_final_stats() {}
