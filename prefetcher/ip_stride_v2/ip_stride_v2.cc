#include <algorithm>
#include <array>
#include <map>
#include <optional>

#include "cache.h"
#include "msl/lru_table.h"
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

namespace
{
struct tracker {
  struct tracker_entry {
    uint64_t ip = 0;           // the IP we're tracking
    uint64_t last_cl_addr = 0; // the last address accessed by this IP
    int64_t last_stride = 0;   // the stride between the last two addresses accessed by this IP

    auto index() const { return ip; }
    auto tag() const { return ip; }
  };

  struct lookahead_entry {
    uint64_t address = 0;
    int64_t stride = 0;
    int degree = 0; // degree remaining
  };

  constexpr static std::size_t TRACKER_SETS = 256;
  constexpr static std::size_t TRACKER_WAYS = 4;

  std::optional<lookahead_entry> active_lookahead;

  champsim::msl::lru_table<tracker_entry> table{TRACKER_SETS, TRACKER_WAYS};

public:
  void initiate_lookahead(uint64_t ip, uint64_t cl_addr, int degree)
  {
    int64_t stride = 0;

    auto found = table.check_hit({ip, cl_addr, stride});

    // if we found a matching entry
    if (found.has_value()) {
      // calculate the stride between the current address and the last address
      // no need to check for overflow since these values are downshifted
      stride = static_cast<int64_t>(cl_addr) - static_cast<int64_t>(found->last_cl_addr);

      // Initialize prefetch state unless we somehow saw the same address twice in
      // a row or if this is the first time we've seen this stride
      if (stride != 0 && stride == found->last_stride)
        active_lookahead = {cl_addr << LOG2_BLOCK_SIZE, stride, degree};
    }

    // update tracking set
    table.fill({ip, cl_addr, stride});
  }

  void advance_lookahead(CACHE* cache)
  {
    // If a lookahead is active
    if (!active_lookahead.has_value()) {
      return;
    }
    auto [old_pf_address, stride, degree] = active_lookahead.value();
    if (degree == 0) {
      active_lookahead.reset();
      return;
    }
    assert(degree > 0);

    auto addr_delta = stride * BLOCK_SIZE;
    auto pf_address = static_cast<uint64_t>(static_cast<int64_t>(old_pf_address) + addr_delta); // cast to signed to allow negative strides

    // If the next step would exceed the degree or run off the page, stop
    if (cache->virtual_prefetch || (pf_address >> LOG2_PAGE_SIZE) == (old_pf_address >> LOG2_PAGE_SIZE)) {
      // check the MSHR occupancy to decide if we're going to prefetch to this level or not
      bool success = cache->prefetch_line(pf_address, (cache->get_mshr_occupancy_ratio() < 0.5), 0);
      if (success)
        active_lookahead = {pf_address, stride, degree - 1};
      // If we fail, try again next cycle

      if (active_lookahead->degree == 0) {
        active_lookahead.reset();
      }
    } else {
      active_lookahead.reset();
    }
  }
};

std::map<CACHE*, tracker> trackers;
} // namespace

void CACHE::prefetcher_initialize() {}

void CACHE::prefetcher_cycle_operate()
{
  ::trackers[this].advance_lookahead(this);

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

uint32_t CACHE::prefetcher_cache_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, uint8_t type, uint32_t metadata_in)
{
  ::private_data[this].cache_operate_cnt += 1;

  ::trackers[this].initiate_lookahead(ip, addr >> LOG2_BLOCK_SIZE, ::private_data[this].degree);
  return metadata_in;
}

uint32_t CACHE::prefetcher_cache_fill(uint64_t addr, uint32_t set, uint32_t way, uint8_t prefetch, uint64_t evicted_addr, uint32_t metadata_in)
{
  return metadata_in;
}

void CACHE::prefetcher_final_stats() {}
