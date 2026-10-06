#include "hal_mock.hpp"

#include <algorithm>
#include <deque>
#include <sstream>

namespace {

enum class Op { kWrite, kWriteRead };

struct Expectation {
    Op op;
    uint8_t addr;
    std::vector<uint8_t> wdata;
    std::vector<uint8_t> response;
    hal_status_t result;
};

struct MockState {
    std::deque<Expectation> queue;
    std::vector<std::string> errors;
    uint32_t now_us = 0;
    uint32_t last_timeout_us = 0;
    uint64_t total_delay_us = 0;
    std::size_t bus_calls = 0;
};

MockState &State()
{
    static MockState state;
    return state;
}

std::string Hex(const uint8_t *data, std::size_t len)
{
    std::ostringstream os;
    os << std::hex;
    for (std::size_t i = 0; i < len; ++i) {
        os << (i ? " " : "") << static_cast<int>(data[i]);
    }
    return os.str();
}

// Pops the next expectation and checks it against the call. Returns false
// (and records why) if there is none or it does not match.
bool Match(Op op, uint8_t addr, const uint8_t *wdata, std::size_t wlen, std::size_t rlen,
           Expectation *out)
{
    MockState &s = State();
    s.bus_calls++;
    if (s.queue.empty()) {
        s.errors.push_back("unexpected bus call to 0x" + Hex(&addr, 1) + " writing [" +
                           Hex(wdata, wlen) + "]");
        return false;
    }
    Expectation e = s.queue.front();
    s.queue.pop_front();
    const bool same = e.op == op && e.addr == addr && e.wdata.size() == wlen &&
                      std::equal(e.wdata.begin(), e.wdata.end(), wdata) &&
                      (op == Op::kWrite || e.response.size() == rlen);
    if (!same) {
        s.errors.push_back("expected [" + Hex(e.wdata.data(), e.wdata.size()) + "] got [" +
                           Hex(wdata, wlen) + "]");
        return false;
    }
    *out = e;
    return true;
}

}  // namespace

namespace hal_mock {

void Reset() { State() = MockState{}; }

void ExpectWrite(uint8_t addr, std::vector<uint8_t> data, hal_status_t result)
{
    State().queue.push_back({Op::kWrite, addr, std::move(data), {}, result});
}

void ExpectWriteRead(uint8_t addr, std::vector<uint8_t> wdata, std::vector<uint8_t> response,
                     hal_status_t result)
{
    State().queue.push_back({Op::kWriteRead, addr, std::move(wdata), std::move(response), result});
}

const std::vector<std::string> &Errors() { return State().errors; }
std::size_t Remaining() { return State().queue.size(); }
uint32_t LastTimeoutUs() { return State().last_timeout_us; }
uint64_t TotalDelayUs() { return State().total_delay_us; }
std::size_t BusCalls() { return State().bus_calls; }

}  // namespace hal_mock

extern "C" {

hal_status_t hal_i2c_write(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *data, size_t len,
                           uint32_t timeout_us)
{
    State().last_timeout_us = timeout_us;
    Expectation e;
    if (bus == nullptr || data == nullptr || !Match(Op::kWrite, addr7, data, len, 0, &e)) {
        return HAL_ERR_BUS;
    }
    return e.result;
}

hal_status_t hal_i2c_write_read(hal_i2c_bus_t *bus, uint8_t addr7, const uint8_t *wdata,
                                size_t wlen, uint8_t *rdata, size_t rlen, uint32_t timeout_us)
{
    State().last_timeout_us = timeout_us;
    Expectation e;
    if (bus == nullptr || wdata == nullptr || rdata == nullptr ||
        !Match(Op::kWriteRead, addr7, wdata, wlen, rlen, &e)) {
        return HAL_ERR_BUS;
    }
    if (e.result == HAL_OK) {
        std::copy(e.response.begin(), e.response.end(), rdata);
    }
    return e.result;
}

uint32_t hal_time_us(void) { return State().now_us; }

void hal_delay_us(uint32_t us)
{
    State().total_delay_us += us;
    State().now_us += us;
}

hal_status_t hal_periodic_init(hal_periodic_t *p, uint32_t period_us)
{
    if (p == nullptr || period_us == 0) {
        return HAL_ERR_PARAM;
    }
    p->period_us = period_us;
    p->last_release = State().now_us;
    p->started = true;
    return HAL_OK;
}

hal_status_t hal_periodic_wait(hal_periodic_t *p, bool *missed)
{
    if (p == nullptr || missed == nullptr || !p->started) {
        return HAL_ERR_PARAM;
    }
    p->last_release += p->period_us;
    *missed = false;
    State().now_us = p->last_release;
    return HAL_OK;
}

void hal_console_write(const char *, size_t) {}

}  // extern "C"
