#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include <pyc/cpp/pyc_tb.hpp>

#include "linxcore_ooo_pyc.hpp"

using pyc::cpp::Testbench;
using pyc::cpp::Wire;

namespace {

constexpr std::uint64_t kBootPc = 0x0000'0000'0001'0000ull;
constexpr std::uint64_t kBootSp = 0x0000'0000'0002'0000ull;
constexpr std::uint64_t kDefaultMaxCycles = 50000000ull;
constexpr std::uint64_t kDefaultDeadlockCycles = 200000ull;
constexpr const char *kDefaultDisasmTool = "/Users/zhoubot/linxisa/tools/isa/linxdisasm.py";
constexpr const char *kDefaultDisasmSpec = "/Users/zhoubot/linxisa/isa/spec/current/linxisa-v0.3.json";

static bool envFlag(const char *name) {
  const char *v = std::getenv(name);
  if (!v)
    return false;
  return !(v[0] == '0' && v[1] == '\0');
}

template <typename MemT>
static bool loadMemh(MemT &mem, const std::string &path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cerr << "ERROR: failed to open memh: " << path << "\n";
    return false;
  }

  std::uint64_t addr = 0;
  std::string tok;
  while (f >> tok) {
    if (tok.empty())
      continue;
    if (tok[0] == '@') {
      addr = std::stoull(tok.substr(1), nullptr, 16);
      continue;
    }
    unsigned v = std::stoul(tok, nullptr, 16) & 0xFFu;
    mem.pokeByte(static_cast<std::size_t>(addr), static_cast<std::uint8_t>(v));
    addr++;
  }
  return true;
}

static std::uint64_t maskInsn(std::uint64_t raw, std::uint8_t len) {
  switch (len) {
  case 2:
    return raw & 0xFFFFu;
  case 4:
    return raw & 0xFFFF'FFFFu;
  case 6:
    return raw & 0xFFFF'FFFF'FFFFu;
  default:
    return raw;
  }
}

static std::string toHex(std::uint64_t v) {
  std::ostringstream oss;
  oss << "0x" << std::hex << v << std::dec;
  return oss.str();
}

static std::string insnHexToken(std::uint64_t raw, std::uint8_t len) {
  unsigned digits = 16;
  if (len == 2)
    digits = 4;
  else if (len == 4)
    digits = 8;
  else if (len == 6)
    digits = 12;
  std::ostringstream oss;
  oss << std::hex << std::nouppercase << std::setfill('0') << std::setw(static_cast<int>(digits)) << maskInsn(raw, len);
  return oss.str();
}

static std::string shellQuote(const std::string &s) {
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('\'');
  for (char ch : s) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out.push_back('\'');
  return out;
}

static std::optional<std::string> runCommandCapture(const std::string &cmd) {
  FILE *fp = ::popen(cmd.c_str(), "r");
  if (!fp)
    return std::nullopt;
  std::string out;
  char buf[512];
  while (std::fgets(buf, sizeof(buf), fp) != nullptr) {
    out += buf;
  }
  const int rc = ::pclose(fp);
  if (rc != 0)
    return std::nullopt;
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) {
    out.pop_back();
  }
  return out;
}

static std::string disasmInsn(const std::string &tool, const std::string &spec, std::uint64_t raw, std::uint8_t len) {
  const std::string token = insnHexToken(raw, len);
  const std::string cmd =
      "python3 " + shellQuote(tool) + " --spec " + shellQuote(spec) + " --hex " + shellQuote(token) + " 2>/dev/null";
  const auto out = runCommandCapture(cmd);
  if (!out.has_value() || out->empty())
    return "<disasm-unavailable>";
  const std::size_t tab = out->find('\t');
  if (tab == std::string::npos || tab + 1 >= out->size())
    return *out;
  return out->substr(tab + 1);
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " <program.memh>\n";
    return 2;
  }
  const std::string memhPath = argv[1];

  pyc::gen::linxcore_ooo_pyc dut{};

  if (!loadMemh(dut.mem2r1w.imem, memhPath) || !loadMemh(dut.mem2r1w.dmem, memhPath)) {
    return 2;
  }

  std::uint64_t bootPc = kBootPc;
  std::uint64_t bootSp = kBootSp;
  std::uint64_t bootRa = bootSp + 0x10000ull;
  if (const char *env = std::getenv("PYC_BOOT_PC"))
    bootPc = static_cast<std::uint64_t>(std::stoull(env, nullptr, 0));
  if (const char *env = std::getenv("PYC_BOOT_SP"))
    bootSp = static_cast<std::uint64_t>(std::stoull(env, nullptr, 0));
  if (const char *env = std::getenv("PYC_BOOT_RA"))
    bootRa = static_cast<std::uint64_t>(std::stoull(env, nullptr, 0));

  dut.boot_pc = Wire<64>(bootPc);
  dut.boot_sp = Wire<64>(bootSp);
  dut.boot_ra = Wire<64>(bootRa);
  dut.host_wvalid = Wire<1>(0);
  dut.host_waddr = Wire<64>(0);
  dut.host_wdata = Wire<64>(0);
  dut.host_wstrb = Wire<8>(0);

  Testbench<pyc::gen::linxcore_ooo_pyc> tb(dut);

  std::ofstream commitTrace{};
  if (const char *p = std::getenv("PYC_COMMIT_TRACE"); p && p[0] != '\0') {
    std::filesystem::path out(p);
    if (out.has_parent_path()) {
      std::filesystem::create_directories(out.parent_path());
    }
    commitTrace.open(out, std::ios::out | std::ios::trunc);
    if (!commitTrace.is_open()) {
      std::cerr << "WARN: cannot open commit trace output: " << out << "\n";
    }
  }

  const bool traceVcd = envFlag("PYC_VCD");
  if (traceVcd) {
    std::filesystem::create_directories("/Users/zhoubot/LinxCore/generated/cpp/linxcore_ooo_pyc");
    tb.enableVcd("/Users/zhoubot/LinxCore/generated/cpp/linxcore_ooo_pyc/tb_linxcore_ooo_pyc.vcd",
                 "tb_linxcore_ooo_pyc");
    tb.vcdTrace(dut.clk, "clk");
    tb.vcdTrace(dut.rst, "rst");
    tb.vcdTrace(dut.cycles, "cycles");
    tb.vcdTrace(dut.halted, "halted");
    tb.vcdTrace(dut.pc, "pc");
  }

  std::uint64_t maxCycles = kDefaultMaxCycles;
  if (const char *env = std::getenv("PYC_MAX_CYCLES"))
    maxCycles = static_cast<std::uint64_t>(std::stoull(env, nullptr, 0));
  std::uint64_t deadlockCycles = kDefaultDeadlockCycles;
  if (const char *env = std::getenv("PYC_DEADLOCK_CYCLES"))
    deadlockCycles = static_cast<std::uint64_t>(std::stoull(env, nullptr, 0));
  std::string disasmTool = kDefaultDisasmTool;
  if (const char *env = std::getenv("PYC_DISASM_TOOL"); env && env[0] != '\0')
    disasmTool = env;
  std::string disasmSpec = kDefaultDisasmSpec;
  if (const char *env = std::getenv("PYC_DISASM_SPEC"); env && env[0] != '\0')
    disasmSpec = env;

  tb.addClock(dut.clk, 1);
  tb.reset(dut.rst, 2, 1);

  std::uint64_t commitTraceSeq = 0;
  std::uint64_t retiredCount = 0;
  std::uint64_t noRetireStreak = 0;

  while (dut.cycles.value() < maxCycles) {
    tb.runCycles(1);
    bool retiredThisCycle = false;

    for (int slot = 0; slot < 4; slot++) {
      bool fire = false;
      std::uint64_t pc = 0;
      std::uint64_t insnRaw = 0;
      std::uint8_t len = 0;
      bool wbValid = false;
      std::uint32_t wbRd = 0;
      std::uint64_t wbData = 0;
      bool memValid = false;
      bool memIsStore = false;
      std::uint64_t memAddr = 0;
      std::uint64_t memWdata = 0;
      std::uint64_t memRdata = 0;
      std::uint64_t memSize = 0;
      bool trapValid = false;
      std::uint32_t trapCause = 0;
      std::uint64_t nextPc = 0;

      if (slot == 0) {
        fire = dut.commit_fire0.toBool();
        pc = dut.commit_pc0.value();
        insnRaw = dut.commit_insn_raw0.value();
        len = static_cast<std::uint8_t>(dut.commit_len0.value() & 0x7u);
        wbValid = dut.commit_wb_valid0.toBool();
        wbRd = static_cast<std::uint32_t>(dut.commit_wb_rd0.value());
        wbData = dut.commit_wb_data0.value();
        memValid = dut.commit_mem_valid0.toBool();
        memIsStore = dut.commit_mem_is_store0.toBool();
        memAddr = dut.commit_mem_addr0.value();
        memWdata = dut.commit_mem_wdata0.value();
        memRdata = dut.commit_mem_rdata0.value();
        memSize = dut.commit_mem_size0.value();
        trapValid = dut.commit_trap_valid0.toBool();
        trapCause = static_cast<std::uint32_t>(dut.commit_trap_cause0.value());
        nextPc = dut.commit_next_pc0.value();
      } else if (slot == 1) {
        fire = dut.commit_fire1.toBool();
        pc = dut.commit_pc1.value();
        insnRaw = dut.commit_insn_raw1.value();
        len = static_cast<std::uint8_t>(dut.commit_len1.value() & 0x7u);
        wbValid = dut.commit_wb_valid1.toBool();
        wbRd = static_cast<std::uint32_t>(dut.commit_wb_rd1.value());
        wbData = dut.commit_wb_data1.value();
        memValid = dut.commit_mem_valid1.toBool();
        memIsStore = dut.commit_mem_is_store1.toBool();
        memAddr = dut.commit_mem_addr1.value();
        memWdata = dut.commit_mem_wdata1.value();
        memRdata = dut.commit_mem_rdata1.value();
        memSize = dut.commit_mem_size1.value();
        trapValid = dut.commit_trap_valid1.toBool();
        trapCause = static_cast<std::uint32_t>(dut.commit_trap_cause1.value());
        nextPc = dut.commit_next_pc1.value();
      } else if (slot == 2) {
        fire = dut.commit_fire2.toBool();
        pc = dut.commit_pc2.value();
        insnRaw = dut.commit_insn_raw2.value();
        len = static_cast<std::uint8_t>(dut.commit_len2.value() & 0x7u);
        wbValid = dut.commit_wb_valid2.toBool();
        wbRd = static_cast<std::uint32_t>(dut.commit_wb_rd2.value());
        wbData = dut.commit_wb_data2.value();
        memValid = dut.commit_mem_valid2.toBool();
        memIsStore = dut.commit_mem_is_store2.toBool();
        memAddr = dut.commit_mem_addr2.value();
        memWdata = dut.commit_mem_wdata2.value();
        memRdata = dut.commit_mem_rdata2.value();
        memSize = dut.commit_mem_size2.value();
        trapValid = dut.commit_trap_valid2.toBool();
        trapCause = static_cast<std::uint32_t>(dut.commit_trap_cause2.value());
        nextPc = dut.commit_next_pc2.value();
      } else {
        fire = dut.commit_fire3.toBool();
        pc = dut.commit_pc3.value();
        insnRaw = dut.commit_insn_raw3.value();
        len = static_cast<std::uint8_t>(dut.commit_len3.value() & 0x7u);
        wbValid = dut.commit_wb_valid3.toBool();
        wbRd = static_cast<std::uint32_t>(dut.commit_wb_rd3.value());
        wbData = dut.commit_wb_data3.value();
        memValid = dut.commit_mem_valid3.toBool();
        memIsStore = dut.commit_mem_is_store3.toBool();
        memAddr = dut.commit_mem_addr3.value();
        memWdata = dut.commit_mem_wdata3.value();
        memRdata = dut.commit_mem_rdata3.value();
        memSize = dut.commit_mem_size3.value();
        trapValid = dut.commit_trap_valid3.toBool();
        trapCause = static_cast<std::uint32_t>(dut.commit_trap_cause3.value());
        nextPc = dut.commit_next_pc3.value();
      }

      if (!fire)
        continue;
      retiredThisCycle = true;
      retiredCount++;
      if (!commitTrace.is_open())
        continue;

      const std::uint64_t insn = maskInsn(insnRaw, len);
      commitTrace << "{"
                  << "\"cycle\":" << commitTraceSeq++ << ","
                  << "\"pc\":" << pc << ","
                  << "\"insn\":" << insn << ","
                  << "\"len\":" << static_cast<unsigned>(len) << ","
                  << "\"wb_valid\":" << (wbValid ? 1 : 0) << ","
                  << "\"wb_rd\":" << wbRd << ","
                  << "\"wb_data\":" << wbData << ","
                  << "\"mem_valid\":" << (memValid ? 1 : 0) << ","
                  << "\"mem_is_store\":" << (memIsStore ? 1 : 0) << ","
                  << "\"mem_addr\":" << memAddr << ","
                  << "\"mem_wdata\":" << memWdata << ","
                  << "\"mem_rdata\":" << memRdata << ","
                  << "\"mem_size\":" << memSize << ","
                  << "\"trap_valid\":" << (trapValid ? 1 : 0) << ","
                  << "\"trap_cause\":" << trapCause << ","
                  << "\"traparg0\":0,"
                  << "\"next_pc\":" << nextPc
                  << "}\n";
    }

    if (retiredThisCycle) {
      noRetireStreak = 0;
    } else {
      noRetireStreak++;
      if (deadlockCycles > 0 && noRetireStreak >= deadlockCycles && !dut.halted.toBool() && !dut.mmio_exit_valid.toBool()) {
        std::uint8_t headLen = static_cast<std::uint8_t>(dut.rob_head_len.value() & 0x7u);
        if (headLen != 2 && headLen != 4 && headLen != 6)
          headLen = 4;
        const std::uint64_t headInsn = dut.rob_head_insn_raw.value();
        const std::string disasm = disasmInsn(disasmTool, disasmSpec, headInsn, headLen);
        std::cerr << "error: deadlock detected after " << noRetireStreak << " cycles without retire\n"
                  << "  cycle=" << dut.cycles.value() << " pc=" << toHex(dut.pc.value()) << " fpc=" << toHex(dut.fpc.value())
                  << " rob_count=" << dut.rob_count.value() << "\n"
                  << "  rob_head_valid=" << dut.rob_head_valid.value() << " rob_head_done=" << dut.rob_head_done.value()
                  << " rob_head_pc=" << toHex(dut.rob_head_pc.value()) << "\n"
                  << "  rob_head_op=" << dut.rob_head_op.value() << " rob_head_len=" << static_cast<unsigned>(headLen)
                  << " rob_head_insn=" << toHex(maskInsn(headInsn, headLen)) << "\n"
                  << "  head_wait_hit=" << dut.head_wait_hit.value()
                  << " head_wait_kind=" << dut.head_wait_kind.value()
                  << " sl=" << dut.head_wait_sl.value() << " sr=" << dut.head_wait_sr.value() << " sp=" << dut.head_wait_sp.value()
                  << " sl_rdy=" << dut.head_wait_sl_rdy.value()
                  << " sr_rdy=" << dut.head_wait_sr_rdy.value()
                  << " sp_rdy=" << dut.head_wait_sp_rdy.value() << "\n"
                  << "  rob_head_disasm=" << disasm << "\n";
        return 1;
      }
    }

    if (dut.mmio_uart_valid.toBool() && !dut.halted.toBool()) {
      const char ch = static_cast<char>(dut.mmio_uart_data.value() & 0xFFu);
      std::cout << ch << std::flush;
    }

    if (dut.mmio_exit_valid.toBool() || dut.halted.toBool()) {
      std::cout << "\nok: program exited, cycles=" << dut.cycles.value() << " commits=" << retiredCount << "\n";
      return 0;
    }
  }

  std::cerr << "error: max cycles reached: " << maxCycles << "\n";
  return 1;
}
