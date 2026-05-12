# AITrace CLI & Skill — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `aitrace` — a C++ CLI tool that exposes MCU debug data (RTT shell, waveform, OpenOCD registers, GDB, symbol resolution) to AI via stdout, plus a Claude Code skill that orchestrates diagnostic workflows.

**Architecture:** Existing mstudio source files are NOT modified. A new `core/` directory compiles the 5 reusable classes from their original locations into `libmstudiocore.a`. `aitrace/` links against it. mstudio's own Makefile stays untouched. The skill file is a new `.md` file in modus_template.

**Tech Stack:** C++17, MinGW-w64 Clang, Winsock (ws2_32), GDB subprocess (batch mode). No GUI dependencies.

---

## File Structure

```
E:\Project\mstudio\
├── src/                    # UNCHANGED — mstudio GUI
│   ├── network_mgr.h/cpp   #   (compiled by core/ Makefile from here)
│   ├── protocol_parser.h/cpp
│   ├── utils/ocd_client.h/cpp
│   ├── utils/elf_parser.h/cpp
│   └── utils/map_parser.h/cpp
├── core/                   # NEW — static library
│   └── Makefile            #   compiles libmstudiocore.a from ../src/
├── aitrace/                # NEW — CLI tool
│   ├── Makefile
│   └── src/
│       ├── main.cpp
│       ├── shell_cmd.h / shell_cmd.cpp
│       ├── wave_cmd.h  / wave_cmd.cpp
│       ├── ocd_cmd.h   / ocd_cmd.cpp
│       ├── gdb_cmd.h   / gdb_cmd.cpp
│       ├── map_cmd.h   / map_cmd.cpp
│       └── crash_cmd.h / crash_cmd.cpp
└── Makefile                # UNCHANGED — mstudio build

E:\Project\modus_template\
└── .claude/skills/
    └── aitrace-skill.md    # NEW — skill definition
```

---

### Task 1: Create `core/Makefile` — static library

**Files:**
- Create: `E:\Project\mstudio\core\Makefile`

- [ ] **Step 1: Write core/Makefile**

This Makefile compiles the 5 classes from their ORIGINAL locations into `libmstudiocore.a`. No source files are moved.

```makefile
# Builds libmstudiocore.a from original source files in ../src/
MSYS64_PATH = D:/software/msys64
CXX = $(MSYS64_PATH)/mingw64/bin/clang++
AR = $(MSYS64_PATH)/mingw64/bin/llvm-ar
CFLAGS = -Wall -Wextra -O2 -g
CXXFLAGS = -std=c++17 $(CFLAGS)

SRC_TOP = ../src
BUILD_DIR = build

INCLUDES = -I$(SRC_TOP) -I$(SRC_TOP)/utils

# Source files (original locations, not moved)
SRCS = $(SRC_TOP)/network_mgr.cpp \
       $(SRC_TOP)/protocol_parser.cpp \
       $(SRC_TOP)/utils/ocd_client.cpp \
       $(SRC_TOP)/utils/elf_parser.cpp \
       $(SRC_TOP)/utils/map_parser.cpp

OBJS = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(notdir $(SRCS))))

TARGET = $(BUILD_DIR)/libmstudiocore.a

vpath %.cpp $(SRC_TOP) $(SRC_TOP)/utils

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(AR) rcs $@ $^

$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	@rm -rf $(BUILD_DIR)

.PHONY: all clean
```

- [ ] **Step 2: Build and verify**

```bash
cd /e/Project/mstudio/core
make clean && make
```

Expected: `build/libmstudiocore.a` created, containing 5 .o files.

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add core/Makefile
git commit -m "add: core static library build for mstudio reusable classes

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 2: Create `aitrace/src/main.cpp` — CLI entry point & subcommand routing

**Files:**
- Create: `E:\Project\mstudio\aitrace\src\main.cpp`

- [ ] **Step 1: Write main.cpp**

```cpp
#include <iostream>
#include <string>
#include <cstring>

// Forward declare command handlers — each returns 0 on success
int shell_main(int argc, char* argv[]);
int wave_main(int argc, char* argv[]);
int ocd_main(int argc, char* argv[]);
int gdb_main(int argc, char* argv[]);
int map_main(int argc, char* argv[]);
int crash_main(int argc, char* argv[]);

static void PrintUsage() {
    std::cout << "aitrace — AI-driven MCU debugging CLI\n\n"
              << "Usage: aitrace <command> [args...]\n\n"
              << "Passive (no intrusion):\n"
              << "  shell <cmd...>       Send command via RTT Ch0 (TCP 9090)\n"
              << "  wave  capture <sec>  Capture waveform to CSV (TCP 9091)\n"
              << "  wave  list|start|stop|rate <n>\n\n"
              << "Halt-based (intrusive):\n"
              << "  ocd   halt|resume|regs|peek <addr>|mdw <addr> [n]|stack [n]\n\n"
              << "GDB (intrusive, requires explicit enable):\n"
              << "  gdb   connect|break <loc>|continue|step|print <expr>|bt|detach\n\n"
              << "Analysis:\n"
              << "  map   resolve <elf> <addr...>|info <elf_or_map>\n"
              << "  crash report --pc=<hex> --lr=<hex> --sp=<hex> --elf=<path>\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage();
        return 1;
    }

    std::string cmd = argv[1];

    if (cmd == "shell")  return shell_main(argc - 1, argv + 1);
    if (cmd == "wave")   return wave_main(argc - 1, argv + 1);
    if (cmd == "ocd")    return ocd_main(argc - 1, argv + 1);
    if (cmd == "gdb")    return gdb_main(argc - 1, argv + 1);
    if (cmd == "map")    return map_main(argc - 1, argv + 1);
    if (cmd == "crash")  return crash_main(argc - 1, argv + 1);

    std::cerr << "Unknown command: " << cmd << "\n";
    PrintUsage();
    return 1;
}
```

- [ ] **Step 2: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/src/main.cpp
git commit -m "add: aitrace CLI entry point with subcommand routing

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 3: Create `aitrace/src/shell_cmd.h` and `shell_cmd.cpp` — RTT Shell commands

**Files:**
- Create: `E:\Project\mstudio\aitrace\src\shell_cmd.h`
- Create: `E:\Project\mstudio\aitrace\src\shell_cmd.cpp`

- [ ] **Step 1: Write shell_cmd.h**

```cpp
#ifndef AITRACE_SHELL_CMD_H
#define AITRACE_SHELL_CMD_H
int shell_main(int argc, char* argv[]);
#endif
```

- [ ] **Step 2: Write shell_cmd.cpp**

Connects to TCP 9090, sends the shell command text, reads response with 2s timeout, prints to stdout. Uses Winsock directly.

```cpp
#include "shell_cmd.h"
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET (-1)
#define closesocket close
#endif

static bool EnsureWSA() {
#ifdef _WIN32
    static bool init = false;
    if (!init) {
        WSADATA d;
        WSAStartup(MAKEWORD(2, 2), &d);
        init = true;
    }
#endif
    return true;
}

static SOCKET ConnectTCP(const char* host, int port) {
    EnsureWSA();
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) return s;

#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(s, FIONBIO, &mode);
#else
    fcntl(s, F_SETFL, O_NONBLOCK);
#endif

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
#ifdef _WIN32
    inet_pton(AF_INET, host, &addr.sin_addr);
#else
    addr.sin_addr.s_addr = inet_addr(host);
#endif

    connect(s, (sockaddr*)&addr, sizeof(addr));

    fd_set wset, eset;
    FD_ZERO(&wset); FD_SET(s, &wset);
    FD_ZERO(&eset); FD_SET(s, &eset);
    timeval tv = {1, 0}; // 1s timeout
    select((int)(s + 1), nullptr, &wset, &eset, &tv);

    if (!FD_ISSET(s, &wset)) {
        closesocket(s);
        return INVALID_SOCKET;
    }
    return s;
}

static std::string RecvAll(SOCKET s, int timeout_ms) {
    std::string result;
    char buf[4096];
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        fd_set rset;
        FD_ZERO(&rset); FD_SET(s, &rset);
        timeval tv = {0, 100000}; // 100ms
        int ret = select((int)(s + 1), &rset, nullptr, nullptr, &tv);
        if (ret <= 0) {
            if (!result.empty()) break; // Got some data, no more coming
            continue;
        }
        int bytes = recv(s, buf, sizeof(buf) - 1, 0);
        if (bytes <= 0) break;
        result.append(buf, bytes);
    }
    return result;
}

int shell_main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: aitrace shell <cmd...>\n";
        return 1;
    }

    // Reconstruct command string from args
    std::string cmd;
    for (int i = 1; i < argc; i++) {
        if (i > 1) cmd += " ";
        cmd += argv[i];
    }
    cmd += "\r\n";

    SOCKET s = ConnectTCP("127.0.0.1", 9090);
    if (s == INVALID_SOCKET) {
        std::cerr << "Failed to connect to RTT Ch0 (TCP 9090). Is OpenOCD running?\n";
        return 1;
    }

    int sent = send(s, cmd.c_str(), (int)cmd.size(), 0);
    if (sent < 0) {
        std::cerr << "Failed to send command.\n";
        closesocket(s);
        return 1;
    }

    // Read response with 2s total timeout
    std::string response = RecvAll(s, 2000);
    std::cout << response;
    if (!response.empty() && response.back() != '\n') std::cout << "\n";

    closesocket(s);
    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/src/shell_cmd.h aitrace/src/shell_cmd.cpp
git commit -m "add: aitrace shell command — RTT Ch0 text send/recv

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 4: Create `aitrace/src/wave_cmd.h` and `wave_cmd.cpp` — Waveform capture

**Files:**
- Create: `E:\Project\mstudio\aitrace\src\wave_cmd.h`
- Create: `E:\Project\mstudio\aitrace\src\wave_cmd.cpp`

- [ ] **Step 1: Write wave_cmd.h**

```cpp
#ifndef AITRACE_WAVE_CMD_H
#define AITRACE_WAVE_CMD_H
int wave_main(int argc, char* argv[]);
#endif
```

- [ ] **Step 2: Write wave_cmd.cpp**

Uses `ProtocolParser` from libmstudiocore to parse binary frames from TCP 9091. Control commands (start/stop/rate/list) go via RTT Ch0 (TCP 9090). Capture reads from Ch1 (TCP 9091) and outputs CSV.

```cpp
#include "wave_cmd.h"
#include "protocol_parser.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET (-1)
#define closesocket close
#endif

namespace {
bool EnsureWSA() {
#ifdef _WIN32
    static bool init = false;
    if (!init) {
        WSADATA d;
        WSAStartup(MAKEWORD(2, 2), &d);
        init = true;
    }
#endif
    return true;
}

SOCKET ConnectTCP(const char* host, int port) {
    EnsureWSA();
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) return s;
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(s, FIONBIO, &mode);
#endif
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
#ifdef _WIN32
    inet_pton(AF_INET, host, &addr.sin_addr);
#else
    addr.sin_addr.s_addr = inet_addr(host);
#endif
    connect(s, (sockaddr*)&addr, sizeof(addr));
    fd_set wset, eset;
    FD_ZERO(&wset); FD_SET(s, &wset);
    FD_ZERO(&eset); FD_SET(s, &eset);
    timeval tv = {1, 0};
    select((int)(s + 1), nullptr, &wset, &eset, &tv);
    if (!FD_ISSET(s, &wset)) { closesocket(s); return INVALID_SOCKET; }
    return s;
}

static void SendShellCmd(const std::string& cmd) {
    SOCKET s = ConnectTCP("127.0.0.1", 9090);
    if (s == INVALID_SOCKET) {
        std::cerr << "Failed to connect to RTT Ch0 (TCP 9090).\n";
        return;
    }
    std::string line = cmd + "\r\n";
    send(s, line.c_str(), (int)line.size(), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    char buf[4096]; int n;
    bool got_data = false;
    while ((n = recv(s, buf, sizeof(buf) - 1, 0)) > 0) {
        buf[n] = 0; std::cout << buf; got_data = true;
    }
    closesocket(s);
    if (!got_data) std::cout << "OK\n";
}

} // anonymous namespace

static void PrintUsage() {
    std::cerr << "Usage: aitrace wave <subcommand> [args]\n"
              << "  list                    List channels\n"
              << "  start                   Start acquisition\n"
              << "  stop                    Stop acquisition\n"
              << "  rate <n>                Set decimation rate\n"
              << "  capture <seconds>       Capture CSV to stdout\n"
              << "  capture <s> --output <f> Capture CSV to file\n";
}

int wave_main(int argc, char* argv[]) {
    if (argc < 2) { PrintUsage(); return 1; }

    std::string sub = argv[1];

    // Control commands go via RTT Ch0
    if (sub == "start" || sub == "stop" || sub == "list" || sub == "rate") {
        std::string cmd = "wave " + sub;
        if (sub == "rate") {
            if (argc < 3) { std::cerr << "Usage: aitrace wave rate <n>\n"; return 1; }
            cmd += " " + std::string(argv[2]);
        }
        SendShellCmd(cmd);
        return 0;
    }

    // capture <seconds> [--output <file>]
    if (sub == "capture") {
        if (argc < 3) { std::cerr << "Usage: aitrace wave capture <seconds>\n"; return 1; }
        double duration = std::stod(argv[2]);
        std::string outfile;
        for (int i = 3; i < argc; i++) {
            if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
                outfile = argv[++i];
            }
        }

        SOCKET s = ConnectTCP("127.0.0.1", 9091);
        if (s == INVALID_SOCKET) {
            std::cerr << "Failed to connect to RTT Ch1 (TCP 9091).\n";
            return 1;
        }

        ProtocolParser parser(16);
        bool header_written = false;

        std::ofstream file_out;
        if (!outfile.empty()) {
            file_out.open(outfile);
            if (!file_out.is_open()) {
                std::cerr << "Failed to open output file: " << outfile << "\n";
                closesocket(s);
                return 1;
            }
        }
        std::ostream& out = outfile.empty() ? std::cout : file_out;

        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::duration<double>(duration);
        auto hard_deadline = deadline + std::chrono::seconds(2);

        while (std::chrono::steady_clock::now() < hard_deadline) {
            fd_set rset;
            FD_ZERO(&rset); FD_SET(s, &rset);
            timeval tv = {0, 100000}; // 100ms
            int ret = select((int)(s + 1), &rset, nullptr, nullptr, &tv);
            if (ret <= 0) {
                if (std::chrono::steady_clock::now() > deadline) break;
                continue;
            }

            uint8_t buf[4096];
            int bytes = recv(s, (char*)buf, sizeof(buf), 0);
            if (bytes <= 0) break;

            std::vector<uint8_t> raw(buf, buf + bytes);
            std::vector<DataSample> samples;
            parser.Feed(raw, samples);

            // Write CSV header from descriptor frame
            if (!header_written && !parser.GetChannels().empty()) {
                const auto& chs = parser.GetChannels();
                out << "time";
                for (size_t i = 0; i < chs.size(); i++) {
                    out << ",";
                    const std::string& name = chs[i].name;
                    size_t end = name.find_last_not_of(' ');
                    if (end != std::string::npos)
                        out.write(name.c_str(), end + 1);
                    else out << name;
                }
                out << "\n";
                header_written = true;
            }

            for (const auto& sample : samples) {
                out << sample.timestamp;
                const auto& chs = parser.GetChannels();
                for (size_t i = 0; i < chs.size(); i++) {
                    out << ",";
                    auto it = sample.ch_values.find((int)i);
                    if (it != sample.ch_values.end()) out << it->second;
                }
                out << "\n";
            }

            if (std::chrono::steady_clock::now() > deadline) break;
        }

        closesocket(s);
        return 0;
    }

    PrintUsage();
    return 1;
}
```

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/src/wave_cmd.h aitrace/src/wave_cmd.cpp
git commit -m "add: aitrace wave command — waveform capture to CSV

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 5: Create `aitrace/src/ocd_cmd.h` and `ocd_cmd.cpp` — OpenOCD commands

**Files:**
- Create: `E:\Project\mstudio\aitrace\src\ocd_cmd.h`
- Create: `E:\Project\mstudio\aitrace\src\ocd_cmd.cpp`

- [ ] **Step 1: Write ocd_cmd.h**

```cpp
#ifndef AITRACE_OCD_CMD_H
#define AITRACE_OCD_CMD_H
int ocd_main(int argc, char* argv[]);
#endif
```

- [ ] **Step 2: Write ocd_cmd.cpp**

Wraps `OcdClient` from libmstudiocore. Each subcommand creates a short-lived connection.

```cpp
#include "ocd_cmd.h"
#include "ocd_client.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <cstdlib>

static void PrintUsage() {
    std::cerr << "Usage: aitrace ocd <subcommand> [args]\n"
              << "  halt                   Halt CPU (intrusive!)\n"
              << "  resume                 Resume CPU\n"
              << "  regs                   Dump all core registers\n"
              << "  peek  <hex_addr>       Read uint32 at address\n"
              << "  mdw   <hex_addr> [n]   Dump n words of memory\n"
              << "  stack [depth]          Dump stack around SP\n";
}

static uint32_t ParseHex(const char* s) {
    return (uint32_t)std::strtoul(s, nullptr, 16);
}

int ocd_main(int argc, char* argv[]) {
    if (argc < 2) { PrintUsage(); return 1; }

    std::string sub = argv[1];

    OcdClient ocd;
    if (!ocd.Connect("127.0.0.1", 4444)) {
        std::cerr << "Failed to connect to OpenOCD (TCP 4444). Is OpenOCD running?\n";
        return 1;
    }

    if (sub == "halt") {
        bool ok = ocd.Halt();
        std::cout << (ok ? "CPU halted.\n" : "Halt failed.\n");
    } else if (sub == "resume") {
        bool ok = ocd.Resume();
        std::cout << (ok ? "CPU resumed.\n" : "Resume failed.\n");
    } else if (sub == "regs") {
        auto regs = ocd.GetRegs();
        for (const auto& r : regs) {
            std::cout << std::setw(8) << r.name << " : 0x"
                      << std::hex << std::setw(8) << std::setfill('0')
                      << r.value << std::dec << std::setfill(' ') << "\n";
        }
    } else if (sub == "peek") {
        if (argc < 3) { std::cerr << "Usage: aitrace ocd peek <hex_addr>\n"; return 1; }
        uint32_t addr = ParseHex(argv[2]);
        uint32_t val = ocd.ReadMem32(addr);
        std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
                  << val << std::dec << std::setfill(' ') << "\n";
    } else if (sub == "mdw") {
        if (argc < 3) { std::cerr << "Usage: aitrace ocd mdw <hex_addr> [count]\n"; return 1; }
        uint32_t addr = ParseHex(argv[2]);
        int count = (argc > 3) ? std::stoi(argv[3]) : 16;
        auto vals = ocd.ReadMemBlock32(addr, count);
        for (size_t i = 0; i < vals.size(); i++) {
            if (i % 4 == 0) {
                if (i > 0) std::cout << "\n";
                std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
                          << (addr + (uint32_t)(i * 4)) << ": ";
            }
            std::cout << std::hex << std::setw(8) << std::setfill('0')
                      << vals[i] << " ";
        }
        std::cout << std::dec << std::setfill(' ') << "\n";
    } else if (sub == "stack") {
        int depth = (argc > 2) ? std::stoi(argv[2]) : 32;
        auto regs = ocd.GetRegs();
        uint32_t sp = 0;
        for (const auto& r : regs) {
            if (r.name == "sp" || r.name == "msp") { sp = r.value; break; }
        }
        if (sp == 0) {
            std::cerr << "Could not read SP.\n";
            return 1;
        }
        sp &= ~0x3u;
        uint32_t start = sp - 32;
        auto vals = ocd.ReadMemBlock32(start, depth);
        for (size_t i = 0; i < vals.size(); i++) {
            uint32_t addr = start + (uint32_t)(i * 4);
            std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
                      << addr << ": 0x" << std::setw(8) << vals[i];
            if (addr == sp) std::cout << " <-- SP";
            std::cout << "\n";
        }
        std::cout << std::dec << std::setfill(' ') << "\n";
    } else {
        PrintUsage();
        return 1;
    }

    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/src/ocd_cmd.h aitrace/src/ocd_cmd.cpp
git commit -m "add: aitrace ocd command — OpenOCD telnet halt/regs/peek/mdw/stack

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 6: Create `aitrace/src/map_cmd.h` and `map_cmd.cpp` — Symbol resolution

**Files:**
- Create: `E:\Project\mstudio\aitrace\src\map_cmd.h`
- Create: `E:\Project\mstudio\aitrace\src\map_cmd.cpp`

- [ ] **Step 1: Write map_cmd.h**

```cpp
#ifndef AITRACE_MAP_CMD_H
#define AITRACE_MAP_CMD_H
int map_main(int argc, char* argv[]);
#endif
```

- [ ] **Step 2: Write map_cmd.cpp**

Uses `ElfParser` for `.elf` files, `MapParser` for `.map` files.

```cpp
#include "map_cmd.h"
#include "elf_parser.h"
#include "map_parser.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>

static void PrintUsage() {
    std::cerr << "Usage: aitrace map <subcommand> [args]\n"
              << "  resolve <elf_or_map> <addr1> [addr2...]    Address -> symbol+offset\n"
              << "  info    <elf_or_map>                        Section sizes\n";
}

static uint32_t ParseHex(const char* s) {
    return (uint32_t)std::strtoul(s, nullptr, 16);
}

static bool IsElf(const std::string& path) {
    return path.size() > 4 &&
        (path.substr(path.size() - 4) == ".elf" || path.substr(path.size() - 4) == ".ELF");
}

// Resolve addresses against ELF symbol table
static void ResolveWithElf(const std::string& path,
                           const std::vector<uint32_t>& addrs) {
    ElfParser elf;
    if (!elf.Load(path)) {
        std::cerr << "Failed to load ELF: " << path << "\n";
        return;
    }
    const auto& syms = elf.GetSymbols();
    for (uint32_t addr : addrs) {
        const ElfSymbol* best = nullptr;
        for (const auto& s : syms) {
            if (s.address <= addr && (!best || s.address > best->address))
                best = &s;
        }
        std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
                  << addr << std::dec << std::setfill(' ') << " : ";
        if (best && (addr - best->address) < 0x10000) {
            uint32_t off = addr - best->address;
            std::cout << best->name << " + 0x" << std::hex << off << std::dec;
            const char* type = (best->type == 2) ? " [FUNC]"
                             : (best->type == 1) ? " [OBJECT]" : "";
            std::cout << type;
        } else {
            std::cout << "<unknown>";
        }
        std::cout << "\n";
    }
}

// Resolve addresses against MAP symbol table
static void ResolveWithMap(const std::string& path,
                           const std::vector<uint32_t>& addrs) {
    MapParser map;
    if (!map.Load(path)) {
        std::cerr << "Failed to load MAP: " << path << "\n";
        return;
    }
    const auto& syms = map.GetSymbols();
    for (uint32_t addr : addrs) {
        const MapSymbol* best = nullptr;
        for (const auto& s : syms) {
            if (s.address <= addr && (!best || s.address > best->address))
                best = &s;
        }
        const MapSection* sec = map.FindSectionByAddr(addr);
        std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
                  << addr << std::dec << std::setfill(' ') << " : ";
        if (best && (addr - best->address) < 0x10000) {
            uint32_t off = addr - best->address;
            std::cout << best->name << " + 0x" << std::hex << off << std::dec;
        } else {
            std::cout << "<unknown>";
        }
        if (sec) std::cout << "  [" << sec->name << "]";
        std::cout << "\n";
    }
}

// Print section info for .map files
static void InfoMap(const std::string& path) {
    MapParser map;
    if (!map.Load(path)) {
        std::cerr << "Failed to load MAP: " << path << "\n";
        return;
    }
    std::cout << "MAP: " << path << "\n\nSections:\n";
    for (const auto& sec : map.GetSections()) {
        std::cout << "  " << std::setw(20) << std::left << sec.name
                  << "  VMA:0x" << std::hex << std::setw(8) << std::setfill('0') << sec.vma
                  << "  Size:" << std::dec << std::setw(8) << std::setfill(' ') << sec.size
                  << "  (" << sec.files.size() << " files)\n";
    }
    std::cout << std::dec << "\nTotal Flash: " << map.GetTotalFlash() << " bytes\n";
    std::cout << "Total RAM:   " << map.GetTotalRam() << " bytes\n";
}

// Print symbol counts for .elf files
static void InfoElf(const std::string& path) {
    ElfParser elf;
    if (!elf.Load(path)) {
        std::cerr << "Failed to load ELF: " << path << "\n";
        return;
    }
    auto funcs = elf.GetFunctions();
    auto vars  = elf.GetVariables();
    std::cout << "ELF: " << path << "\n";
    std::cout << "Symbols:   " << elf.GetSymbols().size() << "\n";
    std::cout << "Functions: " << funcs.size() << "\n";
    std::cout << "Variables: " << vars.size() << "\n";
}

int map_main(int argc, char* argv[]) {
    if (argc < 3) { PrintUsage(); return 1; }

    std::string sub = argv[1];
    std::string path = argv[2];

    if (sub == "resolve") {
        std::vector<uint32_t> addrs;
        for (int i = 3; i < argc; i++) addrs.push_back(ParseHex(argv[i]));
        if (addrs.empty()) { std::cerr << "At least one address required.\n"; return 1; }
        if (IsElf(path)) ResolveWithElf(path, addrs);
        else            ResolveWithMap(path, addrs);
    } else if (sub == "info") {
        if (IsElf(path)) InfoElf(path);
        else             InfoMap(path);
    } else {
        PrintUsage();
        return 1;
    }
    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/src/map_cmd.h aitrace/src/map_cmd.cpp
git commit -m "add: aitrace map command — address-to-symbol resolution

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 7: Create `aitrace/src/crash_cmd.h` and `crash_cmd.cpp` — Crash analysis

**Files:**
- Create: `E:\Project\mstudio\aitrace\src\crash_cmd.h`
- Create: `E:\Project\mstudio\aitrace\src\crash_cmd.cpp`

- [ ] **Step 1: Write crash_cmd.h**

```cpp
#ifndef AITRACE_CRASH_CMD_H
#define AITRACE_CRASH_CMD_H
int crash_main(int argc, char* argv[]);
#endif
```

- [ ] **Step 2: Write crash_cmd.cpp**

Takes PC/LR/SP from crash dump, resolves against ELF/Map, decodes CFSR.

```cpp
#include "crash_cmd.h"
#include "elf_parser.h"
#include "map_parser.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>

static uint32_t ParseHex(const char* s) {
    return (uint32_t)std::strtoul(s, nullptr, 16);
}

static void DecodeCFSR(uint32_t cfsr) {
    std::cout << "\nCFSR: 0x" << std::hex << std::setw(8) << std::setfill('0')
              << cfsr << std::dec << std::setfill(' ') << "\n";
    std::cout << "Fault type decode:\n";

    uint8_t ufsr = (cfsr >> 16) & 0xFF;
    if (ufsr) {
        if (ufsr & (1 << 0)) std::cout << "  - Undefined instruction\n";
        if (ufsr & (1 << 1)) std::cout << "  - Invalid state\n";
        if (ufsr & (1 << 2)) std::cout << "  - Invalid PC load\n";
        if (ufsr & (1 << 3)) std::cout << "  - No coprocessor\n";
        if (ufsr & (1 << 5)) std::cout << "  - Divide by zero\n";
        if (ufsr & (1 << 6)) std::cout << "  - Unaligned access\n";
    }

    uint8_t bfsr = (cfsr >> 8) & 0xFF;
    if (bfsr) {
        if (bfsr & (1 << 0)) std::cout << "  - Instruction bus error\n";
        if (bfsr & (1 << 1)) std::cout << "  - Precise data bus error\n";
        if (bfsr & (1 << 2)) std::cout << "  - Imprecise data bus error\n";
        if (bfsr & (1 << 3)) std::cout << "  - Unstack bus error\n";
        if (bfsr & (1 << 4)) std::cout << "  - Stack bus error\n";
        if (bfsr & (1 << 7)) std::cout << "  - BFAR valid\n";
    }

    uint8_t mmsr = cfsr & 0xFF;
    if (mmsr) {
        if (mmsr & (1 << 0)) std::cout << "  - Instruction access violation (MPU)\n";
        if (mmsr & (1 << 1)) std::cout << "  - Data access violation (MPU)\n";
        if (mmsr & (1 << 3)) std::cout << "  - Unstack MPU violation\n";
        if (mmsr & (1 << 4)) std::cout << "  - Stack MPU violation\n";
        if (mmsr & (1 << 7)) std::cout << "  - MMAR valid\n";
    }
}

static std::string ResolveAddrElf(ElfParser& elf, uint32_t addr) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setw(8) << std::setfill('0') << addr;
    const auto& syms = elf.GetSymbols();
    const ElfSymbol* best = nullptr;
    for (const auto& s : syms) {
        if (s.address <= addr && (!best || s.address > best->address)) best = &s;
    }
    if (best && (addr - best->address) < 0x8000) {
        uint32_t off = addr - best->address;
        oss << std::dec << std::setfill(' ') << "  ->  " << best->name;
        if (off > 0) oss << " + 0x" << std::hex << off;
        oss << ((best->type == 2) ? " (function)" : "");
    } else {
        oss << "  ->  <unknown>";
    }
    return oss.str();
}

static void PrintUsage() {
    std::cerr << "Usage: aitrace crash report --pc=<hex> --lr=<hex> [--sp=<hex>]\n"
              << "                        --elf=<path> [--cfsr=<hex>]\n"
              << "                        [--stack=<h1,h2,...>]\n";
}

int crash_main(int argc, char* argv[]) {
    if (argc < 2) { PrintUsage(); return 1; }

    std::string sub = argv[1];
    if (sub != "report") { PrintUsage(); return 1; }

    uint32_t pc = 0, lr = 0, sp = 0, cfsr = 0;
    std::string elf_path;
    std::vector<uint32_t> stack_vals;
    bool has_pc = false, has_lr = false, has_cfsr = false;

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.find("--pc=") == 0)      { pc = ParseHex(arg.c_str() + 5); has_pc = true; }
        else if (arg.find("--lr=") == 0) { lr = ParseHex(arg.c_str() + 5); has_lr = true; }
        else if (arg.find("--sp=") == 0) { sp = ParseHex(arg.c_str() + 5); }
        else if (arg.find("--cfsr=") == 0) { cfsr = ParseHex(arg.c_str() + 7); has_cfsr = true; }
        else if (arg.find("--elf=") == 0) { elf_path = arg.substr(6); }
        else if (arg.find("--stack=") == 0) {
            std::string vals = arg.substr(8);
            std::istringstream ss(vals);
            std::string token;
            while (std::getline(ss, token, ','))
                stack_vals.push_back(ParseHex(token.c_str()));
        }
    }

    if (!has_pc || !has_lr || elf_path.empty()) {
        std::cerr << "Error: --pc, --lr, and --elf are required.\n";
        PrintUsage();
        return 1;
    }

    ElfParser elf;
    if (!elf.Load(elf_path)) {
        MapParser map;
        if (!map.Load(elf_path)) {
            std::cerr << "Failed to load " << elf_path << " as ELF or MAP file.\n";
            return 1;
        }
        // Resolve via map symbols (limited)
        std::cout << "===== CRASH ANALYSIS REPORT =====\n\n";
        std::cout << "PC: 0x" << std::hex << std::setw(8) << std::setfill('0')
                  << pc << std::dec << std::setfill(' ') << "\n";
        std::cout << "LR: 0x" << std::hex << std::setw(8) << std::setfill('0')
                  << lr << std::dec << std::setfill(' ') << "\n\n";
        std::cout << "(Using .map — limited precision)\n\n";

        const auto& syms = map.GetSymbols();
        for (auto addr : {pc, lr}) {
            const MapSymbol* best = nullptr;
            for (const auto& s : syms)
                if (s.address <= addr && (!best || s.address > best->address)) best = &s;
            std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
                      << addr << " : ";
            if (best && (addr - best->address) < 0x8000) {
                uint32_t off = addr - best->address;
                std::cout << best->name << " + 0x" << std::hex << off;
            } else { std::cout << "<unknown>"; }
            std::cout << std::dec << std::setfill(' ') << "\n";
        }
        if (has_cfsr && cfsr) DecodeCFSR(cfsr);
        return 0;
    }

    // ELF-based analysis
    std::cout << "===== CRASH ANALYSIS REPORT =====\n\n";
    std::cout << "PC: " << ResolveAddrElf(elf, pc) << "\n";
    std::cout << "LR: " << ResolveAddrElf(elf, lr) << "\n";
    if (sp != 0) std::cout << "SP: 0x" << std::hex << std::setw(8)
                           << std::setfill('0') << sp << std::dec
                           << std::setfill(' ') << "\n";

    if (!stack_vals.empty()) {
        std::cout << "\nStack values at SP:\n";
        uint32_t addr = sp;
        for (uint32_t val : stack_vals) {
            std::cout << "  [0x" << std::hex << std::setw(8) << std::setfill('0')
                      << addr << "] = " << ResolveAddrElf(elf, val) << "\n";
            addr += 4;
        }
        std::cout << std::dec << std::setfill(' ');
    }

    if (has_cfsr && cfsr) DecodeCFSR(cfsr);

    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/src/crash_cmd.h aitrace/src/crash_cmd.cpp
git commit -m "add: aitrace crash command — auto crash analysis

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 8: Create `aitrace/src/gdb_cmd.h` and `gdb_cmd.cpp` — GDB commands

**Files:**
- Create: `E:\Project\mstudio\aitrace\src\gdb_cmd.h`
- Create: `E:\Project\mstudio\aitrace\src\gdb_cmd.cpp`

- [ ] **Step 1: Write gdb_cmd.h**

```cpp
#ifndef AITRACE_GDB_CMD_H
#define AITRACE_GDB_CMD_H
int gdb_main(int argc, char* argv[]);
#endif
```

- [ ] **Step 2: Write gdb_cmd.cpp**

Uses `gdb-multiarch -batch -x <tmpfile>` for one-shot commands. `connect` detects OpenOCD, stores ELF path in a temp state file. Other commands read the saved state.

```cpp
#include "gdb_cmd.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#define popen _popen
#define pclose _pclose
#define STATE_FILE "C:/Windows/Temp/aitrace_gdb_state.txt"
#else
#include <unistd.h>
#define STATE_FILE "/tmp/aitrace_gdb_state.txt"
#endif

static void PrintUsage() {
    std::cerr << "Usage: aitrace gdb <subcommand> [args]\n"
              << "  connect [--port 3333] --elf <path>\n"
              << "  break    <location>\n"
              << "  continue\n"
              << "  step\n"
              << "  print    <expression>\n"
              << "  bt\n"
              << "  detach\n";
}

static int g_gdb_port = 3333;

// Save/load session state to a temp file so commands share context
static void SaveState(const std::string& elf) {
    std::ofstream f(STATE_FILE);
    f << elf << "\n" << g_gdb_port << "\n";
}

static std::string LoadState() {
    std::ifstream f(STATE_FILE);
    std::string elf;
    if (std::getline(f, elf)) {
        std::string port_str;
        if (std::getline(f, port_str)) g_gdb_port = std::stoi(port_str);
    }
    return elf;
}

static void ClearState() {
    std::remove(STATE_FILE);
}

static bool DetectOpenOCD() {
#ifdef _WIN32
    WSADATA d; WSAStartup(MAKEWORD(2, 2), &d);
#endif
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) return false;
#ifdef _WIN32
    u_long mode = 1; ioctlsocket(s, FIONBIO, &mode);
#endif
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(4444);
#ifdef _WIN32
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
#else
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
#endif
    connect(s, (sockaddr*)&addr, sizeof(addr));
    fd_set wset, eset;
    FD_ZERO(&wset); FD_SET(s, &wset);
    FD_ZERO(&eset); FD_SET(s, &eset);
    timeval tv = {0, 500000};
    select((int)(s + 1), nullptr, &wset, &eset, &tv);
    bool ok = FD_ISSET(s, &wset);
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
    return ok;
}

// Write commands to a temp file and execute with gdb-multiarch -batch
static std::string RunGdbBatch(const std::string& elf,
                                const std::string& commands) {
    std::ostringstream full_cmd;
    full_cmd << "target extended-remote localhost:" << g_gdb_port << "\n"
             << commands << "\n"
             << "disconnect\n"
             << "quit\n";

#ifdef _WIN32
    char tmpbuf[MAX_PATH];
    GetTempPathA(sizeof(tmpbuf), tmpbuf);
    std::string tmpfile = std::string(tmpbuf) + "aitrace_gdb_cmds.txt";
#else
    std::string tmpfile = "/tmp/aitrace_gdb_cmds.txt";
#endif

    {
        std::ofstream f(tmpfile);
        f << full_cmd.str();
    }

    std::ostringstream run;
    run << "gdb-multiarch -batch -x \"" << tmpfile << "\" \""
        << elf << "\" 2>&1";

    std::string result;
    FILE* pipe = popen(run.str().c_str(), "r");
    if (!pipe) {
        std::cerr << "Failed to start GDB.\n";
        return "";
    }
    char buf[4096];
    while (fgets(buf, sizeof(buf), pipe)) result += buf;
    pclose(pipe);
    std::remove(tmpfile.c_str());
    return result;
}

int gdb_main(int argc, char* argv[]) {
    if (argc < 2) { PrintUsage(); return 1; }

    std::string sub = argv[1];

    if (sub == "connect") {
        std::string elf_path;
        for (int i = 2; i < argc; i++) {
            std::string arg = argv[i];
            if (arg.find("--port=") == 0) g_gdb_port = std::stoi(arg.substr(7));
            else if (arg.find("--elf=") == 0) elf_path = arg.substr(6);
        }
        if (elf_path.empty()) {
            std::cerr << "Error: --elf=<path> is required.\n";
            return 1;
        }
        if (!DetectOpenOCD()) {
            std::cerr << "Error: OpenOCD not detected on TCP 4444.\n"
                      << "Please start OpenOCD first (e.g. 'make.bat rtt').\n";
            return 1;
        }
        // Test connection
        auto result = RunGdbBatch(elf_path, "");
        if (result.find("error") != std::string::npos
            || result.find("Error") != std::string::npos) {
            std::cerr << "GDB connection test failed:\n" << result << "\n";
            return 1;
        }
        SaveState(elf_path);
        std::cout << "GDB connected. ELF: " << elf_path << "\n";
        std::cout << "Ready: break, continue, step, print, bt, detach\n";
        return 0;
    }

    // All other commands need a prior connect
    std::string elf = LoadState();
    if (elf.empty()) {
        std::cerr << "Not connected. Run 'aitrace gdb connect --elf <path>' first.\n";
        return 1;
    }

    if (sub == "break") {
        if (argc < 3) { std::cerr << "Usage: aitrace gdb break <location>\n"; return 1; }
        std::cout << RunGdbBatch(elf, "break " + std::string(argv[2]));
    } else if (sub == "continue") {
        std::cout << RunGdbBatch(elf, "continue");
    } else if (sub == "step") {
        std::cout << RunGdbBatch(elf, "step");
    } else if (sub == "print") {
        if (argc < 3) { std::cerr << "Usage: aitrace gdb print <expr>\n"; return 1; }
        std::cout << RunGdbBatch(elf, "print " + std::string(argv[2]));
    } else if (sub == "bt") {
        std::cout << RunGdbBatch(elf, "bt");
    } else if (sub == "detach") {
        ClearState();
        std::cout << "Disconnected.\n";
    } else {
        PrintUsage();
        return 1;
    }

    return 0;
}
```

> **Note:** This uses GDB batch mode (each command reconnects to the target). For a persistent MI2 session (future enhancement), we would keep a `gdb -i=mi2` process alive and communicate via named pipe. The batch approach is simpler and sufficient for AI debugging where commands come seconds apart.

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/src/gdb_cmd.h aitrace/src/gdb_cmd.cpp
git commit -m "add: aitrace gdb command — one-shot GDB via batch mode

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 9: Create `aitrace/Makefile`

**Files:**
- Create: `E:\Project\mstudio\aitrace\Makefile`

- [ ] **Step 1: Write aitrace/Makefile**

```makefile
# AITrace CLI Tool Makefile
# Links against libmstudiocore.a (built by ../core/Makefile)

MSYS64_PATH = D:/software/msys64
CXX = $(MSYS64_PATH)/mingw64/bin/clang++
CFLAGS = -Wall -Wextra -O2 -g
CXXFLAGS = -std=c++17 $(CFLAGS)

SRC_DIR = src
CORE_DIR = ../core
CORE_BUILD = $(CORE_DIR)/build
SRC_TOP = ../src
BUILD_DIR = build

INCLUDES = -I$(SRC_DIR) -I$(SRC_TOP) -I$(SRC_TOP)/utils

SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(notdir $(SRCS))))

TARGET = aitrace.exe
LIB_CORE = $(CORE_BUILD)/libmstudiocore.a
LIBS = -lws2_32

all: core $(TARGET)

core:
	@$(MAKE) -C $(CORE_DIR)

$(TARGET): $(OBJS) $(LIB_CORE)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OBJS) $(LIB_CORE) -o $@ $(LIBS)

vpath %.cpp $(SRC_DIR)

$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	@rm -rf $(BUILD_DIR)
	@rm -f $(TARGET)

clean-all: clean
	@$(MAKE) -C $(CORE_DIR) clean

.PHONY: all clean clean-all core
```

- [ ] **Step 2: Build and verify**

```bash
cd /e/Project/mstudio/aitrace
make clean-all && make
```

Expected: `aitrace.exe` is produced. Run `./aitrace.exe` to see usage.

- [ ] **Step 3: Commit**

```bash
cd /e/Project/mstudio
git add aitrace/Makefile
git commit -m "add: aitrace Makefile — builds CLI linking against libmstudiocore

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 10: Write Skill definition file

**Files:**
- Create: `E:\Project\modus_template\.claude\skills\aitrace-skill.md`

- [ ] **Step 1: Create skills directory**

```bash
mkdir -p /e/Project/modus_template/.claude/skills
```

- [ ] **Step 2: Write aitrace-skill.md**

```markdown
# AITrace Debug Skill

Use `aitrace.exe` (at `E:\Project\mstudio\aitrace\aitrace.exe`) to debug MCU firmware.

## Prerequisites

OpenOCD must be running with RTT enabled. Start from modus_template root:
```powershell
.\make.bat rtt
```

This exposes TCP 9090 (RTT Shell/Logs), TCP 9091 (Waveform), TCP 4444 (OpenOCD Telnet).

## Intrusion Levels

**Default: Passive.** Always try this first.

| Level | Method | CPU Impact | Confirmation Needed |
|-------|--------|------------|---------------------|
| A. Passive | `aitrace shell ...`, `aitrace wave ...` | None | No |
| B. Halt | `aitrace ocd halt/regs/peek/mdw/stack` | Paused briefly | **Yes** |
| C. GDB | `aitrace gdb connect/break/step` | Full debug control | **Yes** |

## Core Commands

### Passive (always safe)

```powershell
aitrace shell regs / peek <addr> / stack [n] / cfsr / list
aitrace shell log -E -W -I -D
aitrace wave capture 5                  # 5s CSV to stdout
aitrace wave capture 10 --output w.csv  # To file
aitrace wave start / stop / rate <n>
```

### Halt-Based (requires confirmation)

```powershell
aitrace ocd halt / resume / regs
aitrace ocd peek <addr> / mdw <addr> [n] / stack [n]
```

### GDB (requires confirmation)

```powershell
aitrace gdb connect --elf build/template.elf
aitrace gdb break main.c:100
aitrace gdb continue / step / bt
aitrace gdb print g_wTickCounter
aitrace gdb detach
```

### Analysis

```powershell
aitrace map resolve build/template.elf 0x08001234 0x08005678
aitrace map info build/template.map
aitrace crash report --pc=0x... --lr=0x... --sp=0x... --elf=build/template.elf
```

## Workflows

### HardFault Analysis

1. Observe RTT Ch0 output — firmware auto-dumps exception frame + CFSR on fault
2. Extract PC, LR, SP from the dump
3. `aitrace crash report --pc=<PC> --lr=<LR> --sp=<SP> --elf=build/template.elf`
4. Interpret: PC = faulting instruction, LR = caller return address, CFSR = fault type

### Runtime Behavior Analysis

1. `aitrace wave capture 5 > wave.csv` — capture waveform
2. `aitrace shell regs` + `aitrace shell list` — check MCU state
3. Analyze CSV: look for anomalies, compare with expected ranges

### Variable Inspection

- Passive: `aitrace shell peek <addr>`
- Halt: `aitrace ocd halt` → `aitrace ocd peek <addr>` → `aitrace ocd resume` (requires confirmation)
- GDB: `aitrace gdb connect --elf build/template.elf` → `aitrace gdb print <var>` (requires confirmation)

## Safety Rules

- ALWAYS prefer passive commands first — the MCU may be driving a motor/power stage
- Before halt/resume or GDB: explain WHY to the engineer, get confirmation, remind them it interrupts real-time control
- Never modify source code or re-flash firmware without confirmation
```

- [ ] **Step 3: Commit**

```bash
cd /e/Project/modus_template
git add .claude/skills/aitrace-skill.md
git commit -m "add: aitrace Claude Code skill definition

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Self-Review

1. **Spec coverage:** Each spec section maps to tasks: §3 → Tasks 1-9, §4-5 → Tasks 3-8, §6 → Task 8, §7-8 → Task 10, §9 → Tasks 1+9, §10 → incorporated.

2. **Placeholder scan:** No TBD/TODO. All code shown inline. Error handling is explicit.

3. **Type consistency:** `shell_main`/`wave_main`/`ocd_main`/`gdb_main`/`map_main`/`crash_main` signatures match `main.cpp` forward declarations. `ProtocolParser::Feed()` interface matches existing `protocol_parser.h`. `OcdClient` methods used as defined in `ocd_client.h`.

4. **No existing code modified:** `core/Makefile` compiles from original `../src/` paths. mstudio's own Makefile untouched.
