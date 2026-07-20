// ============================================================================================
// Native C++ Verilator harness for the OoO RISC-V SoC (SocHarness).
//
// WHY THIS EXISTS. ChiselSim drives Verilator through svsim and gives no way to turn on
// Verilator's multithreading (`--threads`). Driving the Verilated model directly from C++ does two
// things ChiselSim cannot: (1) build a MULTITHREADED model (~2-5x on a multicore box for a design
// this size), and (2) remove the JVM<->Verilator round-trip, so flashing a Doom-sized image costs
// seconds instead of the ChiselSim hours.
//
// It replicates what ProgramSpec / DoomSpec do: flash a hex image, run to the tohost mailbox (or a
// cycle cap), optionally capture framebuffer PPMs, and print the hardware performance counters.
//
// Usage:
//   simx <image.hex> [maxCycles] [framesDir framesN stride moveFromFrame]
//     image.hex     : one 32-bit little-endian word per line (word N == address N), as build.sh emits
//     maxCycles     : cycle budget (default 8,000,000)
//     framesDir ..  : if given, dump up to framesN PPMs every `stride` completed frames; from frame
//                     moveFromFrame the scripted keys hold FORWARD (walk) -- mirrors DoomSpec
//
// NOTE: untested first pass. Port names follow Chisel's flattened `io_<field>` convention; if the
// first build reports an unknown member, check generated obj_dir/VSocHarness.h and adjust.
// ============================================================================================
#include "VSocHarness.h"
#include "verilated.h"

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>

// framebuffer geometry must match OoOParams (fbWidth x fbHeight)
static const int FB_W = 320, FB_H = 240;
static const int FB_WORDS = FB_W * FB_H;

static VerilatedContext* ctx = nullptr;
static VSocHarness*      dut = nullptr;

// one full clock: falling then rising edge
static inline void tick() {
    dut->clock = 0; dut->eval();
    dut->clock = 1; dut->eval();
}

// read one hex-word-per-line image
static std::vector<uint32_t> load_hex(const char* path) {
    std::vector<uint32_t> words;
    std::ifstream f(path);
    if (!f) { fprintf(stderr, "[sim] cannot open %s\n", path); exit(1); }
    std::string line;
    while (std::getline(f, line)) {
        size_t a = line.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) continue;
        size_t b = line.find_last_not_of(" \t\r\n");
        words.push_back((uint32_t)strtoul(line.substr(a, b - a + 1).c_str(), nullptr, 16));
    }
    return words;
}

// scan the framebuffer capture RAM (combinational read: no clock step, does not advance the CPU)
// and write a binary PPM. Pixels are 0x0RGB (RGB444); expand each nibble to 8 bits.
static void dump_ppm(const char* path) {
    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << FB_W << " " << FB_H << "\n255\n";
    for (int i = 0; i < FB_WORDS; i++) {
        dut->io_fb_read_addr = (uint32_t)i;
        dut->eval();                                  // combinational read only
        uint32_t px = dut->io_fb_read_data;
        int r4 = (px >> 8) & 0xf, g4 = (px >> 4) & 0xf, b4 = px & 0xf;
        unsigned char rgb[3] = {
            (unsigned char)((r4 << 4) | r4),
            (unsigned char)((g4 << 4) | g4),
            (unsigned char)((b4 << 4) | b4)
        };
        out.write((char*)rgb, 3);
    }
    fprintf(stderr, "[sim] wrote %s\n", path);
}

int main(int argc, char** argv) {
    ctx = new VerilatedContext;
    ctx->commandArgs(argc, argv);
    dut = new VSocHarness{ctx};

    if (argc < 2) {
        fprintf(stderr, "usage: %s <image.hex> [maxCycles] [framesDir framesN stride moveFrom]\n", argv[0]);
        return 1;
    }
    const char* hexPath   = argv[1];
    uint64_t    maxCycles = (argc >= 3) ? strtoull(argv[2], nullptr, 10) : 8000000ULL;
    const char* framesDir = (argc >= 4) ? argv[3] : nullptr;
    int         framesN   = (argc >= 5) ? atoi(argv[4]) : 0;
    int         stride    = (argc >= 6) ? atoi(argv[5]) : 1;
    long        moveFrom  = (argc >= 7) ? atol(argv[6]) : 1000000000L;

    // drive all inputs to known values
    dut->io_execute       = 0;
    dut->io_flash         = 0;
    dut->io_flash_address = 0;
    dut->io_flash_value   = 0;
    dut->io_btns          = 0;
    dut->io_key_bits      = 0;
    dut->io_fb_read_addr  = 0;
    dut->io_pc_range_reset = 0;

    // reset
    dut->reset = 1;
    for (int i = 0; i < 5; i++) tick();
    dut->reset = 0;

    // flash the image through the flash port (native speed -- no round-trip)
    std::vector<uint32_t> img = load_hex(hexPath);
    fprintf(stderr, "[sim] flashing %zu words from %s ...\n", img.size(), hexPath);
    dut->io_execute = 0;
    dut->io_flash   = 1;
    for (size_t i = 0; i < img.size(); i++) {
        dut->io_flash_address = (uint32_t)i;
        dut->io_flash_value   = img[i];
        tick();
    }
    dut->io_flash = 0;

    // run
    fprintf(stderr, "[sim] running up to %llu cycles ...\n", (unsigned long long)maxCycles);
    dut->io_execute = 1;

    const uint32_t ENTER   = 1u << 8;   // HID 0x28 = Enter  (matches DoomSpec)
    const uint32_t FORWARD = 1u << 26;  // HID 0x1A = W = forward
    uint32_t lastKey = 0xFFFFFFFFu;
    int      framesDumped = 0;
    uint64_t lastFrameCount = 0;

    for (uint64_t c = 0; c < maxCycles; c++) {
        tick();
        uint64_t fbw     = dut->io_fb_writes;
        long     frameNo = (long)(fbw / FB_WORDS);

        // scripted input: title -> menu -> New Game -> Episode -> Skill -> in-game, then walk
        uint32_t want = 0;
        if      (frameNo >= 3  && frameNo < 6)  want = ENTER;
        else if (frameNo >= 10 && frameNo < 13) want = ENTER;
        else if (frameNo >= 17 && frameNo < 20) want = ENTER;
        else if (frameNo >= 24 && frameNo < 27) want = ENTER;
        else if (frameNo >= moveFrom)           want = FORWARD;
        if (want != lastKey) { dut->io_key_bits = want; lastKey = want; }

        // capture a sequence of frames
        if (framesDir && framesDumped < framesN && (fbw / FB_WORDS) >= lastFrameCount + (uint64_t)stride) {
            char path[600];
            snprintf(path, sizeof(path), "%s/frame_%04d.ppm", framesDir, framesDumped);
            dump_ppm(path);
            framesDumped++;
            lastFrameCount = fbw / FB_WORDS;
        }

        if (dut->io_tohost_seen) {
            fprintf(stderr, "[sim] tohost=0x%08x @ cycle %llu\n",
                    (unsigned)dut->io_tohost_data, (unsigned long long)c);
            break;
        }
    }

    // final stats + frame
    fprintf(stderr, "[sim] cycles=%llu commits=%llu redirects=%llu divbusy=%llu headstall=%llu fbwrites=%llu\n",
            (unsigned long long)dut->io_perf_cycles,
            (unsigned long long)dut->io_perf_commits,
            (unsigned long long)dut->io_perf_redirects,
            (unsigned long long)dut->io_perf_divbusy,
            (unsigned long long)dut->io_perf_headstall,
            (unsigned long long)dut->io_fb_writes);
    if ((unsigned long long)dut->io_perf_cycles > 0) {
        double ipc = (double)dut->io_perf_commits / (double)dut->io_perf_cycles;
        fprintf(stderr, "[sim] IPC=%.3f\n", ipc);
    }
    if (framesDir) {
        char path[600];
        snprintf(path, sizeof(path), "%s/final.ppm", framesDir);
        dump_ppm(path);
    }

    dut->final();
    delete dut;
    delete ctx;
    return 0;
}
