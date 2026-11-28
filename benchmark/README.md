# WarpSpeed Performance Benchmark

I've designed different kernels that measure how fast the WarpSpeed SM runs real work, where the cycles go when it does not, and what it costs in area and power on the FPGA. Each kernel is loaded into a modelled DDR, run at 1, 2, 4 and 8 warps, checked against a reference, and its counters printed.

## Summary

| | |
|---|---|
| Device | Xilinx Artix-7 XC7A100T-1 (Nexys A7), Vivado 2025.2, post-route |
| Clock | 100 MHz (WNS +0.014 ns, WHS +0.035 ns) |
| Peak throughput | 800 M thread-ops/s (8 lanes × 100 MHz) · 1.6 GFLOP/s FP32 (FMA = 2 FLOPs) |
| SM area | 48,924 LUTs (77% of device) · 39,618 FFs (31%) · 24 BRAM36 (18%) · 56 DSP48 (23%) |
| Whole design | 52,276 LUTs (82%), 99.9% of slices occupied |
| Power (vectorless estimate, low confidence) | 1.383 W chip total: **SM 0.273 W**, MIG 0.891 W, MIG reference clock 0.107 W, static 0.110 W |

## Method

Performance was measured in simulation with Verilator, using 5 random seeds (`+verilator+seed+1` to `+5`). A behavioural MIG with random latency was used to model the DDR main memory. Every run checks the final registers, the DDR contents, and the expected instruction and memory counts; a failed check stops the simulation. Ranges are given where the seeds differ by more than about 1%.

| Metric | Meaning |
|---|---|
| Thread-ops/cycle | Instructions completed per thread, per cycle. Peak is 8 (8 lanes). |
| SIMD efficiency | Active threads ÷ (issued instructions × 32). Lower means idle lanes from divergence or partial warps. |
| Coalescing ratio | Cache transactions per memory instruction. 1.0 is perfect; 32 is the worst case. |
| D$ blocked | Cycles the D$ is busy with a miss and cannot accept another request. |
| Load latency | Cycles from a load leaving the coalescer to its writeback, split into queueing and service. |

**Lost-cycle breakdown.** Every cycle is assigned to exactly one row, in this order, and the rows sum to 100% (checked by an assertion):

| Row | The cycle is counted here when… |
|---|---|
| Writeback stalls | the writeback buffer is full and the front end is frozen |
| Issue | a sub-warp with at least one active thread is issued |
| Empty sub-warps | an issue slot is used by a sub-warp with no active threads (sub-warp 0 always issues, and every instruction takes at least 2 issue cycles) |
| Flush | all warps are done and the D$ is being written back |
| Memory stalls | every running warp has an instruction in flight, and all of those are memory instructions |
| Execute stalls | every running warp has an instruction in flight, and at least one is not a memory instruction |
| Fetch stalls | a warp is free but the fetch buffer has no instruction ready |
| Other | a warp is free and an instruction is ready, yet nothing issued (not yet diagnosed) |

## Microbenchmarks

| Kernel | What it does |
|---|---|
| Arithmetic stream | 200 independent `ADD`s |
| Divergence cost | A 50-iteration loop around an if/else, with 0–4 of the 4 sub-warps taking the branch |
| Multi-warp memory | Loads and stores behind a divergent branch, with a strided access pattern |
| Full system | Integration Test 10: three levels of nested divergence, INT/MUL/FMA, loads and stores, and a forced line replacement |

### Warp scaling (thread-ops/cycle, seed 1)

| Kernel | 1 warp | 2 warps | 4 warps | 8 warps |
|---|---|---|---|---|
| Arithmetic stream | 2.55 | 5.12 | 5.05 | 6.73 |
| Multi-warp memory | 0.63 | 0.92 | 1.03 | 1.06 |
| Full system | 0.50 | 0.81 | 0.95 | 0.99 |

**One warp cannot keep the SM busy.** A warp has at most one instruction in flight, so with a single warp the SM waits for every result. In the arithmetic stream, 32% of cycles are execute stalls at 1 warp, and 4.6% at 8 warps.

**The 4-warp dip comes from the scheduler.** It always picks the lowest-numbered ready warp. Three warps are enough to fill the issue slots, so a fourth runs mostly alone after the others finish.

**The memory kernels stop scaling** at about 1 thread-op/cycle because of the blocking D$.

### Divergence (8 warps)

| Sub-warps taking the branch | Cycles | Thread-ops/cycle | SIMD efficiency | Empty sub-warps | Other |
|---|---|---|---|---|---|
| 0 of 4 (no divergence) | 15425 | 6.65 | 100% | 0.2% | 3.9% |
| 1 of 4 | 20714 | 4.80 | 64.6% | 7.9% | 19.8% |
| 2 of 4 | 20282 | 4.75 | 62.6% | 8.0% | 19.4% |
| 3 of 4 | 21023 | 4.43 | 60.5% | 17.3% | 15.9% |
| 4 of 4 (no divergence) | 13329 | 6.74 | 100% | 0.2% | 3.8% |

A divergent loop takes 31–36% longer than a uniform one, because both paths run one after the other. Most of the remaining loss is in two rows:
- **Empty sub-warps:** every instruction issues sub-warp 0 and takes at least 2 cycles, even when only one sub-warp has active threads.
- **Other:** this is still undiagnosed.

The 4-of-4 case is faster than 0-of-4 only because the taken path is one instruction shorter.

## Memory behaviour

| Kernel (8 warps) | Coalescing | D$ hit rate | D$ blocked | Load latency (avg) | DDR bandwidth | Bus busy |
|---|---|---|---|---|---|---|
| Multi-warp memory | 2.40 | 0.67 | 48.5% | 259 cycles | 327 MB/s | 20.4% |
| Full system | 1.00 | 0.46 | 65.4% | 219 cycles | 345 MB/s | 21.6% |
| SAXPY | 1.00 | 0.33 | 76.9% | 266 cycles | 332 MB/s | 20.7% |
| Matrix-vector | 1.00 | 0.49 | 77.5% | 149 cycles | 256 MB/s | 16.0% |

**The D$ is the bottleneck, not DRAM bandwidth.** The D$ is blocking: it serves one miss at a time and refuses other requests until the line arrives. All memory-heavy kernels top out at 330–345 MB/s, about 25% of the 1.3 GB/s DDR2 peak, while the memory bus is idle about 80% of the time.

**Latency grows with warps because of queueing.** At 8 warps a load takes 150–270 cycles, most of it waiting behind other warps' misses (224 of 259 for multi-warp memory); a single fill is about 35.

**Conflict misses in Full system.** All six of a warp's lines map to the same D$ set, so they compete for 4 ways, causing 3 evictions per warp. It is the only kernel with conflict misses: every miss in the real kernels is a first touch, because the data they reuse (such as `x` in matrix-vector) fits in the cache and everything else is used once.

## Real kernels

### SAXPY: `y = ax + y`, 4096 FP32 elements

Grid-stride loop: element `= step × 256 + warp × 32 + thread`, so each warp instruction touches exactly one 128-byte line. Weak scaling: 512 elements per warp.

| Warps | Thread-ops/cycle | DDR bandwidth | D$ blocked | Load latency |
|---|---|---|---|---|
| 1 | 1.13–1.15 | 161–165 MB/s | 0% | 62 cycles |
| 2 | 1.90 | 269 MB/s | 27% | 78 cycles |
| 4 | 2.31 | 326 MB/s | 68% | 129 cycles |
| 8 | 2.34–2.37 | 329–334 MB/s | 77% | 266 cycles |

SAXPY has no reuse: every element is loaded once and stored once (45 bytes of data transferred between the SM and DDR per instruction). At 4 warps, the D$ becomes busy most of the time, so adding warps beyond that only lengthens the queue.

### Matrix-vector: `y = A·x`, A is 256 × 64 FP32 (column-major)

One thread per row. Each step loads one column element of `A` (one line per warp) and the shared `x[c]` (every warp uses it).

| Warps | Thread-ops/cycle | DDR bandwidth | D$ hit rate | D$ blocked |
|---|---|---|---|---|
| 1 | 1.46–1.49 | 88–90 MB/s | 0.48 | 0% |
| 2 | 2.48 | 146 MB/s | 0.49 | 26% |
| 4 | 3.12 | 182 MB/s | 0.49 | 53% |
| 8 | 4.42–4.52 | 256–262 MB/s | 0.49 | 78% |

Half the loads hit, because `x` stays in the cache. That cuts traffic to 18.5 bytes per instruction, so matrix-vector keeps scaling to 8 warps where SAXPY stops at 4.

### Mandelbrot: 40 × 6 image, Q12 fixed point, at most 32 steps per pixel

Each pixel iterates `z = z² + c` until `|z| > 2` or 32 steps, using integer `MUL` and `SRAI` (Q12: values × 4096). The image is split into 12 chunks of 32 pixels (2 per row; the right chunk of each row has only 8 real pixels). Warp `w` draws chunks `w, w + n, …` for `n` warps, so every warp count processes the full image. The output rows are padded to 64 words so that each chunk stores exactly one cache line.

| Warps | Cycles | Speedup | Thread-ops/cycle | Execute stalls | Empty sub-warps | SIMD efficiency |
|---|---|---|---|---|---|---|
| 1 | 39734–39764 | 1.00× | 1.21 | 39% | 7.5% | 38.6% |
| 2 | 27969–28034 | 1.42× | 1.73 | 26% | 10.7% | 38.7% |
| 4 | 18113–18317 | 2.20× | 2.68–2.71 | 10% | 16.6% | 39.0% |
| 8 | 16586–16738 | 2.38× | 3.00–3.03 | 7% | 18.2% | 39.5% |

Memory does not matter here: 12 stores in total and 20 MB/s. Two things limit it:

- **Divergence.** Neighbouring pixels escape after very different numbers of steps, and a warp keeps looping until its slowest thread finishes. Together with the 24 idle lanes in every 8-pixel chunk, only 39.5% of lanes do useful work.
- **Load imbalance and latency.** The step loop is one long dependent chain (`MUL → SRAI → ADD → BLT`), so 1 warp stalls 39% of the time. More warps hide that (7% at 8 warps). Going from 4 to 8 warps gains only 1.08×: latency is already mostly hidden, and 12 chunks over 8 warps means warps 0–3 do two chunks while warps 4–7 do one.

The three real kernels are each limited by a different part of the design:

| Kernel | Thread-ops/cycle (8 warps) | Limited by |
|---|---|---|
| SAXPY | 2.34–2.37 | The blocking D$: saturates at 4 warps |
| Matrix-vector | 4.42–4.52 | D$ misses, but reuse of `x` keeps it scaling to 8 warps |
| Mandelbrot | 3.00–3.03 | Divergence: 39.5% SIMD efficiency |

Pure arithmetic reaches 6.7 of 8 thread-ops/cycle at 8 warps. Every memory-heavy kernel plateaus at about 330–345 MB/s, a quarter of the DDR2 peak, because the D$ handles one miss at a time.

## Real units

Energy per op = power ÷ ops per second.

| Kernel (8 warps) | Thread-ops/cycle | M thread-ops/s | Runtime | DDR bandwidth | Energy/op (SM) | Energy/op (chip) |
|---|---|---|---|---|---|---|
| Peak | 8.00 | 800 | – | – | 341 pJ | 1.73 nJ |
| Arithmetic stream | 6.68–6.73 | 670 | 77 µs | – | 407 pJ | 2.06 nJ |
| Matrix-vector | 4.42–4.52 | 447 | 260–265 µs | 256–262 MB/s | 611 pJ | 3.09 nJ |
| Mandelbrot | 3.00–3.03 | 301 | 166–167 µs | 20 MB/s | 907 pJ | 4.59 nJ |
| SAXPY | 2.34–2.37 | 236 | 148–150 µs | 329–334 MB/s | 1.16 nJ | 5.86 nJ |

Floating-point rate: matrix-vector reaches **125 MFLOP/s** (7.8% of the FP32 peak), and SAXPY **55 MFLOP/s** (3.4%). Most of their instructions are loads, address arithmetic and loop control, not FMAs.

## Area and power breakdown

| Block | LUTs | % of SM LUTs | FFs | DSP | Power |
|---|---|---|---|---|---|
| Coalescing unit | 12,412 | 25% | 19,737 | | 0.049 W |
| Recon stacks (8) | 11,479 | 23% | 6,224 | | 0.040 W |
| Integer ALUs (8) | 4,840 | 10% | 893 | 24 | 0.013 W |
| FMAs (8) | 4,538 | 9% | 2,061 | 32 | 0.032 W |
| I$ | 3,437 | 7% | 1,667 | | 0.007 W |
| Writeback arbiters (8) | 3,287 | 7% | 904 | | 0.023 W |
| D$ | 3,150 | 6% | 2,700 | | 0.008 W |
| LSU | 3,098 | 6% | 2,043 | | 0.005 W |
| Register files (8) | – | – | – | 24 BRAM36 | 0.063 W |
| Commit tracker, fetch, scheduler, other | ~2,700 | 6% | ~3,400 | | ~0.033 W |

**Control costs more than compute.**
- **Control** (coalescing unit + recon stacks): **49%** of the SM's LUTs and more than half its flip-flops.
- **Compute** (FMAs + integer ALUs): **19%** of the LUTs.

Two blocks explain most of the control cost:
- **Coalescing unit:** it buffers a full memory instruction for each of the 8 warps (32 addresses and 32 store values each, 16,384 flip-flops).
- **Recon stacks:** each is 8 entries × 96 bits held in flip-flops, plus the multiplexers to read and write them.

Most of the chip's power is the DDR2 interface: the MIG (including its I/O) and its reference clock account for 72% of the 1.383 W.

## Limitations

- **Blocking D$:** one outstanding miss caps memory-heavy kernels at about 25% of DDR2 bandwidth. A non-blocking D$ would need changes to the D$, the LSU and `mem_arbiter`.
- **One instruction in flight per warp:** at least 4 warps are needed to hide execution latency.
- **Lowest-ID scheduling:** this causes the 4-warp dip in throughput.
- **Issue overhead with few active sub-warps:** sub-warp 0 always issues (even if it has no active threads), and every instruction takes at least 2 cycles.
- **The Other row is undiagnosed:** up to 20% of cycles in divergent kernels.
- **The kernels are small** (300 to 40,000 cycles), so start-up and flush are a visible part of the short ones.
- **The design is close to full:** 99.9% of slices are occupied and the timing slack is +0.014 ns.
- **Not validated against the real MIG IP or hardware:** absolute latency and bandwidth may differ. However, the blocking-D$ bottleneck does not depend on DDR timing.

## Running it

From the repository root (Verilator 5.x):

```bash
verilator --binary --sv --timing --Wall --Wno-fatal \
    -Wno-ASCRANGE -Wno-PINCONNECTEMPTY -Wno-EOFNEWLINE \
    -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
    --top-module benchmark \
    $(find src -type f -name "*.sv") benchmark/benchmark.sv \
    && ./obj_dir/Vbenchmark +verilator+seed+1
```

Change the seed to repeat the run with different memory latencies.
