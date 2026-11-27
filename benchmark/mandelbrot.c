// Mandelbrot in fixed point (every value is stored as value x 4096)
// Image: width 40, height 6. Build and run:  cc -O2 -o mandelbrot mandelbrot.c && ./mandelbrot
#include <stdio.h>

// One pixel: how many steps until it escapes (|z| > 2), at most 32
int pixel(int px, int py) {
    // pixel -> point c
    int cx = -8192 + px * 307;      // x from -2.0 in steps of 0.075 (307.2 rounded)
    int cy = -4096 + py * 1365;     // y from -1.0 in steps of 0.333 (1365.3 rounded)

    int zx = 0, zy = 0, steps = 0;

    while (steps < 32) {
        int zx2 = (zx * zx) >> 12;  // 1. square
        int zy2 = (zy * zy) >> 12;

        if (zx2 + zy2 > 16384)      // 2. check: |z|^2 > 4.0
            break;

        int zxzy = (zx * zy) >> 12; // 3. update (zy first: it needs the old zx)
        zy = zxzy + zxzy + cy;
        zx = zx2 - zy2 + cx;

        steps = steps + 1;          // 4. count
    }
    return steps;
}

int main(void) {
    // Reference: the plain way, row by row
    int ref[6][40];

    for (int py = 0; py < 6; py++) {
        for (int px = 0; px < 40; px++) {
            ref[py][px] = pixel(px, py);

            // '@' = never escaped (inside the set), '.' = escaped
            putchar(ref[py][px] == 32 ? '@' : '.');
        }
        putchar('\n');
    }
    putchar('\n');

    // The GPU way, once per warp count: every count must draw the same full image
    int warp_counts[4] = {1, 2, 4, 8};

    for (int run = 0; run < 4; run++) {
        int num_warps = warp_counts[run];               // the testbench presets this in R30

        // Output rows padded to 64 words, -1 = never written
        int out[6 * 64];
        for (int i = 0; i < 6 * 64; i++)
            out[i] = -1;

        for (int w = 0; w < num_warps; ++w) {               // each warp
            for (int t = 0; t < 32; ++t) {                  // each thread of that warp
                // this thread's own loop: chunk w, w + num_warps, w + 2 * num_warps, ... while < 12
                // (8 warps: warps 0-3 go around twice, warps 4-7 once. 1 warp: warp 0 does all 12)
                int chunk = w;
                do {
                    int py = chunk >> 1;                    // 2 chunks per row
                    int px = (chunk & 1) * 32 + t;          // left or right half of the row

                    if (px < 40) {                          // threads past the edge do nothing
                        // pixel -> point c
                        int cx = -8192 + px * 307;          // x from -2.0 in steps of 0.075 (307.2 rounded)
                        int cy = -4096 + py * 1365;         // y from -1.0 in steps of 0.333 (1365.3 rounded)

                        int zx = 0, zy = 0, steps = 0;

                        do {
                            int zx2 = (zx * zx) >> 12;      // 1. square
                            int zy2 = (zy * zy) >> 12;

                            if (zx2 + zy2 > 16384)          // 2. check: |z|^2 > 4.0
                                break;

                            int zxzy = (zx * zy) >> 12;     // 3. update (zy first: it needs the old zx)
                            zy = zxzy + zxzy + cy;
                            zx = zx2 - zy2 + cx;

                            steps++;                        // 4. count
                        } while (steps < 32);

                        out[py * 64 + px] = steps;
                    }

                    chunk += num_warps;
                } while (chunk < 12);
            }
        }

        // Check: same counts as the reference, and the padding (px 40-63) never written
        int errors = 0;
        for (int py = 0; py < 6; py++) {
            for (int px = 0; px < 64; px++) {
                int expected = (px < 40) ? ref[py][px] : -1;
                if (out[py * 64 + px] != expected)
                    errors++;
            }
        }
        printf("%d warps: %s (%d mismatches)\n", num_warps, errors == 0 ? "MATCH" : "MISMATCH", errors);
    }

    return 0;
}

/*
    0x00 MOV_WID R1                     // R1 = warp
    0x04 MOV_TID R2                     // R2 = thread
    0x08 ADD R3, R1, R0                 // R3 = chunk
    
    // constants
    0x0C ADDI R4, R0, 12                // R4 = 12
    0x10 ADDI R5, R0, 40                // R5 = 40
    0x14 ADDI R6, R0, 307               // R6 = 307
    0x18 ADDI R7, R0, 1365              // R7 = 1365
    0x1C ADDI R8, R0, 16384             // R8 = 16384
    0x20 ADDI R9, R0, 32                // R9 = 32

loop:
    0x24 ANDI R10, R3, 1
    0x28 SLLI R10, R10, 5
    0x2C ADD R10, R10, R2               // R10 = px
    0x30 SRAI R11, R3, 1                // R11 = py

    0x34 BGE R10, R5, skip              // target_offset = 0x6C, recon_offset = 0x6C
    0x38 MUL R12, R10, R6            
    0x3C SUBI R12, R12, 8192            // R12 = cx
    0x40 MUL R13, R11, R7
    0x44 SUBI R13, R13, 4096            // R13 = cy

    0x48 ADDI R14, R0, 0                // R14 = zx = 0
    0x4C ADDI R15, R0, 0                // R15 = zy = 0
    0x50 ADDI R16, R0, 0                // R16 = steps = 0

continue_steps:
    0x54 MUL R17, R14, R14
    0x58 SRAI R17, R17, 12              // R17 = zx^2
    0x5C MUL R18, R15, R15
    0x60 SRAI R18, R18, 12              // R18 = zy^2
    0x64 ADD R19, R17, R18              // R19 = zx^2 + zy^2
    0x68 BLT R8, R19, break             // target_offset = 0x24, recon_offset = 0x24

    0x6C MUL R20, R14, R15
    0x70 SRAI R20, R20, 12              // R20 = zx * zy
    0x74 SUB R21, R17, R18              // R21 = zx^2 - zy^2
    0x78 ADD R14, R21, R12              // update zx
    0x7C ADD R22, R20, R20              // R22 = (zx * zy) + (zx * zy)
    0x80 ADD R15, R22, R13
    0x84 ADDI R16, R16, 1               // steps++
    0x88 BLT R16, R9, continue_steps    // target_offset = -0x34 (0xCC), recon_offset = 0x04

break:
    // out[py * 64 + px] = steps;
    0x8C SLLI R23, R11, 6
    0x90 ADD R23, R23, R10          // R23 = py * 64 + px -> element
    0x94 SLLI R23, R23, 2           // R23 = 4 * (py * 64 + px) -> offset
    0x98 ADD R23, R31, R23          // R23 = address in output
    0x9C STORE R16, 0(R23)

skip:
    // chunk += number of warps (preset in R30)
    0xA0 ADD R3, R3, R30
    0xA4 BLT R3, R4, loop          // target_offset = -0x80 (0x80), recon_offset = 0x04

    0xA8 DONE
*/
