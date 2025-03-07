#ifndef CORE_DUMP_UTILS_H
#define CORE_DUMP_UTILS_H

#include <Arduino.h>
#include <esp_core_dump.h>
#include <esp_log.h>
#include <esp_partition.h>

typedef struct {
    uint32_t core_dumped;
    uint32_t pc;        // Program Counter
    uint32_t ps;        // Processor State
    uint32_t a[16];     // A0-A15 registers
    uint32_t sar;       // Shift Amount Register
    uint32_t exccause;  // Exception Cause
    uint32_t excvaddr;  // Exception Virtual Address
    uint32_t lbeg;      // Loop Begin
    uint32_t lend;      // Loop End
    uint32_t lcount;    // Loop Count
} core_dump_regs_t;

inline esp_err_t parse_core_dump_simple(core_dump_regs_t* regs) {
    size_t dump_addr;
    size_t dump_size;
    regs->core_dumped = 0;

    // Check if a core dump exists
    if (esp_core_dump_image_get(&dump_addr, &dump_size) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t buf[512];
    size_t offset = 0;
    const uint32_t deadbeef_marker = 0xdeadbeef;

    const esp_partition_t* coredump_part = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);

    bool found = false;
    while (found == false && offset < dump_size) {
        size_t read_size = min(sizeof(buf), dump_size - offset);
        if (esp_partition_read(coredump_part, offset, buf, read_size) != ESP_OK) {
            break;
        }
        // Find deadbeef marker
        for (size_t i = 0; i < read_size - 4; i++) {
            if (*(uint32_t*)(buf + i) == deadbeef_marker) {
                if (i + 4 + 72 <= read_size) {
                    const uint8_t* reg_data = buf + i + 4;
                    regs->pc = *(uint32_t*)(reg_data);
                    regs->ps = *(uint32_t*)(reg_data + 4);
                    for (int j = 0; j < 16; j++) {  // a0-a15 (64バイト)
                        regs->a[j] = *(uint32_t*)(reg_data + 8 + j * 4);
                    }
                    regs->sar = *(uint32_t*)(reg_data + 72);
                    regs->exccause = *(uint32_t*)(reg_data + 76);
                    regs->excvaddr = *(uint32_t*)(reg_data + 80);
                    regs->lbeg = *(uint32_t*)(reg_data + 84);
                    regs->lend = *(uint32_t*)(reg_data + 88);
                    regs->lcount = *(uint32_t*)(reg_data + 92);
                    regs->core_dumped = 1;
                    found = true;
                } else {
                    offset = offset + i;  // adjust to deadbeef
                    break;
                }
            }
        }
        offset += read_size;
    }
    // Erase the core dump after processing
    esp_core_dump_image_erase();
    return ESP_OK;
}

#endif  // CORE_DUMP_UTILS_H
