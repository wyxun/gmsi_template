#include "port_nvs.h"
#include "at32f403a_407.h"
#include "perf_counter.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define NVS_SECTOR_ADDR   0x080FF800
#define NVS_MAX_SIZE      2048

static void nvs_log(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        SEGGER_RTT_Write(0, buf, len);
    }
}

void port_nvs_init(void)
{
    /* Internal flash controller requires no special clock/GPIO config. */
}

bool port_nvs_read(uint8_t *dest, uint32_t size)
{
    nvs_log("[NVS] read size=%lu, first_byte=0x%02X\r\n", (unsigned long)size, *(volatile uint8_t *)NVS_SECTOR_ADDR);
    if (size > NVS_MAX_SIZE) {
        return false;
    }
    memcpy(dest, (const void *)NVS_SECTOR_ADDR, size);
    return true;
}

bool port_nvs_write(const uint8_t *source, uint32_t size)
{
    nvs_log("[NVS] write size=%lu, first_byte=0x%02X\r\n", (unsigned long)size, source[0]);
    if (size > NVS_MAX_SIZE) {
        return false;
    }

    flash_status_type status = FLASH_OPERATE_DONE;

    /* wait for any busy state on internal flash */
    while (flash_flag_get(FLASH_OBF_FLAG));

    /* unlock internal flash controllers */
    flash_unlock();

    /* erase the sector */
    status = flash_sector_erase(NVS_SECTOR_ADDR);
    nvs_log("[NVS] erase status=%d\r\n", (int)status);
    if (status != FLASH_OPERATE_DONE) {
        flash_lock();
        return false;
    }

    /* program the sector word by word */
    uint32_t num_words = (size + 3) / 4;
    const uint32_t *src_words = (const uint32_t *)source;
    for (uint32_t i = 0; i < num_words; i++) {
        status = flash_word_program(NVS_SECTOR_ADDR + i * 4, src_words[i]);
        if (status != FLASH_OPERATE_DONE) {
            nvs_log("[NVS] write failed at word %lu, status=%d\r\n", (unsigned long)i, (int)status);
            flash_lock();
            return false;
        }
    }

    /* lock internal flash controllers */
    flash_lock();

    /* read back verify */
    uint32_t verify_val = *(volatile uint32_t *)NVS_SECTOR_ADDR;
    nvs_log("[NVS] write success! immediately read back first word: 0x%08X (expected: 0x%08X)\r\n", 
            (unsigned long)verify_val, (unsigned long)src_words[0]);
    return true;
}
