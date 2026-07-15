#ifndef __PORT_NVS_H__
#define __PORT_NVS_H__

#include <stdint.h>
#include <stdbool.h>

void port_nvs_init(void);
bool port_nvs_read(uint8_t *dest, uint32_t size);
bool port_nvs_write(const uint8_t *source, uint32_t size);

#endif /* __PORT_NVS_H__ */
