#include <Arduino.h>

int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize);
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
void msc_flush_cb(void);