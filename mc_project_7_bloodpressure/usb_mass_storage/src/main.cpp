#include <Arduino.h>
#include "SPI.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"

#include "main.h"

// Library object for the flash configuration
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
// file system object from SdFat
FatVolume fatfs;
FatFile root;
FatFile file;
// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
// Check if flash is formatted
bool fs_formatted = false;
// Set to true when PC write to flash
bool fs_changed = true;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  flash.begin();
  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size() / 512, 512);
  // MSC is ready for read/write
  usb_msc.setUnitReady(true);
  usb_msc.begin();
  // Init file system on the flash
  fs_formatted = fatfs.begin(&flash);
  Serial.begin(9600);
  while (!Serial)
    delay(10); // wait for native usb
  Serial.println("Adafruit TinyUSB Mass Storage External Flash example");
  Serial.print("JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: ");
  Serial.print(flash.size() / 1024);
  Serial.println(" KB");
}

void loop()
{
  // Do nothing in the loop -> uC is not busy with delay() and stuff
}

// Callback invoked when received READ10 command.
// Copy disk’s data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize)
{
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don’t need to cache it, yahhhh !!
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk’s storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize)
{
  digitalWrite(LED_BUILTIN, HIGH);
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don’t need to cache it, yahhhh !!
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb(void)
{
  // sync with flash
  flash.syncBlocks();
  // clear file system’s cache to force refresh
  fatfs.cacheClear();
  fs_changed = true;
  digitalWrite(LED_BUILTIN, LOW);
}