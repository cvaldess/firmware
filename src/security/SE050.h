#pragma once

// NXP SE050 secure element over T=1oI2C (UM11225).
//
// Layer 1 only for now: block framing (NAD PCB LEN INF CRC), the CRC-16 variant
// the SE050 uses, and the interface reset that returns the ATR. APDUs, SCP03 and
// the X25519 identity come on top of this and are ported separately.
//
// The protocol here is a port of a driver already validated on silicon against
// an SE050E2 (applet 7.2.0); only the transport differs - ESP-IDF i2c_master
// there, Arduino Wire here.

#include "configuration.h"

#if defined(HAS_SE050)

#include <Wire.h>
#include <stddef.h>
#include <stdint.h>

class SE050
{
  public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x48;

    // Wire must already be begun by the caller - the I2C bus is shared, so this
    // class never configures or owns it.
    SE050(TwoWire &bus, uint8_t address = DEFAULT_ADDRESS) : bus(bus), address(address) {}

    // Interface reset (S-block). The SE050 answers with its ATR, which also
    // resynchronises the block layer. Returns true and fills atrOut/atrLen on
    // success. Safe to call repeatedly.
    bool reset(uint8_t *atrOut = nullptr, size_t atrCap = 0, size_t *atrLen = nullptr);

    // Reset, then SELECT the IoT applet. Every APDU exchange needs the applet
    // selected first. Returns true on SW=9000.
    bool open();

    // Sends one APDU wrapped in an I-block and returns the R-APDU (response INF,
    // trailing SW included). Handles WTX. Returns the R-APDU length, or -1.
    int transceive(const uint8_t *apdu, size_t apduLen, uint8_t *resp, size_t respCap);

    // Trailing status word of an R-APDU.
    static uint16_t statusWord(const uint8_t *resp, int len);

    // Layer-1/2 bring-up check: open, read the applet version, pull random bytes.
    bool probe();

  private:
    // Writes one block and reads the answer. Returns the total framed length
    // (3 + LEN + 2), or 0 if nothing valid came back.
    size_t xfer(const uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxCap);

    bool selectApplet();

    static uint16_t crc(const uint8_t *data, size_t len);

    TwoWire &bus;
    uint8_t address;
    uint8_t seq = 0; // host N(S), toggled per I-block, reset by the interface reset
};

#endif // HAS_SE050
