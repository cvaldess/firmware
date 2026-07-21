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

    // Opens a PlatformSCP03 secure channel on top of an already-open applet:
    // INITIALIZE UPDATE, derive the session keys, verify the card cryptogram,
    // then EXTERNAL AUTHENTICATE. The SE050 refuses key agreement outside an
    // authenticated channel, so this is a prerequisite, not a hardening step.
    //
    // Verifying the card cryptogram also authenticates the chip to us: it can
    // only be reproduced with the right static keys and KDF.
    bool openSecureChannel();

    // Ensures this node has an X25519 identity inside the SE050, generating it on
    // first use and reusing it afterwards. The private half is created in the chip
    // and never leaves it. Returns the public key, little-endian as the rest of
    // Meshtastic expects. Requires an open secure channel.
    bool identityEnsure(uint8_t publicKey[32]);

    // One key agreement against the on-chip identity. Peer key and output are
    // little-endian; the SE050 works big-endian, so both are reversed here.
    bool identityEcdh(const uint8_t peerPublic[32], uint8_t shared[32]);

    // Bring-up check for all four layers.
    bool probe();

  private:
    // Writes one block and reads the answer. Returns the total framed length
    // (3 + LEN + 2), or 0 if nothing valid came back.
    size_t xfer(const uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxCap);

    bool selectApplet();

    static uint16_t crc(const uint8_t *data, size_t len);

    // SCP03 session state. mcv is the MAC chaining value: zero until the first
    // C-MAC, then the full CMAC of the previous command.
    struct Scp03 {
        uint8_t senc[16];  // command data encryption
        uint8_t smac[16];  // command MAC
        uint8_t srmac[16]; // response MAC
        uint8_t mcv[16];
        uint32_t counter; // command counter, drives the encryption ICV
        bool open;
    };

    // Runs one APDU inside the secure channel: encrypt and MAC the command, then
    // verify and decrypt the response. Returns the plaintext length (SW stripped).
    int secureApdu(const uint8_t header[4], const uint8_t *data, int dataLen, bool expectResponse, uint8_t *resp,
                   int respCap, uint16_t *sw);

    // Same, but nested inside a UserID session (ProcessSessionCmd). Key agreement
    // needs the object bound to a session authenticator; the secure channel alone
    // is only transport and is not enough.
    int sessionApdu(const uint8_t header[4], const uint8_t *data, int dataLen, bool expectResponse, uint8_t *resp,
                    int respCap, uint16_t *sw);

    void encryptionIcv(bool response, uint8_t icv[16]);
    static void cbc(const uint8_t key[16], const uint8_t iv[16], const uint8_t *in, size_t len, uint8_t *out,
                    bool encrypt);
    static const uint8_t *tlv1(const uint8_t *resp, int len, int *valueLen);
    static void reverse(const uint8_t *in, uint8_t *out, size_t len);

    static void cmac(const uint8_t key[16], const uint8_t *data, size_t len, uint8_t out[16]);
    static void kdf(const uint8_t key[16], uint8_t constant, uint16_t bits, const uint8_t context[16], uint8_t out[16]);
    void sessionKeys(const uint8_t context[16]);
    void cryptogram(uint8_t constant, const uint8_t context[16], uint8_t out[8]);
    void chainedCmac(const uint8_t *cmd, size_t len, uint8_t mac[8]);
    bool initializeUpdate(const uint8_t hostChallenge[8], uint8_t cardChallenge[8], uint8_t cardCryptogram[8]);

    TwoWire &bus;
    uint8_t address;
    uint8_t seq = 0; // host N(S), toggled per I-block, reset by the interface reset
    Scp03 scp = {};
    uint8_t sessionId[8] = {};
    bool identityReady = false;
};

#endif // HAS_SE050
