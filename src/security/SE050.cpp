#include "SE050.h"

#if defined(HAS_SE050)

#include "configuration.h"
#include <AES.h>
#include <Arduino.h>
#include <string.h>

#ifdef ARCH_RP2040
#include <hardware/watchdog.h>
#define SE050_FEED_WATCHDOG() watchdog_update()
#else
#define SE050_FEED_WATCHDOG() ((void)0)
#endif

namespace
{
constexpr uint8_t NAD_HOST_TO_SE = 0x5A;
constexpr uint8_t NAD_SE_TO_HOST = 0xA5; // also the SOF we resynchronise on
constexpr uint8_t PCB_S_REQ = 0xC0;
constexpr uint8_t PCB_S_RSP = 0xE0;
constexpr uint8_t S_INTF_RESET = 0x0F;
constexpr uint8_t S_WTX = 0x03; // wait-time extension request/response

// The answer to a fresh key generation can take seconds (compute plus an NVM
// write), so the poll window has to be generous. Not-ready simply NACKs, which
// returns immediately, so a normal answer still exits after a couple of passes.
constexpr int POLL_ATTEMPTS = 400;
constexpr uint32_t POLL_INTERVAL_MS = 10;
} // namespace

// CRC-16 as T1oI2C uses it: reflected polynomial 0x8408, init and xorout 0xFFFF,
// and the result byte-swapped (UM11225), appended big-endian.
uint16_t SE050::crc(const uint8_t *data, size_t len)
{
    uint16_t cal = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        cal ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            cal = (cal & 1) ? ((cal >> 1) ^ 0x8408) : (cal >> 1);
    }
    cal ^= 0xFFFF;
    return (uint16_t)(((cal & 0xFF) << 8) | ((cal >> 8) & 0xFF));
}

size_t SE050::xfer(const uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxCap)
{
    if (rxCap < 3)
        return 0;

    bus.beginTransmission(address);
    bus.write(tx, txLen);
    if (bus.endTransmission() != 0) {
        LOG_DEBUG("SE050: write of %u bytes was not acked", (unsigned)txLen);
        return 0;
    }

    // Read in two phases, the way NXP's own PAL does it: poll for the 3-byte
    // header until the start-of-frame shows up, then read exactly the body the
    // header announced. A single large fixed read does not survive a slow answer.
    uint8_t header[3];
    bool haveHeader = false;
    for (int attempt = 0; attempt < POLL_ATTEMPTS && !haveHeader; attempt++) {
        delay(POLL_INTERVAL_MS);
        SE050_FEED_WATCHDOG(); // this loop can run for seconds
        if (bus.requestFrom(address, (uint8_t)sizeof(header)) == sizeof(header)) {
            for (size_t i = 0; i < sizeof(header); i++)
                header[i] = bus.read();
            haveHeader = (header[0] == NAD_SE_TO_HOST);
        }
    }
    if (!haveHeader)
        return 0;

    rx[0] = header[0];
    rx[1] = header[1];
    rx[2] = header[2];

    size_t body = (size_t)header[2] + 2; // INF plus the two CRC bytes
    if (3 + body > rxCap) {
        LOG_WARN("SE050: response of %u bytes does not fit in %u", (unsigned)(3 + body), (unsigned)rxCap);
        return 0;
    }
    if (body > 0) {
        if (bus.requestFrom(address, (uint8_t)body) != body) {
            LOG_DEBUG("SE050: body read failed (LEN=%u)", (unsigned)header[2]);
            return 0;
        }
        for (size_t i = 0; i < body; i++)
            rx[3 + i] = bus.read();
    }

    // Validate the CRC so a desynchronised read is discarded rather than parsed.
    // Retransmitting resynchronises the SE050, so the caller can simply retry.
    size_t total = 3 + body;
    if (total >= 5) {
        uint16_t want = crc(rx, total - 2);
        if (rx[total - 2] != ((want >> 8) & 0xFF) || rx[total - 1] != (want & 0xFF)) {
            LOG_DEBUG("SE050: response CRC mismatch, discarding frame");
            return 0;
        }
    }
    return total;
}

bool SE050::reset(uint8_t *atrOut, size_t atrCap, size_t *atrLen)
{
    uint8_t frame[5] = {NAD_HOST_TO_SE, (uint8_t)(PCB_S_REQ | S_INTF_RESET), 0x00, 0, 0};
    uint16_t c = crc(frame, 3);
    frame[3] = (c >> 8) & 0xFF;
    frame[4] = c & 0xFF;

    const uint8_t expected = (uint8_t)(PCB_S_RSP | S_INTF_RESET); // 0xEF

    // Retransmitting the interface reset is also how a stream that went out of
    // step is recovered, so a failed attempt is worth repeating.
    for (int attempt = 0; attempt < 4; attempt++) {
        uint8_t rx[128];
        size_t n = xfer(frame, sizeof(frame), rx, sizeof(rx));
        if (n >= 3 && rx[1] == expected) {
            size_t len = rx[2];
            if (3 + len > n)
                len = n - 3;
            if (atrOut && atrLen) {
                size_t copy = len < atrCap ? len : atrCap;
                memcpy(atrOut, &rx[3], copy);
                *atrLen = copy;
            }
            seq = 0;
            return true;
        }
        LOG_DEBUG("SE050: interface reset attempt %d gave %s", attempt + 1, n == 0 ? "no valid frame" : "an unexpected PCB");
    }
    return false;
}

uint16_t SE050::statusWord(const uint8_t *resp, int len)
{
    return len >= 2 ? (uint16_t)((resp[len - 2] << 8) | resp[len - 1]) : 0xFFFF;
}

int SE050::transceive(const uint8_t *apdu, size_t apduLen, uint8_t *resp, size_t respCap)
{
    uint8_t frame[288];
    if (5 + apduLen > sizeof(frame))
        return -1;

    frame[0] = NAD_HOST_TO_SE;
    frame[1] = (uint8_t)((seq & 1) << 6); // I-block: bit7=0, N(S) in bit6
    frame[2] = (uint8_t)apduLen;          // LEN, single byte (UM11225)
    memcpy(&frame[3], apdu, apduLen);
    uint16_t c = crc(frame, 3 + apduLen);
    frame[3 + apduLen] = (c >> 8) & 0xFF;
    frame[4 + apduLen] = c & 0xFF;

    uint8_t rx[288];
    size_t n = xfer(frame, 5 + apduLen, rx, sizeof(rx));
    seq ^= 1;
    if (n == 0)
        return -1;

    // WTX: the SE050 asks for more time (S-block request). Grant it and re-read.
    while (n >= 2 && rx[1] == (uint8_t)(PCB_S_REQ | S_WTX)) {
        uint8_t wtx = rx[2] >= 1 ? rx[3] : 1;
        uint8_t w[6] = {NAD_HOST_TO_SE, (uint8_t)(PCB_S_RSP | S_WTX), 0x01, wtx, 0, 0};
        uint16_t wc = crc(w, 4);
        w[4] = (wc >> 8) & 0xFF;
        w[5] = wc & 0xFF;
        n = xfer(w, sizeof(w), rx, sizeof(rx));
        if (n == 0)
            return -1;
    }

    size_t len = rx[2];
    if (3 + len > n)
        len = n > 3 ? n - 3 : 0;
    if (len > respCap)
        len = respCap;
    memcpy(resp, &rx[3], len);
    return (int)len;
}

bool SE050::selectApplet()
{
    // SELECT (by name) the SE05x IoT applet - AID A0000003965453000000010300000000.
    static const uint8_t SEL[] = {0x00, 0xA4, 0x04, 0x00, 0x10, 0xA0, 0x00, 0x00, 0x03, 0x96, 0x54,
                                  0x53, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    uint8_t r[64];
    int n = transceive(SEL, sizeof(SEL), r, sizeof(r));
    uint16_t sw = statusWord(r, n);
    if (sw != 0x9000) {
        LOG_ERROR("SE050: applet SELECT returned SW=%04x", sw);
        return false;
    }
    if (n >= 3)
        LOG_INFO("SE050: IoT applet selected, version %d.%d.%d", r[0], r[1], r[2]);
    return true;
}

bool SE050::open()
{
    if (!reset())
        return false;
    return selectApplet();
}

// --- PlatformSCP03 -------------------------------------------------------
//
// Factory keys for OEF 0x0001A921 (SE050E). These are the default keys NXP ships
// in its middleware, so they are public and confer no secrecy - they only open
// the transport channel the SE050 requires before it will do key agreement.
static const uint8_t SCP_KEY_ENC[16] = {0xD2, 0xDB, 0x63, 0xE7, 0xA0, 0xA5, 0xAE, 0xD7,
                                        0x2A, 0x64, 0x60, 0xC4, 0xDF, 0xDC, 0xAF, 0x64};
static const uint8_t SCP_KEY_MAC[16] = {0x73, 0x8D, 0x5B, 0x79, 0x8E, 0xD2, 0x41, 0xB0,
                                        0xB2, 0x47, 0x68, 0x51, 0x4B, 0xFB, 0xA9, 0x5B};
static constexpr uint8_t SCP03_KEYVER = 0x0B;

// AES-CMAC (RFC 4493). The bundled Crypto library only exposes OMAC in its EAX
// form, which prepends a tag block, so the plain construction is done here.
void SE050::cmac(const uint8_t key[16], const uint8_t *data, size_t len, uint8_t out[16])
{
    AES128 aes;
    aes.setKey(key, 16);

    // Subkeys: L = E(K, 0), K1 = dbl(L), K2 = dbl(K1); dbl left-shifts and, on
    // carry out of the top bit, folds in the 0x87 field polynomial.
    uint8_t k1[16] = {0}, k2[16] = {0};
    aes.encryptBlock(k1, k1);
    for (int round = 0; round < 2; round++) {
        uint8_t *k = round == 0 ? k1 : k2;
        if (round == 1)
            memcpy(k2, k1, 16);
        uint8_t carry = k[0] & 0x80;
        for (int i = 0; i < 15; i++)
            k[i] = (uint8_t)((k[i] << 1) | (k[i + 1] >> 7));
        k[15] = (uint8_t)(k[15] << 1);
        if (carry)
            k[15] ^= 0x87;
    }

    uint8_t state[16] = {0};
    size_t full = len ? (len - 1) / 16 : 0; // blocks processed before the last one
    for (size_t b = 0; b < full; b++) {
        for (int i = 0; i < 16; i++)
            state[i] ^= data[b * 16 + i];
        aes.encryptBlock(state, state);
    }

    // Last block: XOR K1 if it is exactly full, otherwise pad with 0x80 00.. and
    // XOR K2 instead.
    uint8_t last[16] = {0};
    size_t rem = len - full * 16;
    if (len > 0 && rem == 16) {
        memcpy(last, &data[full * 16], 16);
        for (int i = 0; i < 16; i++)
            last[i] ^= k1[i];
    } else {
        memcpy(last, &data[full * 16], rem);
        last[rem] = 0x80;
        for (int i = 0; i < 16; i++)
            last[i] ^= k2[i];
    }
    for (int i = 0; i < 16; i++)
        state[i] ^= last[i];
    aes.encryptBlock(out, state);
}

// SCP03 key derivation (SP800-108 in counter mode, CMAC as the PRF).
void SE050::kdf(const uint8_t key[16], uint8_t constant, uint16_t bits, const uint8_t context[16], uint8_t out[16])
{
    uint8_t dd[32];
    memset(dd, 0, 11);
    dd[11] = constant;
    dd[12] = 0x00;
    dd[13] = (uint8_t)(bits >> 8);
    dd[14] = (uint8_t)(bits & 0xFF);
    dd[15] = 0x01;
    memcpy(&dd[16], context, 16);
    cmac(key, dd, sizeof(dd), out);
}

void SE050::sessionKeys(const uint8_t context[16])
{
    kdf(SCP_KEY_ENC, 0x04, 128, context, scp.senc);
    kdf(SCP_KEY_MAC, 0x06, 128, context, scp.smac);
    kdf(SCP_KEY_MAC, 0x07, 128, context, scp.srmac);
}

// Cryptograms are the first 8 bytes of a 64-bit derivation off S-MAC.
void SE050::cryptogram(uint8_t constant, const uint8_t context[16], uint8_t out[8])
{
    uint8_t full[16];
    kdf(scp.smac, constant, 64, context, full);
    memcpy(out, full, 8);
}

// C-MAC over MCV || command, updating the MCV to the full CMAC so successive
// commands chain.
void SE050::chainedCmac(const uint8_t *cmd, size_t len, uint8_t mac[8])
{
    uint8_t buf[16 + 288];
    if (16 + len > sizeof(buf))
        return;
    memcpy(buf, scp.mcv, 16);
    memcpy(&buf[16], cmd, len);
    uint8_t full[16];
    cmac(scp.smac, buf, 16 + len, full);
    memcpy(scp.mcv, full, 16);
    memcpy(mac, full, 8);
}

bool SE050::initializeUpdate(const uint8_t hostChallenge[8], uint8_t cardChallenge[8], uint8_t cardCryptogram[8])
{
    uint8_t iu[] = {0x80,
                    0x50,
                    SCP03_KEYVER,
                    0x00,
                    0x08,
                    hostChallenge[0],
                    hostChallenge[1],
                    hostChallenge[2],
                    hostChallenge[3],
                    hostChallenge[4],
                    hostChallenge[5],
                    hostChallenge[6],
                    hostChallenge[7],
                    0x00};
    uint8_t r[64];
    int n = transceive(iu, sizeof(iu), r, sizeof(r));
    if (statusWord(r, n) != 0x9000 || n < 31) {
        LOG_ERROR("SE050: INITIALIZE UPDATE (keyver=%02x) SW=%04x n=%d", SCP03_KEYVER, statusWord(r, n), n);
        return false;
    }
    // keyDivData(10) || keyInfo(3) || cardChallenge(8) || cardCryptogram(8)
    memcpy(cardChallenge, &r[13], 8);
    memcpy(cardCryptogram, &r[21], 8);
    return true;
}

bool SE050::openSecureChannel()
{
    memset(&scp, 0, sizeof(scp));

    uint8_t hostChallenge[8];
    for (size_t i = 0; i < sizeof(hostChallenge); i++)
        hostChallenge[i] = (uint8_t)random(256);

    uint8_t cardChallenge[8], cardCryptogram[8];
    if (!initializeUpdate(hostChallenge, cardChallenge, cardCryptogram))
        return false;

    uint8_t context[16];
    memcpy(context, hostChallenge, 8);
    memcpy(&context[8], cardChallenge, 8);
    sessionKeys(context);

    // If this does not match, the static keys or the KDF are wrong - there is no
    // point continuing, and it is also how the chip authenticates itself to us.
    uint8_t expected[8];
    cryptogram(0x00, context, expected);
    if (memcmp(expected, cardCryptogram, 8) != 0) {
        LOG_ERROR("SE050: card cryptogram mismatch - keys rotated, or a different KDF");
        return false;
    }

    uint8_t hostCryptogram[8];
    cryptogram(0x01, context, hostCryptogram);

    // EXTERNAL AUTHENTICATE. CLA 0x84 carries the security bit; P1 0x33 asks for
    // C-DEC | C-MAC | R-MAC | R-ENC. Lc covers the cryptogram plus its C-MAC, and
    // the C-MAC chains from the still-zero MCV.
    uint8_t cmd[13];
    cmd[0] = 0x84;
    cmd[1] = 0x82;
    cmd[2] = 0x33;
    cmd[3] = 0x00;
    cmd[4] = 0x10;
    memcpy(&cmd[5], hostCryptogram, 8);

    uint8_t mac[8];
    chainedCmac(cmd, sizeof(cmd), mac);

    uint8_t apdu[sizeof(cmd) + 8];
    memcpy(apdu, cmd, sizeof(cmd));
    memcpy(&apdu[sizeof(cmd)], mac, 8);

    uint8_t r[32];
    int n = transceive(apdu, sizeof(apdu), r, sizeof(r));
    uint16_t sw = statusWord(r, n);
    if (sw != 0x9000) {
        LOG_ERROR("SE050: EXTERNAL AUTHENTICATE SW=%04x - channel not open", sw);
        return false;
    }

    scp.open = true;
    scp.counter = 0; // the first wrapped command increments this to 1
    return true;
}

bool SE050::probe()
{
    if (!open()) {
        LOG_ERROR("SE050: bring-up failed at 0x%02x", address);
        return false;
    }

    // GetVersion: CLA=80 INS_MGMT=04 P1=00 P2_VERSION=20, Le=00. The 7-byte
    // VersionInfo comes back in a BER-TLV (tag 0x41): applet version, then the
    // 2-byte AppletConfig and 2-byte SecureBox. Long-form length is possible.
    static const uint8_t GV[] = {0x80, 0x04, 0x00, 0x20, 0x00};
    uint8_t r[32];
    int n = transceive(GV, sizeof(GV), r, sizeof(r));
    if (statusWord(r, n) == 0x9000) {
        const uint8_t *vi = r;
        int off = 0;
        if (n >= 2 && r[0] == 0x41) {
            if (r[1] == 0x82)
                off = 4;
            else if (r[1] == 0x81)
                off = 3;
            else
                off = 2;
        }
        vi = &r[off];
        if (n - off >= 7) {
            uint16_t cfg = (uint16_t)((vi[3] << 8) | vi[4]);
            LOG_INFO("SE050: applet %d.%d.%d AppletConfig=0x%04x (DH_MONT %s, FIPS %s)", vi[0], vi[1], vi[2], cfg,
                     (cfg & 0x0008) ? "on" : "off", (cfg & 0x1000) ? "off" : "on");
        }
    } else {
        LOG_WARN("SE050: GetVersion returned SW=%04x", statusWord(r, n));
    }

    // GetRandom (16 bytes): CLA=80 INS_MGMT=04 P1=00 P2_RANDOM=49, TLV 41 02 <size>, Le=00.
    // Proves the on-chip TRNG is live - the bytes must differ every boot.
    static const uint8_t GR[] = {0x80, 0x04, 0x00, 0x49, 0x04, 0x41, 0x02, 0x00, 0x10, 0x00};
    n = transceive(GR, sizeof(GR), r, sizeof(r));
    if (statusWord(r, n) == 0x9000 && n >= 2 && r[0] == 0x41) {
        int off = (r[1] == 0x82) ? 4 : (r[1] == 0x81) ? 3 : 2;
        int rl = (r[1] == 0x82) ? ((r[2] << 8) | r[3]) : (r[1] == 0x81) ? r[2] : r[1];
        char hex[2 * 16 + 1];
        int shown = 0;
        for (int i = 0; i < rl && off + i < n - 2 && shown < 16; i++, shown++)
            snprintf(&hex[shown * 2], 3, "%02X", r[off + i]);
        hex[shown * 2] = '\0';
        LOG_INFO("SE050: GetRandom OK, TRNG live: %s", hex);
    } else {
        LOG_WARN("SE050: GetRandom returned SW=%04x", statusWord(r, n));
    }

    if (openSecureChannel())
        LOG_INFO("SE050: SCP03 secure channel open (chip authenticated)");
    else
        LOG_WARN("SE050: SCP03 secure channel not established");

    return true;
}

#endif // HAS_SE050
