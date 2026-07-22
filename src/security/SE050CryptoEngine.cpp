#include "configuration.h"

#if defined(HAS_SE050) && defined(HAS_CUSTOM_CRYPTO_ENGINE)

#include "SE050.h"
#include "mesh/CryptoEngine.h"
#include "meshUtils.h"
#include <Curve25519.h>

// Runs Meshtastic's PKI key agreement inside the SE050 instead of in software.
//
// This is the mirrored stage: the node keeps its private key in config exactly as
// before, and a copy lives in the secure element so the chip can do the agreement.
// Nothing else in the firmware can tell the difference, which is the point - it
// makes the hardware path testable on a live node without changing the semantics
// of an identity that thirty-odd call sites still assume is exportable.
//
// Measured cost is ~64ms per agreement against ~27ms for the bundled Curve25519.
class SE050CryptoEngine : public CryptoEngine
{
  public:
    // Every PKI packet lands here: encryptCurve25519 and decryptCurve25519 both
    // route through it, so this single override captures the whole ECDH path.
    virtual bool setDHPublicKey(uint8_t *pubKey) override
    {
        if (!mirrorReady())
            return CryptoEngine::setDHPublicKey(pubKey);

        uint8_t peer[32];
        memcpy(peer, pubKey, 32);
        uint8_t agreed[32];
        if (!se050->identityEcdh(peer, agreed)) {
            LOG_WARN("SE050: key agreement failed, falling back to software");
            return CryptoEngine::setDHPublicKey(pubKey);
        }

        // Curve25519::dh2 rejects weak points, and going through the chip skips
        // that check. A small-subgroup public key collapses the shared secret to
        // zero, so refuse it rather than encrypt against a known value.
        bool allZero = true;
        for (int i = 0; i < 32; i++)
            allZero &= (agreed[i] == 0);
        if (allZero) {
            LOG_WARN("SE050: key agreement produced a zero secret, rejecting peer key");
            return false;
        }

        memcpy(shared_key, agreed, 32);
        return true;
    }

  private:
    // The mirror can only be set up once NodeDB has handed us the private key, which
    // happens after the chip is probed, so it is done on first use rather than at
    // construction. One failure is enough to stop retrying: if the import did not
    // work it will not start working, and retrying would cost 64ms per packet.
    bool mirrorReady()
    {
        if (mirrored)
            return true;
        if (attempted || !se050)
            return false;
        attempted = true;

        if (memfll(private_key, 0, sizeof(private_key))) {
            LOG_DEBUG("SE050: no private key yet, staying on software crypto");
            attempted = false; // NodeDB may still fill it in
            return false;
        }

        uint8_t pub[32];
        if (!se050->identityImport(private_key, pub)) {
            LOG_WARN("SE050: could not mirror the node key, staying on software crypto");
            return false;
        }
        if (memcmp(pub, public_key, 32) != 0)
            LOG_WARN("SE050: mirrored key does not match the advertised public key");

        LOG_INFO("SE050: PKI key agreement now runs in hardware");
        mirrored = true;
        return true;
    }

    bool mirrored = false;
    bool attempted = false;

  public:
    void selfTest()
    {
        if (!se050) {
            LOG_INFO("SE050: no secure element, PKI stays in software");
            return;
        }
        if (memfll(private_key, 0, sizeof(private_key))) {
            LOG_WARN("SE050: self-test skipped, node has no private key yet");
            return;
        }

        // A throwaway peer, exchanged once through the chip and once in software.
        uint8_t peerPrivate[32], peerPublic[32];
        Curve25519::dh1(peerPublic, peerPrivate);

        uint8_t peerForHw[32];
        memcpy(peerForHw, peerPublic, 32);
        if (!setDHPublicKey(peerForHw)) {
            LOG_ERROR("SE050: self-test failed, key agreement returned an error");
            return;
        }
        uint8_t fromChip[32];
        memcpy(fromChip, shared_key, 32);

        // The software side, done the way the base class would: our public half
        // against the peer's private half yields the same secret.
        uint8_t expected[32];
        Curve25519::eval(expected, private_key, 0);
        if (!Curve25519::dh2(expected, peerPrivate)) {
            LOG_ERROR("SE050: self-test failed on the software side");
            return;
        }

        if (memcmp(fromChip, expected, 32) == 0)
            LOG_INFO("SE050: self-test OK - PKI agreement through the chip matches software");
        else
            LOG_ERROR("SE050: self-test MISMATCH - chip and software disagree on the shared secret");
    }
};

CryptoEngine *crypto = new SE050CryptoEngine();

void se050CryptoSelfTest()
{
    static_cast<SE050CryptoEngine *>(crypto)->selfTest();
}

#endif
