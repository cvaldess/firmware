#pragma once

#include "configuration.h"

#if HAS_ETHERNET && defined(HAS_ETHERNET_TLS_API) && defined(ARCH_RP2040)

// HTTPS server on TCP/443 that reuses the HTTP API handlers via the
// transport-agnostic IStreamReadWrite interface (see ethApiHandlers.h). The
// server waits for the cert produced by [[ethCert.h]] before binding the
// listening socket, so it is safe to call this from setup() / reconnectETH().
//
// Idempotent: subsequent calls are no-ops.
void initEthTlsApiServer();

/// Reset the TLS server to its pre-bind state (frees the mbedTLS context, drops
/// the TCP/443 listener, clears the ready flag) so the worker rebuilds and
/// rebinds on its next tick. Called by reconnectETH() after a W5500 chip reset.
void deInitEthTlsApiServer();

/// Move the current connection's deadline a full session budget past now. For a handler that streams a
/// long body after authenticating (the OTA upload), called as the data keeps arriving: the deadline is
/// there so a client trickling bytes cannot hold core0, and an upload that stalls still runs into it.
void ethTlsApiRenewDeadline();

#endif // HAS_ETHERNET && HAS_ETHERNET_TLS_API && ARCH_RP2040
