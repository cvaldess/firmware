#pragma once

/*
 * Nordic nRF54LM20-DK (PCA10184) with an EBYTE E22-900M30S (SX1262).
 *
 * The DK carries an nRF54LM20B; the image is built for the nRF54LM20A (same die without the Axon NPU).
 *
 * This header shadows the framework's variants/nrf54lm20dk/variant.h, so it carries the core
 * pin table definitions as well. Arduino pin = physical GPIO: P0.n = n, P1.n = 32+n, P2.n = 64+n,
 * P3.n = 96+n.
 *
 * Serial peripherals are port bound: SERIAL00 (UARTE00/SPIM00) -> P2, SERIAL2x -> P1 and P3,
 * SERIAL30 -> P0. Pin interrupts: P0 -> GPIOTE30, P1 and P3 -> GPIOTE20, P2 none.
 *
 * VDD: the DK ships at 1.8 V. The E22's SPI and DIO lines are 3.3 V, so VDD:nRF and VDD:IO must be
 * raised to 3.3 V in the Board Configurator before the module is connected (Nordic advises 2.7 V at
 * most only because of the on-board LEDs).
 *
 * E22 wiring - PROVISIONAL, kept identical to the nRF54L15-DK so the same harness moves across;
 * to be checked against the DK user guide (header positions, MX25R64 sharing P2.01-P2.05):
 *   SCK P2.01, MOSI P2.02, BUSY P2.03, MISO P2.04, NSS P2.05, RXEN P2.07, NRESET P2.00, DIO1 P0.00
 *   DIO2 -> TXEN bridge on the module, DIO3 drives the TCXO (1.8 V).
 */

#define VARIANT_MCK (128000000ul)
#define USE_LFXO

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PINS_COUNT (109)
#define NUM_DIGITAL_PINS (109)
#define NUM_ANALOG_INPUTS (8)
#define NUM_ANALOG_OUTPUTS (0)
#define ADC_RESOLUTION 14

// LEDs (active HIGH on this DK): LED0 P1.22 status, LED1 P1.25
#define PIN_LED1 54
#define PIN_LED2 57
#define LED_BUILTIN PIN_LED1
#define LED_STATE_ON 1

// BTN0 P1.26 (active low)
#define PIN_BUTTON1 58
#define BUTTON_NEED_PULLUP

// Serial1: VCOM0 of the on-board J-Link (UARTE20): TX P1.16, RX P1.17
#define PIN_SERIAL1_RX 49
#define PIN_SERIAL1_TX 48
#define SERIAL1_UARTE NRF_UARTE20
#define SERIAL1_IRQN SERIAL20_IRQn
#define SERIAL1_IRQ_HANDLER SERIAL20_IRQHandler

// SPI (SPIM00) for the E22
#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MISO 68
#define PIN_SPI_MOSI 66
#define PIN_SPI_SCK 65
static const uint8_t SS = 69;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

// I2C (TWIM30): SDA P0.03, SCL P0.04, external 4.7k pull-ups required
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA 3
#define PIN_WIRE_SCL 4
#define WIRE_TWIM NRF_TWIM30
#define WIRE_TWIS NRF_TWIS30
#define WIRE_IRQN SERIAL30_IRQn
#define WIRE_IRQ_HANDLER SERIAL30_IRQHandler

#ifdef __cplusplus
}
#endif

// SX1262 / E22-900M30S
#define USE_SX1262
#define SX126X_CS 69
#define SX126X_DIO1 0 // P0.00: P2 has no GPIOTE
#define SX126X_BUSY 67
#define SX126X_RESET 64
// RXEN is held high permanently (LNA always on); TXEN follows DIO2.
#define SX126X_ANT_SW 71
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8f
