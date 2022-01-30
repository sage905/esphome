#include "aok_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.aok";
/* 
 * This plugin takes care of sending and receiving the A-OK protocl used for Window Shades
 * 
 * Author  (present)  : Patrick Toal (aka Sage905)
 * 
 * https://www.a-okmotors.com/en/
 * 
 * This is an implementation of the A-OK protocol. 
 * Thanks to Jason von Nieda and Akirjavainen for their work in decoding the protocol and
 * providing many of the details below 
 *
 * Ref: https://github.com/akirjavainen/A-OK
 *
 * PROTOCOL DESCRIPTION
 * 
 * UP and DOWN buttons send two different commands for some reason, listed below
 * as AOK_UP/DOWN_EXAMPLE and AOK_AFTER_UP_DOWN_EXAMPLE. However, the latter command
 * would not seem to be required at all.
 * 
 * Tri-state bits are used.
 * 
 * AGC:
 * Some remotes start the first command with a preamble of 8 times: HIGH of approx. 340 us + LOW of approx. 520 us
 * But most remotes do not transmit that preamble.
 * Every remote starts the command with an AGC HIGH of approx. 5200 us.
 * Then go LOW for approx. 530 us and start the commands bits.
 *
 * RADIO SILENCE:
 * Some remotes instantly repeat the commands, some transmit a radio silence of approx. 5030 us at the end
 * of each command.
 * 
 * Pulse length:
 * SHORT = 300 us 
 * LONG = 600 us
 *
 * Data bits:
 * Data 0 = short HIGH, long LOW (wire 100)
 * Data 1 = long HIGH, short LOW (wire 110)
 * 
 * This code transmits a radio silence of 5100us after each command, although not all remotes do.
 *
 * Jason von Nieda reverse engineered the protocol, including the checksum. 
 * This is from his post from Github:
 * 
 * 64 bits of data, with a trailing 1 making it 65 bits transmitted. The packet format is:
 * 
 * [Start][ID][Address][Command][Checksum][1]
 *
 * Start: 8 bits unsigned, always 0xa3, hardcoded.
 * ID: 24 bits unsigned, unique per remote.
 * Address: 16 bits unsigned. This is a bit field so a remote can have up to 16 channels, 
 *   or you can send multiple bits as 1 to trigger multiple channels at once.
 * Command: 8 bits unsigned (UP = 11, DOWN = 67, STOP = 35, PROGRAM = 83, AFTER UP/DOWN = 36).
 * Checksum: 8 bits unsigned, 8 bit sum of ID, Address, and Command.
 * 
 * SSSSSSSS IIIIIIIIIIIIIIIIIIIIIIII AAAAAAAAAAAAAAAA CCCCCCCC KKKKKKKK E
 * 10100011 010100000101110111101001 0000000100000000 00001011 10100010 1 - UP
 * 10100011 010100000101110111101001 0000000100000000 01000011 11011010 1 - DOWN
 * 10100011 010100000101110111101001 0000000100000000 00100100 10111011 1 - AFTER UP/DOWN
 * 10100011 010100000101110111101001 0000000100000000 00100011 10111010 1 - STOP
 *
 * S = Start Code (Always the same)
 * I = Transmitter ID
 * A = Address (Bitflags)
 * C = Command 
 * K = Checksum
 * E = Extra bit (always 1)
 \*********************************************************************************************/

static const uint16_t AOK_PULSE_US = 300; // 300us pulse length

/* Pulse count below is multiplied by the pulse length above.
 * All pulses start High, so all of the pulses below are defined as a series of: {H, L}
 * H = duration of High signal
 * L = duration of Low signal
 * stated in multiples of the AOK_PULSE_LENGTH
 */ 
static const uint8_t AOK_PREAMBLE_LENGTH = 8; // Number of binary 1's to send to wake up the receiver.

static const uint16_t AOK_SYNC [] = { 17 * AOK_PULSE_US, 2 * AOK_PULSE_US};
static const uint16_t AOK_ONE [] = { 2 * AOK_PULSE_US, 1 * AOK_PULSE_US };
static const uint16_t AOK_ZERO [] = { 1 * AOK_PULSE_US, 2 * AOK_PULSE_US };
static const uint16_t AOK_EOM [] = { 2 * AOK_PULSE_US, 17 * AOK_PULSE_US };

void AOKProtocol::preamble(RemoteTransmitData *dst) const {
  for (int i = 0; i < AOK_PREAMBLE_LENGTH ; i++) {
    zero(dst);
  }
}
void AOKProtocol::one(RemoteTransmitData *dst) const {
  // Tri-bit for a binary 1
  dst->item(AOK_ONE[0], AOK_ONE[1]);
}

void AOKProtocol::zero(RemoteTransmitData *dst) const {
  // Tri-bit for a binary 0
  dst->item(AOK_ZERO[0], AOK_ZERO[1]);
}

void AOKProtocol::sync(RemoteTransmitData *dst) const { 
  // Sync Pulse
  dst->item(AOK_SYNC[0], AOK_SYNC[1]); 
}

void AOKProtocol::eom(RemoteTransmitData *dst) const {
  // EOM
  dst->item(AOK_EOM[0],AOK_EOM[1]);
}

void AOKProtocol::encode(RemoteTransmitData *dst, const AOKData &data) {
  dst->set_carrier_frequency(0);

  // Send Preamble
  this->preamble(dst);

  for (int8_t i = 0 ; i < 6 ; i++) {
    // Send SYNC
    this->sync(dst);

    // Start Code
    for (int16_t i = 8 - 1; i >= 0; i--) {
      if (data.start & (1 << i)) {
        this->one(dst);
      } else {
        this->zero(dst);
      }
    }

    // Device ID (24 bits)
    for (int16_t i = 24 - 1; i >= 0; i--) {
      if (data.device & (1 << i)) {
        this->one(dst);
      } else {
        this->zero(dst);
      }
    }

    // Address (16 bits)
    for (int16_t i = 16 - 1; i >= 0; i--) {
      if (data.address & (1 << i)) {
        this->one(dst);
      } else {
        this->zero(dst);
      }
    }

    // Command (8 bits)
    for (int16_t i = 8 - 1; i >= 0; i--) {
      if (data.command & (1 << i)) {
        this->one(dst);
      } else {
        this->zero(dst);
      }
    }

    // Checksum (8 bits)
    int8_t checksum = data.checksum();

    for (int16_t i = 8 - 1; i >= 0; i--) {
      if (checksum & (1 << i)) {
        this->one(dst);
      } else {
        this->zero(dst);
      }
    }

    // Send EOM
    this->eom(dst);
  }
}

optional<AOKData> AOKProtocol::decode(RemoteReceiveData src) {
  AOKData out{
      .start = 0,
      .device = 0,
      .address = 0,
      .command = 0,
  };

  // Require Sync pulse
  if (!src.expect_pulse_with_gap(AOK_SYNC[0], AOK_SYNC[1]))
    return {};
  
  ESP_LOGD(TAG, "Received AOK: sync pulse");
  
  // Start
  for (uint8_t i = 0 ; i < 8; i++ ) {
    out.start <<= 1UL;
    if (src.expect_item(AOK_ONE[0],AOK_ONE[1])) {
      out.start |= 0x01;
    } else if (src.expect_item(AOK_ZERO[0], AOK_ZERO[0])) {
      out.start |= 0x00;
    } else {
      // Wait, Wut?
      return {};
    }
  }
  
  // DeviceID
  for (uint8_t i = 0 ; i < 24; i++ ) {
    out.device <<= 1UL;
    if (src.expect_item(AOK_ONE[0],AOK_ONE[1])) {
      out.device |= 0x01;
    } else if (src.expect_item(AOK_ZERO[0], AOK_ZERO[0])) {
      out.device |= 0x00;
    } else {
      // Wait, Wut?
      return {};
    }
  }

  // Address
  for (uint8_t i = 0 ; i < 16; i++ ) {
    out.address <<= 1UL;
    if (src.expect_item(AOK_ONE[0],AOK_ONE[1])) {
      out.address |= 0x01;
    } else if (src.expect_item(AOK_ZERO[0], AOK_ZERO[0])) {
      out.address |= 0x00;
    } else {
      // Wait, Wut?
      return {};
    }
  }

  // Command
  for (uint8_t i = 0 ; i < 8; i++ ) {
    out.command <<= 1UL;
    if (src.expect_item(AOK_ONE[0],AOK_ONE[1])) {
      out.command |= 0x01;
    } else if (src.expect_item(AOK_ZERO[0], AOK_ZERO[0])) {
      out.command |= 0x00;
    } else {
      // Wait, Wut?
      return {};
    }
  }

  int8_t checksum = 0;
  // Checksum
  for (uint8_t i = 0 ; i < 8; i++ ) {
    checksum <<= 1UL;
    if (src.expect_item(AOK_ONE[0],AOK_ONE[1])) {
      checksum |= 0x01;
    } else if (src.expect_item(AOK_ZERO[0], AOK_ZERO[0])) {
      checksum |= 0x00;
    } else {
      // Wait, Wut?
      return {};
    }
  }
  return out;
  // if (checksum == out.checksum()) {
  //   return out;
  // } else {
  //   return {}; // Checksum didn't match.  Throw it away.
  // }
}

void AOKProtocol::dump(const AOKData &data) {
  ESP_LOGD(TAG, "Received AOK: device=0x%04X address=%d command=%d", 
  data.device, data.address, data.command);
}

}  // namespace remote_base
}  // namespace esphome
