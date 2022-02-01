#include "aok_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.aok";
/* 
 * This plugin takes care of sending and receiving (in progress) the A-OK protocl used for 
 * Window Shades. This has been tested on AM25 433MHz shades from Zemismart.
 * 
 * Author  (present)  : Patrick Toal (Sage905)
 * 
 * https://www.a-okmotors.com/en/
 * 
 * This is an implementation of the A-OK protocol. 
 * Thanks to Jason von Nieda and Akirjavainen for their work in decoding the protocol and
 * providing many of the details below.  I have refined the data based on my observations.
 *
 * Ref: https://github.com/akirjavainen/A-OK
 *
 * PROTOCOL DESCRIPTION
 * 
 * The pulse multiplier (p) is 300us.
 * 
 * The data packets are combinations of H/L pulses.  
 * 
 * The table below describes the various items sent (in pulse multiples and usecs):
 * 
 *             |  H  |  L  |   H   |   L   |
 * ============+=====+=====+=======+=======+
 * SYNC        | 17p | 1p  | 5100us| 300us |
 * DATA ONE    | 2p  | 1p  | 600us | 300us |
 * DATA ZERO   | 1p  | 2p  | 300us | 600us |
 * EOM         | 2p  | 17p | 600us | 5100us|
 * 
 * The format of a complete packet is:
 * <Preamble> [<Sync><Message 1><EOM>]x6 [<Sync><Message 2><EOM>]x6(optional)
 *
 * The Preamble is sent by some remotes, but not all.  When sent, it is 8 x DATA ZERO.  
 * I suspect this is to "wake up" receivers.  
 *  
 * Message = 64-bit command message (see below). All messages are sent 6 times in a row.  
 * 
 * NOTE: The EOM could be interepreted in multiple ways.  Either as a 65th bit to the message, 
 * plus a silence, or as I have interpreted it, as a High of 2p, followed by a LOW of 17p. This
 * keeps the message itself to 64-bits, which seems more likely.
 * 
 * MESSAGE FORMAT
 *
 * Jason von Nieda reverse engineered the protocol, including the checksum. I have ammended it
 * based on my observations.
 *  
 * 64 bits of data. The packet format is:
 * 
 * [Start][ID][Address][Command][Checksum][1]
 *
 * Start: 8 bits unsigned, always 0xa3, hardcoded.
 * ID: 24 bits unsigned, unique per remote.
 * Address: 16 bits unsigned. This is a bit field so a remote can have up to 16 channels, 
 *   or you can send multiple bits as 1 to trigger multiple channels at once.
 * Command: 8 bits unsigned:
 *   UP = 11 / 0x0b
 *   DOWN = 67 / 0x43
 *   AFTER UP/DOWN = 36 / 0x24
 *   DOWN LONG PRESS = 195 / 0xC3
 *   UP LONG PRESS = 139 / 0x8B
 *   STOP = 35 / 0x23
 *   PROGRAM = 83 / 0x53
 * Checksum: 8 bits unsigned, 8 bit sum of ID, Address, and Command.
 * 
 * EXAMPLE:
 * 
 * SSSSSSSS IIIIIIIIIIIIIIIIIIIIIIII AAAAAAAAAAAAAAAA CCCCCCCC KKKKKKKK
 * 10100011 010100000101110111101001 0000000100000000 00001011 10100010 - UP
 * 10100011 010100000101110111101001 0000000100000000 01000011 11011010 - DOWN
 * 10100011 010100000101110111101001 0000000100000000 00100100 10111011 - AFTER UP/DOWN
 * 10100011 010100000101110111101001 0000000100000000 10001011 01101110 - UP LONG PRESS
 * 10100011 010100000101110111101001 0000000100000000 11000011 01011010 - DOWN LONG PRESS
 * 10100011 010100000101110111101001 0000000100000000 00100011 10111010 - STOP
 *
 * S = Start Code (Always the same)
 * I = Transmitter ID
 * A = Address (Bitflags)
 * C = Command 
 * K = Checksum
 * 
 * BUTTON PRESSES
 * 
 * (These observations based on a AC123-02D remote)
 * UP and DOWN buttons behave differently, based on whether it's a quick press, or a long press.
 *
 * For a quick press (< 1s) of an UP or DOWN button, the remote will send:
 * t: 0ms    code: UP or DOWN x 6
 * t: 41ms   code: AFTER_UPDOWN x 6
 * 
 * For a long press (>1s) of an UP or DOWN button, the remote will send:
 * t: 0ms    code: UP or DOWN x6
 * t: 1000ms code: repeat UP or DOWN x6
 * t: 1500ms code: UP or DOWN LONG PRESS x6
 * 
 * The STOP button only sends one packet with the STOP command x6, regardless of how long 
 * the button is pressed.
 * 
 
 \*********************************************************************************************/

static const uint16_t AOK_PULSE_US = 300; // 300us pulse length

/* Pulse count below is multiplied by the pulse length above.
 * All pulses start High, so all of the pulses below are defined as a series of: {H, L}
 * H = duration of High signal
 * L = duration of Low signal
 * stated in multiples of the AOK_PULSE_LENGTH
 */ 
static const uint8_t AOK_PREAMBLE_LENGTH = 8; // Number of binary 0's to send to wake up the receiver.

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
  if (data.preamble == true) {
    this->preamble(dst);
  }

  for (int8_t i = 0 ; i < 6 ; i++) {
    // Send SYNC
    this->sync(dst);

    // Start Code
    for (int16_t i = 8 - 1; i >= 0; i--) {
      if (startcode & (1 << i)) {
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

bool AOKProtocol::expect_one(RemoteReceiveData &src) const {
  if (!src.peek_mark(AOK_ONE[0]))
    return false;
  if (!src.peek_space(AOK_ONE[1], 1))
    return false;
  src.advance(2);
  return true;
}

bool AOKProtocol::expect_zero(RemoteReceiveData &src) const {
  if (!src.peek_mark(AOK_ZERO[0]))
    return false;
  if (!src.peek_space(AOK_ZERO[1], 1))
    return false;
  src.advance(2);
  return true;
}

bool AOKProtocol::expect_sync(RemoteReceiveData &src) const { 
  if (!src.peek_mark(AOK_SYNC[0]))
    return false;
  if (!src.peek_space(AOK_SYNC[1], 1))
    return false; 
  src.advance(2);
  return true;
}

bool AOKProtocol::expect_eom(RemoteReceiveData &src) const {
  if (!src.peek_mark(AOK_EOM[0]))
    return false;
  if (!src.peek_space(AOK_EOM[1], 1))
    return false;
  src.advance(2);
  return true;
}

uint32_t AOKProtocol::decode_bits(RemoteReceiveData &src, uint8_t length) const {
  uint32_t result = 0;

  for (uint8_t i = 0 ; i < length; i++ ) {
    result <<= 1UL;
    if (expect_one(src)) {
      result |= 0x01;
    } else if (expect_zero(src)) {
      result |= 0x00;
    } else {
      return -1;
    }
  }
  return result;
}

optional<AOKData> AOKProtocol::decode(RemoteReceiveData src) {
  AOKData out{
      .device = 0,
      .address = 0,
      .command = 0,
      .preamble = true,
  };

  // Scan for a Sync
  for (uint16_t i=0 ; i < src.size() ; i++ ) {
    if (src.peek_item(AOK_SYNC[0],AOK_SYNC[1])) {
      break;
    } else {
      src.advance();
    }
  }

  while (1) {
 
  // Require Sync pulse
  if (!expect_sync(src))
    return {};
  
  uint8_t start = 0;
  start = decode_bits(src, 8);
  if (start == -1 || start != 0xA3) return {};

  out.device = decode_bits(src, 24);
  if (out.device == -1) return {};

  out.address = decode_bits(src, 16);
  if (out.address == -1) return {};

  out.command = decode_bits(src, 8);
  if (out.command == -1) return {};

  int8_t checksum = decode_bits(src, 8);
  if (checksum == -1) return {};

  if (checksum == out.checksum()) {
    return out;
  } else {
    return {}; // Checksum didn't match.  Throw it away.
  }
  expect_eom(src);
  }
}

void AOKProtocol::dump(const AOKData &data) {
  ESP_LOGD(TAG, "Received AOK: device=0x%03X address=0x%04X command=0x%01X", 
  data.device, data.address, data.command);
}

}  // namespace remote_base
}  // namespace esphome
