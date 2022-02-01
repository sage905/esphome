#pragma once

#include "remote_base.h"

namespace esphome {
namespace remote_base {

static uint8_t startcode = 0xa3;

struct AOKData {
  uint32_t device;
  uint16_t address;
  uint8_t command;
  bool preamble = false;

  bool operator==(const AOKData &rhs) const {
    return device == rhs.device && address == rhs.address && 
    command == rhs.command;
  }

  int8_t checksum() const {
        int8_t checksum = (device & 0xff) + (device >> 8 & 0xff) 
        + (device >> 16 & 0xff) + (address & 0xff) + (address >> 8 & 0xff) 
        + command;
        checksum &= 0xff;
        return checksum;
  }
};

class AOKProtocol : public RemoteProtocol<AOKData> {
 public:
  void preamble(RemoteTransmitData *dst) const;
  void one(RemoteTransmitData *dst) const;
  void zero(RemoteTransmitData *dst) const;
  void sync(RemoteTransmitData *dst) const;
  void eom(RemoteTransmitData *dst) const;
  void encode(RemoteTransmitData *dst, const AOKData &data) override;
  optional<AOKData> decode(RemoteReceiveData src) override;
  void dump(const AOKData &data) override;
  bool expect_one(RemoteReceiveData &src) const;
  bool expect_zero(RemoteReceiveData &src) const;
  bool expect_sync(RemoteReceiveData &src) const;
  bool expect_eom(RemoteReceiveData &src) const;
  uint32_t decode_bits(RemoteReceiveData &src, uint8_t length) const;

};

DECLARE_REMOTE_PROTOCOL(AOK)

template<typename... Ts> class AOKAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint32_t, device)
  TEMPLATABLE_VALUE(uint16_t, address)
  TEMPLATABLE_VALUE(uint8_t, command)
  TEMPLATABLE_VALUE(bool, preamble)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    AOKData data{};
    data.device = this->device_.value(x...);
    data.address = this->address_.value(x...);
    data.command = this->command_.value(x...);
    data.preamble = this->preamble_.value(x...);
    AOKProtocol().encode(dst, data);
  }
};

}  // namespace remote_base
}  // namespace esphome
