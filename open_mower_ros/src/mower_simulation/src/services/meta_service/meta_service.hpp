#ifndef META_SERVICE_HPP
#define META_SERVICE_HPP

#include <MetaServiceBase.hpp>

using namespace xbot::service;

// Answers the firmware version queries of mower_comms_v2, which keeps the motors disabled
// (FIRMWARE_INCOMPATIBLE) until it gets a compatible major version.
class MetaService : public MetaServiceBase {
 public:
  explicit MetaService(uint16_t service_id) : MetaServiceBase(service_id) {
  }

 protected:
  void RPCGetFirmwareVersion(uint16_t call_id, char* data, uint16_t* response_length) override;
  void RPCGetMajorVersion(uint16_t call_id) override;
};

#endif  // META_SERVICE_HPP
