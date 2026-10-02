#include "meta_service.hpp"

#include <cstring>

// Major version mower_comms_v2 accepts
static constexpr uint16_t MAJOR_VERSION = 1;
static constexpr auto FIRMWARE_VERSION = "1.0.0-sim";

void MetaService::RPCGetFirmwareVersion(uint16_t call_id, char* data, uint16_t* response_length) {
  (void)data;
  (void)response_length;
  SendRpcResponse(call_id, xbot::datatypes::RpcStatus::SUCCESS, FIRMWARE_VERSION, strlen(FIRMWARE_VERSION));
}

void MetaService::RPCGetMajorVersion(uint16_t call_id) {
  const uint16_t major = MAJOR_VERSION;
  SendRpcResponse(call_id, xbot::datatypes::RpcStatus::SUCCESS, &major, sizeof(major));
}
