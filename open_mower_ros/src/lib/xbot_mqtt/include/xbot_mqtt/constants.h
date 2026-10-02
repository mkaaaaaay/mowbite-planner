#pragma once

namespace xbot_mqtt {

inline constexpr const char* TOPIC_REQUEST = "/xbot/rpc/request";
inline constexpr const char* TOPIC_RESPONSE = "/xbot/rpc/response";
inline constexpr const char* TOPIC_ERROR = "/xbot/rpc/error";
inline constexpr const char* SERVICE_REGISTER_METHODS = "/xbot/rpc/register";
// latched by xbot_monitoring when it starts, providers register their methods again then
inline constexpr const char* TOPIC_REGISTRY = "/xbot/rpc/registry";

}  // namespace xbot_mqtt
