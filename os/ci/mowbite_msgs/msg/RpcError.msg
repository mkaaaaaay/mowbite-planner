# bridge/rpc/error: a call that failed.
int32 ERROR_PARSE=-32700
int32 ERROR_INVALID_REQUEST=-32600
int32 ERROR_METHOD_NOT_FOUND=-32601
int32 ERROR_INVALID_PARAMS=-32602
int32 ERROR_INTERNAL=-32603
int32 code
string message
string id
# Optional JSON text with details, empty for none.
string data
