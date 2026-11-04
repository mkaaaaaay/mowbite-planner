# bridge/rpc/request: a JSON-RPC call for a method a node registered on bridge/rpc/methods. Every node gets every
# call and answers only its own methods.
string method
# The params as JSON text, empty without params.
string params
# Empty when the caller sent no id.
string id
