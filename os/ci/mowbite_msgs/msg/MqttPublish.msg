# bridge/publish: a text the bridge sends unchanged on MQTT; events/json also goes into the event history.
string topic
string payload
# Retained texts are sent with QoS 1 and again after every new connection.
bool retain
