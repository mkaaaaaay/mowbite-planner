# what the scripts in the container expect from OpenMower, like src/lib/openmower.ts for the app.
# OpenMower is being restructured (OSv3), when names change it should be here
TOPIC_ROBOT_STATE=robot_state/json
TOPIC_EVENTS=events/json
TOPIC_SENSOR_DATA='sensors/+/data'
TOPIC_ACTION=action
TOPIC_MAP=map/json
ACTION_START_MOWING=mower_logic:idle/start_mowing
ACTION_SKIP_AREA=mower_logic:mowing/skip_area
ACTION_HOME=mower_logic:mowing/abort_mowing
ACTION_RESET_JOB=mower_logic:idle/reset_job
TOPIC_RPC_REQUEST=rpc/request
TOPIC_RPC_RESPONSE=rpc/response
TOPIC_RPC_ERROR=rpc/error
# the last ros warnings and errors, kept in memory by the monitoring node
RPC_LOGS=logs.recent
