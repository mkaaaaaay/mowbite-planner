// Everything MowBite expects from OpenMower in one place: MQTT topics, RPC methods, action ids,
// parameter names and a few paths that only show up as hints. OpenMower is being restructured
// (OSv3), when names change it should be this file (and docker/openmower.sh for the container).

export const TOPIC = {
  robotState: 'robot_state/json',
  position: 'position/json',
  map: 'map/json',
  mapOverlay: 'map_overlay/json',
  sensorInfos: 'sensor_infos/json',
  sensorData: 'sensors/+/data',
  params: 'params/json',
  actions: 'actions/json',
  action: 'action',
  teleop: 'teleop',
  rpcRequest: 'rpc/request',
  rpcResponse: 'rpc/response',
} as const;

export const RPC = {
  // what this mower answers, newer OpenMower versions only
  methods: 'rpc.methods',
  replaceMap: 'map.replace',
  eventDays: 'events.history.list',
  events: 'events.history',
  jobs: 'position.history.list',
  jobTrack: 'position.history',
  // the mowing plan for an area straight from the mower's planner. OpenMower doesn't offer it yet,
  // the name is a guess until it does
  areaPlan: 'mowing.plan',
  // the area settings this mower keeps in map.json
  areaProperties: 'map.area_properties',
  // the last warnings and errors ROS logged, kept in memory by the mower. OpenMower doesn't offer it yet
  logs: 'logs.recent',
} as const;

export const ACTION = {
  startMowing: 'mower_logic:idle/start_mowing',
  pause: 'mower_logic:mowing/pause',
  resume: 'mower_logic:mowing/continue',
  goHome: 'mower_logic:mowing/abort_mowing',
  skipArea: 'mower_logic:mowing/skip_area',
  resetEmergency: 'mower_logic/reset_emergency',
  startRecording: 'mower_logic:idle/start_area_recording',
  // drops an interrupted job, only offered while idle with one (needs an openmower that has it)
  resetJob: 'mower_logic:idle/reset_job',
  // the steps while recording an area, e.g. finish_mowing_area
  recording: (step: string) => `mower_logic:area_recording/${step}`,
} as const;

// ros parameters, as they come in params/json
export const PARAM = {
  mowerLogic: (key: string) => `/mower_logic/${key}`,
  toolWidth: '/mower_logic/tool_width',
  mowAngleOffset: '/mower_logic/mow_angle_offset',
  mowAngleOffsetIsAbsolute: '/mower_logic/mow_angle_offset_is_absolute',
  mowAngleIncrement: '/mower_logic/mow_angle_increment',
  outlineCount: '/mower_logic/outline_count',
  dockingApproachDistance: '/mower_logic/docking_approach_distance',
  maxPositionAccuracy: '/mower_logic/max_position_accuracy',
} as const;

// mower_logic's own defaults (MowerLogic.cfg, the same in 1.4.0 and edge). params/json is sent when xbot_monitoring
// connects to the broker, that can be before mower_logic put its defaults in, then only what the mower's yaml sets
// is there. what's missing then is the default, that's what the mower runs with
export const PARAM_DEFAULTS: Record<string, number> = {
  [PARAM.toolWidth]: 0.14,
  [PARAM.outlineCount]: 3,
  '/mower_logic/outline_overlap_count': 0,
  '/mower_logic/outline_offset': 0,
  [PARAM.mowAngleOffset]: 0,
  [PARAM.mowAngleIncrement]: 0,
  [PARAM.dockingApproachDistance]: 1.5,
  [PARAM.maxPositionAccuracy]: 0.2,
  '/mower_logic/gps_timeout': 10,
};

// where things are on a mower set up with openmower-cli today, only shown to explain things
export const PATHS = {
  eventHistory: '/home/openmower/ros/event_history/',
  checkpoint: '~/ros/checkpoint.bag',
  restart: 'openmower restart',
} as const;

// the source of the mower logic, linked next to events so you can see where they come from
export const LOGIC_SOURCE = 'https://github.com/ClemensElflein/open_mower_ros/blob/main/src/mower_logic/src/mower_logic/';
