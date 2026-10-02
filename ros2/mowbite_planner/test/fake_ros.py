"""A stand-in for rclpy and the message packages the node uses, for testing it without ROS.

The messages check their fields as strictly as Humble's generated classes do (an int in a float64 field
is an error), and the planner's own interfaces are built from the .msg/.srv files of
mowbite_planner_interfaces, so the test also catches a node that doesn't match them.
install() only puts these into sys.modules; with a sourced ROS 2 workspace the real ones are used.
"""

import enum
import os
import re
import sys
import types

INTERFACES = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'mowbite_planner_interfaces')

_INT_RANGES = {
    'int8': (-2 ** 7, 2 ** 7 - 1), 'uint8': (0, 2 ** 8 - 1), 'int16': (-2 ** 15, 2 ** 15 - 1),
    'uint16': (0, 2 ** 16 - 1), 'int32': (-2 ** 31, 2 ** 31 - 1), 'uint32': (0, 2 ** 32 - 1),
    'int64': (-2 ** 63, 2 ** 63 - 1), 'uint64': (0, 2 ** 64 - 1),
}
_REGISTRY = {}


class Message:
    """Fields: name -> (type, default). type is 'float64', 'string', ... a message class name, or a list of one."""

    _fields = {}

    def __init__(self, **kwargs):
        for name, (ftype, default) in self._fields.items():
            setattr(self, name, kwargs.pop(name) if name in kwargs else _default(ftype, default))
        if kwargs:
            raise AssertionError(f'{type(self).__name__} has no fields {sorted(kwargs)}')

    def __setattr__(self, name, value):
        if name not in self._fields:
            raise AttributeError(f'{type(self).__name__} has no field {name!r}')
        _check(self._fields[name][0], value, f'{type(self).__name__}.{name}')
        object.__setattr__(self, name, value)

    def __repr__(self):
        return f'{type(self).__name__}({", ".join(f"{k}={getattr(self, k)!r}" for k in self._fields)})'


def _default(ftype, default):
    if isinstance(ftype, list):
        return []
    if default is not None:
        return default
    if ftype in ('float32', 'float64'):
        return 0.0
    if ftype in _INT_RANGES:
        return 0
    if ftype == 'bool':
        return False
    if ftype == 'string':
        return ''
    return _REGISTRY[ftype]()


def _check(ftype, value, where):
    if isinstance(ftype, list):
        assert isinstance(value, (list, tuple)), f'{where} must be a sequence, got {type(value).__name__}'
        for v in value:
            _check(ftype[0], v, where + '[]')
    elif ftype in ('float32', 'float64'):
        # Humble's generated setters: `isinstance(value, float)`, an int fails
        assert isinstance(value, float), f'{where} must be a float, got {type(value).__name__} {value!r}'
    elif ftype in _INT_RANGES:
        lo, hi = _INT_RANGES[ftype]
        assert isinstance(value, int) and lo <= value <= hi, f'{where} must be a {ftype}, got {value!r}'
    elif ftype == 'bool':
        assert isinstance(value, bool), f'{where} must be a bool, got {value!r}'
    elif ftype == 'string':
        assert isinstance(value, str), f'{where} must be a str, got {value!r}'
    elif ftype == 'byte':
        assert isinstance(value, bytes) and len(value) == 1, f'{where} must be one byte, got {value!r}'
    else:
        cls = _REGISTRY[ftype]
        assert isinstance(value, cls), f'{where} must be a {cls.__name__}, got {type(value).__name__}'


def _message(name, fields, constants=None):
    cls = type(name.split('/')[-1], (Message,), {'_fields': dict(fields)})
    for k, v in (constants or {}).items():
        setattr(cls, k, v)
    _REGISTRY[name] = cls
    return cls


def _parse(text, package):
    """Fields and constants of a .msg (or one half of a .srv)."""
    fields, constants = {}, {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        m = re.match(r'^(\S+)\s+([A-Z][A-Z0-9_]*)\s*=\s*(\S+)$', line)
        if m:
            ftype, name, value = m.groups()
            constants[name] = float(value) if ftype.startswith('float') else int(value)
            continue
        m = re.match(r'^(\S+?)(\[\])?\s+([a-z][a-z0-9_]*)(?:\s+(\S+))?$', line)
        assert m, f'cannot parse {line!r}'
        base, array, name, default = m.groups()
        if '/' not in base and base not in _INT_RANGES and base not in ('float32', 'float64', 'bool', 'string'):
            base = f'{package}/{base}'
        if default is not None:
            default = {'true': True, 'false': False}.get(default) if base == 'bool' else \
                float(default) if base.startswith('float') else int(default)
        fields[name] = ([base] if array else base, default)
    return fields, constants


def _interfaces(package, folder):
    msgs = types.ModuleType(f'{package}.msg')
    srvs = types.ModuleType(f'{package}.srv')
    for fname in sorted(os.listdir(os.path.join(folder, 'msg'))):
        with open(os.path.join(folder, 'msg', fname)) as f:
            fields, constants = _parse(f.read(), package)
        setattr(msgs, fname[:-4], _message(f'{package}/{fname[:-4]}', fields, constants))
    for fname in sorted(os.listdir(os.path.join(folder, 'srv'))):
        with open(os.path.join(folder, 'srv', fname)) as f:
            request, response = f.read().split('\n---\n')
        name = fname[:-4]
        srv = type(name, (), {})
        srv.Request = _message(f'{package}/{name}_Request', *_parse(request, package))
        srv.Response = _message(f'{package}/{name}_Response', *_parse(response, package))
        setattr(srvs, name, srv)
    return msgs, srvs


# rclpy --------------------------------------------------------------------------------------------------------


class ParameterType(enum.IntEnum):
    NOT_SET = 0
    BOOL = 1
    INTEGER = 2
    DOUBLE = 3
    STRING = 4
    BYTE_ARRAY = 5
    BOOL_ARRAY = 6
    INTEGER_ARRAY = 7
    DOUBLE_ARRAY = 8
    STRING_ARRAY = 9

    @classmethod
    def from_parameter_value(cls, value):
        if value is None:
            return cls.NOT_SET
        if isinstance(value, bool):
            return cls.BOOL
        if isinstance(value, int):
            return cls.INTEGER
        if isinstance(value, float):
            return cls.DOUBLE
        if isinstance(value, str):
            return cls.STRING
        if isinstance(value, (list, tuple)):
            # like rclpy: an empty list is taken for a byte array
            if all(isinstance(v, bytes) for v in value):
                return cls.BYTE_ARRAY
            if all(isinstance(v, bool) for v in value):
                return cls.BOOL_ARRAY
            if all(isinstance(v, int) for v in value):
                return cls.INTEGER_ARRAY
            if all(isinstance(v, float) for v in value):
                return cls.DOUBLE_ARRAY
            if all(isinstance(v, str) for v in value):
                return cls.STRING_ARRAY
        raise TypeError(f'no parameter type for {value!r}')


class Parameter:
    Type = ParameterType

    def __init__(self, name, type_=None, value=None):
        self.name = name
        self.type_ = ParameterType.from_parameter_value(value) if type_ is None else type_
        self.value = value

    @classmethod
    def from_parameter_msg(cls, msg):
        type_ = ParameterType(msg.value.type)
        value = None if type_ == ParameterType.NOT_SET else getattr(msg.value, type_.name.lower() + '_value')
        return cls(msg.name, type_, value)


class ParameterUninitializedException(Exception):
    pass


class InvalidParameterValueException(Exception):
    pass


class InvalidParameterTypeException(Exception):
    pass


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg):
        self.lines.append((level, msg))

    def debug(self, msg):
        self._log('debug', msg)

    def info(self, msg):
        self._log('info', msg)

    def warning(self, msg):
        self._log('warning', msg)

    warn = warning

    def error(self, msg):
        self._log('error', msg)


class _Publisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type, self.topic, self.qos = msg_type, topic, qos
        self.published = []

    def publish(self, msg):
        assert isinstance(msg, self.msg_type)
        self.published.append(msg)


class _Clock:
    def now(self):
        class _Now:
            def to_msg(self):
                return _REGISTRY['builtin_interfaces/Time'](sec=1700000000, nanosec=123)
        return _Now()


class Node:
    def __init__(self, node_name, **kwargs):
        self.name = node_name
        self._params = {}
        self._callbacks = []
        self.publishers = {}
        self.services = {}
        self._logger = _Logger()

    def add_on_set_parameters_callback(self, callback):
        self._callbacks.insert(0, callback)

    def _validate(self, param, descriptor):
        for cb in self._callbacks:
            result = cb([param])
            if not result.successful:
                return result.reason
        for rng in getattr(descriptor, 'floating_point_range', []) or []:
            if not rng.from_value <= param.value <= rng.to_value:
                return f'{param.name} out of range'
        for rng in getattr(descriptor, 'integer_range', []) or []:
            if not rng.from_value <= param.value <= rng.to_value:
                return f'{param.name} out of range'
        return None

    def declare_parameter(self, name, value=None, descriptor=None):
        if isinstance(value, ParameterType):
            param = Parameter(name, value, None)
        else:
            param = Parameter(name, value=value)
            reason = self._validate(param, descriptor)
            if reason:
                raise InvalidParameterValueException(reason)
        self._params[name] = (param, descriptor)
        return param

    def get_parameter(self, name):
        param = self._params[name][0]
        if param.value is None:
            raise ParameterUninitializedException(name)
        return param

    def get_parameter_or(self, name, alternative_value=None):
        if name not in self._params or self._params[name][0].value is None:
            return alternative_value
        return self._params[name][0]

    def set_parameters(self, params):
        results = []
        for p in params:
            old, descriptor = self._params[p.name]
            if p.type_ != old.type_:
                results.append(_REGISTRY['rcl_interfaces/SetParametersResult'](
                    successful=False, reason='Wrong parameter type'))
                continue
            reason = self._validate(p, descriptor)
            if reason is None:
                self._params[p.name] = (p, descriptor)
            results.append(_REGISTRY['rcl_interfaces/SetParametersResult'](
                successful=reason is None, reason=reason or ''))
        return results

    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        pub = _Publisher(msg_type, topic, qos_profile)
        self.publishers[topic] = pub
        return pub

    def create_service(self, srv_type, srv_name, callback, **kwargs):
        self.services[srv_name] = (srv_type, callback)

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


class _Policy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2
    KEEP_LAST = 3
    KEEP_ALL = 4
    TRANSIENT_LOCAL = 5
    VOLATILE = 6


class QoSProfile:
    def __init__(self, **kwargs):
        assert 'depth' in kwargs or 'history' in kwargs
        self.__dict__.update(kwargs)


def install():
    """Puts the stand-ins into sys.modules (only call it when the real ones can't be imported)."""
    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    time_cls = _message('builtin_interfaces/Time', {'sec': ('int32', None), 'nanosec': ('uint32', None)})
    header = _message('std_msgs/Header', {'stamp': ('builtin_interfaces/Time', None), 'frame_id': ('string', None)})
    point = _message('geometry_msgs/Point', {k: ('float64', None) for k in 'xyz'})
    point32 = _message('geometry_msgs/Point32', {k: ('float32', None) for k in 'xyz'})
    quat = _message('geometry_msgs/Quaternion', {'x': ('float64', None), 'y': ('float64', None),
                                                 'z': ('float64', None), 'w': ('float64', 1.0)})
    pose = _message('geometry_msgs/Pose', {'position': ('geometry_msgs/Point', None),
                                           'orientation': ('geometry_msgs/Quaternion', None)})
    pose_stamped = _message('geometry_msgs/PoseStamped', {'header': ('std_msgs/Header', None),
                                                          'pose': ('geometry_msgs/Pose', None)})
    polygon = _message('geometry_msgs/Polygon', {'points': (['geometry_msgs/Point32'], None)})
    path = _message('nav_msgs/Path', {'header': ('std_msgs/Header', None),
                                      'poses': (['geometry_msgs/PoseStamped'], None)})
    trigger = type('Trigger', (), {})
    trigger.Request = _message('std_srvs/Trigger_Request', {})
    trigger.Response = _message('std_srvs/Trigger_Response', {'success': ('bool', None), 'message': ('string', None)})

    class _Plain:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    descriptor = type('ParameterDescriptor', (_Plain,), {})
    fp_range = type('FloatingPointRange', (_Plain,), {})
    int_range = type('IntegerRange', (_Plain,), {})
    set_result = _message('rcl_interfaces/SetParametersResult', {'successful': ('bool', None),
                                                                 'reason': ('string', None)})
    param_value = _message('rcl_interfaces/ParameterValue', {
        'type': ('uint8', None), 'bool_value': ('bool', None), 'integer_value': ('int64', None),
        'double_value': ('float64', None), 'string_value': ('string', None), 'byte_array_value': (['byte'], None),
        'bool_array_value': (['bool'], None), 'integer_array_value': (['int64'], None),
        'double_array_value': (['float64'], None), 'string_array_value': (['string'], None)})
    param_msg = _message('rcl_interfaces/Parameter', {'name': ('string', None),
                                                      'value': ('rcl_interfaces/ParameterValue', None)})

    module('builtin_interfaces')
    module('builtin_interfaces.msg', Time=time_cls)
    module('std_msgs')
    module('std_msgs.msg', Header=header)
    module('geometry_msgs')
    module('geometry_msgs.msg', Point=point, Point32=point32, Quaternion=quat, Pose=pose, PoseStamped=pose_stamped,
           Polygon=polygon)
    module('nav_msgs')
    module('nav_msgs.msg', Path=path)
    module('std_srvs')
    module('std_srvs.srv', Trigger=trigger)
    module('rcl_interfaces')
    module('rcl_interfaces.msg', ParameterDescriptor=descriptor, FloatingPointRange=fp_range, IntegerRange=int_range,
           SetParametersResult=set_result, Parameter=param_msg, ParameterValue=param_value)
    msgs, srvs = _interfaces('mowbite_planner_interfaces', INTERFACES)
    module('mowbite_planner_interfaces')
    sys.modules['mowbite_planner_interfaces.msg'] = msgs
    sys.modules['mowbite_planner_interfaces.srv'] = srvs

    class ExternalShutdownException(Exception):
        pass

    rclpy = module('rclpy', init=lambda args=None: None, spin=lambda node: None, try_shutdown=lambda: None,
                   ok=lambda: True)
    rclpy.node = module('rclpy.node', Node=Node)
    rclpy.parameter = module('rclpy.parameter', Parameter=Parameter)
    rclpy.executors = module('rclpy.executors', ExternalShutdownException=ExternalShutdownException)
    rclpy.exceptions = module('rclpy.exceptions', ParameterUninitializedException=ParameterUninitializedException,
                              InvalidParameterValueException=InvalidParameterValueException,
                              InvalidParameterTypeException=InvalidParameterTypeException)
    rclpy.qos = module('rclpy.qos', QoSProfile=QoSProfile, DurabilityPolicy=_Policy, HistoryPolicy=_Policy,
                       ReliabilityPolicy=_Policy)
