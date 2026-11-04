"""The planner's part of MowBite OS' app interface (api/planner.schema.json, AM6) and a check of a message against a
schema of it: the parts of JSON Schema it uses (type, enum, const, properties, required, additionalProperties false,
items, minimum), jsonschema not needed (MowBite OS' stack may not have it)."""

import json
import os

SCHEMA = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'api', 'planner.schema.json')
TYPES = {'object': dict, 'array': list, 'string': str, 'boolean': bool, 'null': type(None)}
USED = {'type', 'enum', 'const', 'properties', 'required', 'additionalProperties', 'items', 'minimum', 'description',
        'x-unit'}


def part() -> dict:
    with open(SCHEMA) as f:
        return json.load(f)


def _is(value, kind: str) -> bool:
    if kind == 'integer':
        return isinstance(value, int) and not isinstance(value, bool)
    if kind == 'number':
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    return isinstance(value, TYPES[kind])


def problems(value, schema: dict, at: str = '') -> list:
    """What of value doesn't keep to schema, where ([] none)."""
    unknown = set(schema) - USED
    if unknown:
        return [f'{at or "/"}: the check knows no {sorted(unknown)}']
    out = []
    kinds = schema.get('type')
    if kinds is not None and not any(_is(value, k) for k in ([kinds] if isinstance(kinds, str) else kinds)):
        return [f'{at or "/"}: {value!r} is no {kinds}']
    if 'enum' in schema and value not in schema['enum']:
        out.append(f'{at or "/"}: {value!r} not one of {schema["enum"]}')
    if 'const' in schema and value != schema['const']:
        out.append(f'{at or "/"}: {value!r} is not {schema["const"]!r}')
    if 'minimum' in schema and _is(value, 'number') and value < schema['minimum']:
        out.append(f'{at or "/"}: {value!r} below {schema["minimum"]}')
    if isinstance(value, dict):
        props = schema.get('properties', {})
        out += [f'{at}/{k}: missing' for k in schema.get('required', []) if k not in value]
        if schema.get('additionalProperties') is False:
            out += [f'{at}/{k}: not in the schema' for k in value if k not in props]
        for k, v in value.items():
            if k in props:
                out += problems(v, props[k], f'{at}/{k}')
    if isinstance(value, list) and 'items' in schema:
        for i, v in enumerate(value):
            out += problems(v, schema['items'], f'{at}/{i}')
    return out


def sent(topic: str, payload: str) -> list:
    """What of a message the planner sent on bridge/publish doesn't keep to its part ([] none)."""
    topics = part()['topics']
    if topic not in topics:
        return [f'{topic}: no topic of the planner\'s part']
    try:
        value = json.loads(payload)
    except ValueError as e:
        return [f'{topic}: not JSON: {e}']
    return [f'{topic}{p}' for p in problems(value, topics[topic])]
