"""
utils.config

Dataclass-backed ROS 2 parameter loading.

`load(node, cls)` constructs a dataclass instance by reading ROS 2 parameters from `node`.

Mapping rules:
- Each dataclass field name corresponds to a ROS 2 parameter key.
- If the field has a default value (or default_factory), the parameter is optional and
  the default is used when the key is not supplied in YAML.
- If the field has no default, the parameter is required; `load()` will raise if it is missing / unset.
- Nested dataclasses are supported and map to nested parameter dictionaries.

Supported field types:
- Primitives: `bool`, `int`, `float`, `str`, `bytes`
- Arrays: `list[bool]`, `list[int]`, `list[float]`, `list[str]`
- `pathlib.Path` (declared as a string parameter, coerced to `Path` on load)
- `typing.Literal[...]` of values sharing one supported primitive type; validated against the allowed set

For example, you can create the following config dataclass:
```py
from dataclasses import dataclass
from rclpy.node import Node
import utils.config

@dataclass
class Weights:
    clearance: float  # required
    heading: float = 1.0

@dataclass
class PlannerConfig:
    weights: Weights
    timeout_s: float       # required
    max_iters: int = 10_000
```

Which would map to a ROS2 config yaml structure like this:
```yaml
planner:
  ros__parameters:
    # max_iters is optional (defaults to 10000)
    timeout_s: 3.0            # required
    weights:
      # heading is optional (defaults to 1.0)
      clearance: 0.75         # required
```

You can then load it in code by calling utils.config.load
```py
class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.config = utils.config.load(self, PlannerConfig)
        print(self.config.max_iters)
        print(self.config.timeout_s)
        print(self.config.weights.clearance)
        print(self.config.weights.heading)
```
"""

import sys
from collections.abc import Callable
from dataclasses import MISSING, dataclass, fields, is_dataclass
from pathlib import Path
from typing import Any, Generic, Literal, TypeVar, cast, get_args, get_origin, get_type_hints

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter

T = TypeVar("T")
RawT = TypeVar("RawT")
FieldT = TypeVar("FieldT")


def _identity(x: T) -> T:
    return x


@dataclass
class _Entry(Generic[RawT, FieldT]):
    raw_type: type[RawT]
    ros_type: Parameter.Type
    deserialize: Callable[[RawT], FieldT]  # raw -> field (after reading from ROS)
    serialize: Callable[[FieldT], RawT]  # field -> raw (when declaring with a default)


_ENTRIES: dict[type, _Entry[Any, Any]] = {
    bool: _Entry(bool, Parameter.Type.BOOL, _identity, _identity),
    int: _Entry(int, Parameter.Type.INTEGER, _identity, _identity),
    float: _Entry(float, Parameter.Type.DOUBLE, _identity, _identity),
    str: _Entry(str, Parameter.Type.STRING, _identity, _identity),
    bytes: _Entry(bytes, Parameter.Type.BYTE_ARRAY, _identity, _identity),
    list[bool]: _Entry(list[bool], Parameter.Type.BOOL_ARRAY, _identity, _identity),
    list[int]: _Entry(list[int], Parameter.Type.INTEGER_ARRAY, _identity, _identity),
    list[float]: _Entry(list[float], Parameter.Type.DOUBLE_ARRAY, _identity, _identity),
    list[str]: _Entry(list[str], Parameter.Type.STRING_ARRAY, _identity, _identity),
    Path: _Entry(str, Parameter.Type.STRING, Path, str),
}


def _literal_entry(field_type: Any, key: str) -> _Entry[Any, Any]:
    """Build a per-field entry for a `Literal[...]` whose values share a supported primitive type."""
    allowed = get_args(field_type)
    if not allowed:
        raise TypeError(f"Parameter '{key}' Literal must have at least one value")
    base_type = type(allowed[0])
    if not all(type(a) is base_type for a in allowed):
        raise TypeError(f"Parameter '{key}' Literal values must all be the same type; got {allowed!r}")
    base = _ENTRIES.get(base_type)
    if base is None:
        raise TypeError(f"Parameter '{key}' Literal has unsupported value type {base_type.__name__!r}")

    def deserialize(v: Any) -> Any:
        field_value = base.deserialize(v)
        if field_value not in allowed:
            raise RuntimeError(f"Parameter '{key}' must be one of {list(allowed)!r}, got {field_value!r}")
        return field_value

    return _Entry(base.raw_type, base.ros_type, deserialize, base.serialize)


def load(node: Node, cls: type[T], _prefix: str = "") -> T:
    """
    Load ROS 2 parameters from `node` into a dataclass instance of type `cls`.
    Args:
        node: ROS 2 node providing parameters.
        cls: Dataclass type to construct.
    Returns:
        An instance of `cls` populated from ROS 2 parameters.
    Raises:
        RuntimeError: If a required parameter (field without a default) is missing or unset.
    """
    if not is_dataclass(cls):
        raise TypeError(f"{cls!r} is not a dataclass type")

    try:
        frame = sys._getframe(1)
        type_hints = get_type_hints(cls, globalns=frame.f_globals, localns=frame.f_locals)
    except (NameError, AttributeError, ValueError):
        try:
            type_hints = get_type_hints(cls)
        except Exception:
            type_hints = {}

    kwargs: dict[str, Any] = {}

    for f in fields(cls):
        key = f"{_prefix}{f.name}"
        field_type = type_hints.get(f.name, f.type)
        try:
            if is_dataclass(field_type):
                kwargs[f.name] = load(node, cast(type, field_type), _prefix=f"{key}.")
                continue
        except (TypeError, AttributeError):
            pass

        entry = _literal_entry(field_type, key) if get_origin(field_type) is Literal else _ENTRIES.get(field_type)

        if entry is None:
            raise TypeError(
                f"Parameter '{key}' has unsupported type {field_type!r}. "
                f"Supported types: {', '.join(t.__name__ for t in _ENTRIES)}"
            )

        # fmt: off
        default_value = (
            f.default if f.default is not MISSING
            else f.default_factory() if f.default_factory is not MISSING
            else MISSING
        )
        # fmt: on

        if default_value is MISSING:
            node.declare_parameter(key, descriptor=ParameterDescriptor(type=entry.ros_type.value))
            value = node.get_parameter(key).value
            if value is None:
                raise RuntimeError(f"Required parameter '{key}' not set for node '{node.get_name()}'")
            kwargs[f.name] = entry.deserialize(value)
        else:
            node.declare_parameter(key, entry.serialize(default_value))
            kwargs[f.name] = entry.deserialize(node.get_parameter(key).value)

    return cls(**kwargs)
