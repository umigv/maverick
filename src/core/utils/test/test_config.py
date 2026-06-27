from dataclasses import dataclass, field
from pathlib import Path

import pytest
from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import Parameter
from utils.config import load


class Param:
    def __init__(self, value: object):
        self.value = value


class MockNode:
    """Minimal mock for rclpy.Node, mimicking its parameter behavior.

    declare_parameter accepts either a default value or a Parameter.Type. A value is stored as default while a type
        leaves the parameter as required with the given static type
    get_parameter raises ParameterUninitializedException when a parameter is required and not set.
    """

    def __init__(self, initial: dict[str, object] | None = None) -> None:
        self._store: dict[str, object] = dict(initial or {})
        self.declared: list[tuple[str, object | None]] = []

    def get_name(self) -> str:
        return "MockNode"

    def declare_parameter(self, key: str, value: object = None) -> None:
        self.declared.append((key, value))
        if key not in self._store and not isinstance(value, Parameter.Type):
            self._store[key] = value

    def get_parameter(self, key: str) -> Param:
        if key not in self._store:
            raise ParameterUninitializedException(key)
        return Param(self._store[key])


def test_load_required_param_success() -> None:
    @dataclass(frozen=True)
    class Config:
        rate: int

    node = MockNode(initial={"rate": 10})
    config = load(node, Config)

    assert ("rate", Parameter.Type.INTEGER) in node.declared
    assert config.rate == 10


def test_load_required_param_missing_raises() -> None:
    @dataclass(frozen=True)
    class Config:
        rate: int

    node = MockNode(initial={})
    with pytest.raises(ParameterUninitializedException):
        load(node, Config)


def test_load_default_is_used_if_missing() -> None:
    @dataclass(frozen=True)
    class Config:
        rate: int = 20

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.rate == 20


def test_load_default_overridden_if_present() -> None:
    @dataclass(frozen=True)
    class Config:
        rate: int = 20

    node = MockNode(initial={"rate": 7})
    config = load(node, Config)

    assert config.rate == 7


def test_load_default_factory() -> None:
    @dataclass(frozen=True)
    class Config:
        ids: list[int] = field(default_factory=lambda: [1, 2, 3])

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.ids == [1, 2, 3]


def test_load_nested_dataclass() -> None:
    @dataclass(frozen=True)
    class Inner:
        gain: float = 0.5
        name: str = "abc"

    @dataclass(frozen=True)
    class Config:
        inner: Inner
        rate: int = 10

    node = MockNode(
        initial={
            "inner.gain": 1.25,
            "rate": 42,
        }
    )

    config = load(node, Config)

    assert config.rate == 42
    assert config.inner.gain == 1.25
    assert config.inner.name == "abc"


def test_load_nested_default_instance() -> None:

    @dataclass(frozen=True)
    class Inner:
        gain: float = 0.5
        name: str = "abc"

    @dataclass(frozen=True)
    class Config:
        inner: Inner = Inner(gain=2.0)

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.inner.gain == 2.0
    assert config.inner.name == "abc"


def test_load_nested_default_instance_overridden_by_params() -> None:
    @dataclass(frozen=True)
    class Inner:
        gain: float = 0.5
        name: str = "abc"

    @dataclass(frozen=True)
    class Config:
        inner: Inner = Inner(gain=2.0)

    node = MockNode(initial={"inner.gain": 3.5})
    config = load(node, Config)

    assert config.inner.gain == 3.5
    assert config.inner.name == "abc"


def test_load_nested_default_factory_makes_required_leaf_optional() -> None:
    @dataclass(frozen=True)
    class InnerWithRequired:
        gain: float
        name: str = "abc"

    @dataclass(frozen=True)
    class Config:
        inner: InnerWithRequired = field(default_factory=lambda: InnerWithRequired(gain=1.5))

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.inner.gain == 1.5
    assert config.inner.name == "abc"


def test_load_nested_without_default_instance_keeps_leaf_required() -> None:
    @dataclass(frozen=True)
    class InnerWithRequired:
        gain: float
        name: str = "abc"

    @dataclass(frozen=True)
    class Config:
        inner: InnerWithRequired

    node = MockNode(initial={})
    with pytest.raises(ParameterUninitializedException):
        load(node, Config)


def test_load_nested_default_instance_recurses() -> None:
    @dataclass(frozen=True)
    class Inner:
        gain: float = 0.5
        name: str = "abc"

    @dataclass(frozen=True)
    class Middle:
        inner: Inner = Inner()
        scale: float = 1.0

    @dataclass(frozen=True)
    class Config:
        middle: Middle = Middle(inner=Inner(gain=4.0), scale=2.0)

    node = MockNode(initial={"middle.inner.name": "xyz"})
    config = load(node, Config)

    assert config.middle.scale == 2.0
    assert config.middle.inner.gain == 4.0
    assert config.middle.inner.name == "xyz"


def test_load_bytes() -> None:
    @dataclass(frozen=True)
    class Config:
        data: bytes

    node = MockNode(initial={"data": b"\x01\x02\x03"})
    config = load(node, Config)

    assert config.data == b"\x01\x02\x03"


def test_load_list() -> None:
    @dataclass(frozen=True)
    class Config:
        ids: list[int]

    node = MockNode(initial={"ids": [10, 20, 30]})
    config = load(node, Config)

    assert config.ids == [10, 20, 30]


def test_load_path_required() -> None:
    @dataclass(frozen=True)
    class Config:
        log_dir: Path

    node = MockNode(initial={"log_dir": "/tmp/logs"})
    config = load(node, Config)

    assert config.log_dir == Path("/tmp/logs")
    assert isinstance(config.log_dir, Path)


def test_load_path_default() -> None:
    @dataclass(frozen=True)
    class Config:
        log_dir: Path = Path("/var/log")

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.log_dir == Path("/var/log")
    assert isinstance(config.log_dir, Path)


def test_load_path_default_overridden() -> None:
    @dataclass(frozen=True)
    class Config:
        log_dir: Path = Path("/var/log")

    node = MockNode(initial={"log_dir": "/tmp/override"})
    config = load(node, Config)

    assert config.log_dir == Path("/tmp/override")


def test_load_unsupported_type_raises() -> None:
    @dataclass(frozen=True)
    class Config:
        value: tuple

    node = MockNode(initial={})
    with pytest.raises(TypeError, match=r"unsupported type"):
        load(node, Config)
