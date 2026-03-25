from dataclasses import dataclass, field
from pathlib import Path

import pytest
from rcl_interfaces.msg import ParameterDescriptor
from utils.config import load


class Param:
    def __init__(self, value):
        self.value = value


class MockNode:
    def __init__(self, initial: dict[str, object] | None = None):
        self._store: dict[str, object] = dict(initial or {})
        self.declared: list[tuple[str, object | None]] = []

    def get_name(self) -> str:
        return "MockNode"

    def declare_parameter(self, key: str, default_value=None, descriptor: ParameterDescriptor | None = None):
        self.declared.append((key, default_value))
        if key not in self._store:
            self._store[key] = default_value

    def get_parameter(self, key: str) -> Param:
        return Param(self._store.get(key, None))


def test_load_required_param_success():
    @dataclass
    class Config:
        rate: int

    node = MockNode(initial={"rate": 10})
    config = load(node, Config)

    assert ("rate", None) in node.declared
    assert config.rate == 10


def test_load_required_param_missing_raises():
    @dataclass
    class Config:
        rate: int

    node = MockNode(initial={})
    with pytest.raises(RuntimeError, match=r"Required parameter 'rate' not set"):
        load(node, Config)


def test_load_default_is_used_if_missing():
    @dataclass
    class Config:
        rate: int = 20

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.rate == 20


def test_load_default_overridden_if_present():
    @dataclass
    class Config:
        rate: int = 20

    node = MockNode(initial={"rate": 7})
    config = load(node, Config)

    assert config.rate == 7


def test_load_default_factory():
    @dataclass
    class Config:
        ids: list[int] = field(default_factory=lambda: [1, 2, 3])

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.ids == [1, 2, 3]


@dataclass
class Inner:
    gain: float = 0.5
    name: str = "abc"


@dataclass
class Outer:
    inner: Inner
    rate: int = 10


def test_load_nested_dataclass():
    node = MockNode(
        initial={
            "inner.gain": 1.25,
            "rate": 42,
        }
    )

    config = load(node, Outer)

    assert config.rate == 42
    assert config.inner.gain == 1.25
    assert config.inner.name == "abc"


def test_load_bytes():
    @dataclass
    class Config:
        data: bytes

    node = MockNode(initial={"data": b"\x01\x02\x03"})
    config = load(node, Config)

    assert config.data == b"\x01\x02\x03"


def test_load_list():
    @dataclass
    class Config:
        ids: list[int]

    node = MockNode(initial={"ids": [10, 20, 30]})
    config = load(node, Config)

    assert config.ids == [10, 20, 30]


def test_load_path_required():
    @dataclass
    class Config:
        log_dir: Path

    node = MockNode(initial={"log_dir": "/tmp/logs"})
    config = load(node, Config)

    assert config.log_dir == Path("/tmp/logs")
    assert isinstance(config.log_dir, Path)


def test_load_path_default():
    @dataclass
    class Config:
        log_dir: Path = Path("/var/log")

    node = MockNode(initial={})
    config = load(node, Config)

    assert config.log_dir == Path("/var/log")
    assert isinstance(config.log_dir, Path)


def test_load_path_default_overridden():
    @dataclass
    class Config:
        log_dir: Path = Path("/var/log")

    node = MockNode(initial={"log_dir": "/tmp/override"})
    config = load(node, Config)

    assert config.log_dir == Path("/tmp/override")


def test_load_unsupported_type_raises():
    @dataclass
    class Config:
        value: tuple

    node = MockNode(initial={})
    with pytest.raises(TypeError, match=r"unsupported type"):
        load(node, Config)
