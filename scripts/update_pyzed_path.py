from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WHEELS_DIR = ROOT / "wheels"
PYPROJECT = ROOT / "pyproject.toml"

wheels = list(WHEELS_DIR.glob("pyzed-*.whl"))


if len(wheels) != 1:
    raise RuntimeError(f"Expected exactly one pyzed wheel in {WHEELS_DIR}, but found {len(wheels)}")

wheel_path = wheels[0].relative_to(ROOT).as_posix()

content = PYPROJECT.read_text()
lines = content.splitlines(keepends=True)

matches = [i for i, line in enumerate(lines) if line.lstrip().startswith("pyzed =")]

if len(matches) != 1:
    raise RuntimeError(
        f"Expected exactly one 'pyzed =' line in {PYPROJECT}, but found {len(matches)}. If 0 were found, you may need to \
            add a dummy pyzed = {{ path = \"pyzed.whl entry to \"}} line under [tool.pixi.feature.ros.pypi-dependencies]."
    )

index = matches[0]
ending = "\r\n" if lines[index].endswith("\r\n") else "\n"

indent = lines[index][: len(lines[index]) - len(lines[index].lstrip())]

lines[index] = f'{indent}pyzed = {{ path = "{wheel_path}" }}{ending}'

PYPROJECT.write_text("".join(lines))

print(f"Updated pyzed path to: {wheel_path}")
