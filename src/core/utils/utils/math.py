def clamp(value: float, *, min: float, max: float) -> float:  # noqa: A002
    """Clamp `value` into the closed interval [min, max].

    Args:
        value: Input value.
        min: Lower bound (keyword-only).
        max: Upper bound (keyword-only).

    Returns:
        `min` if value < min, `max` if value > max, otherwise value.
    """
    if value < min:
        return min
    if value > max:
        return max
    return value
