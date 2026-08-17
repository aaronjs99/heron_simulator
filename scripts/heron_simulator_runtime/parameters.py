"""Validate shared Heron simulator runtime parameter types."""

from __future__ import annotations


def strict_bool(value, *, name="value"):
    """Return a real boolean and reject strings or numeric stand-ins."""

    if isinstance(value, bool):
        return value
    raise ValueError("{} must be a boolean, got {!r}".format(name, value))
