#!/usr/bin/env python3

import math


def distance(first, second):
    """Return the Euclidean distance between two 3-D points."""
    return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))


def forward_progress_and_target(path, position, progress_index, lookahead):
    """Return reached progress and a forward look-ahead target on the path."""
    if not path:
        raise ValueError("path must not be empty")
    if lookahead <= 0.0 or not math.isfinite(lookahead):
        raise ValueError("lookahead must be finite and positive")

    last = len(path) - 1
    start = max(0, min(int(progress_index), last))
    nearest = min(
        range(start, len(path)),
        key=lambda index: distance(position, path[index]),
    )
    selected = last
    first_candidate = min(last, nearest + 1)
    for index in range(first_candidate, len(path)):
        if distance(position, path[index]) >= lookahead:
            selected = index
            break
    return nearest, max(nearest, selected)


def forward_target_index(path, position, progress_index, lookahead):
    """Select a look-ahead waypoint without ever moving backward in the path."""
    _, selected = forward_progress_and_target(
        path, position, progress_index, lookahead
    )
    return selected
