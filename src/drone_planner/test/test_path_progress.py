import math
import pathlib
import sys


sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "scripts"))

from path_progress import forward_progress_and_target, forward_target_index


def test_progress_does_not_jump_to_a_spatially_near_older_segment():
    path = [
        (0.0, 0.0, 1.0),
        (2.0, 0.0, 1.0),
        (2.0, 1.0, 1.0),
        (0.1, 1.0, 1.0),
        (0.1, 0.05, 1.0),
        (3.0, 0.05, 1.0),
    ]

    selected = forward_target_index(path, (0.02, 0.02, 1.0), 3, 0.45)

    assert selected >= 3


def test_progress_is_monotonic_across_a_complete_folded_path():
    path = [
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.0),
        (1.0, 1.0, 1.0),
        (0.0, 1.0, 1.0),
        (0.0, 0.1, 1.0),
        (2.0, 0.1, 1.0),
    ]
    positions = [
        (0.0, 0.0, 1.0),
        (0.8, 0.0, 1.0),
        (1.0, 0.8, 1.0),
        (0.2, 1.0, 1.0),
        (0.0, 0.12, 1.0),
        (1.5, 0.1, 1.0),
    ]
    progress = 0
    history = []
    for position in positions:
        progress, target = forward_progress_and_target(
            path, position, progress, 0.45
        )
        history.append(progress)
        assert target >= progress

    assert history == sorted(history)
    assert history[-1] == len(path) - 1


def test_lookahead_target_is_not_counted_as_reached_progress():
    path = [
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.0),
        (2.0, 0.0, 1.0),
        (3.0, 0.0, 1.0),
    ]

    progress, target = forward_progress_and_target(
        path, (0.0, 0.0, 1.0), 0, 0.45
    )

    assert progress == 0
    assert target == 1


def test_invalid_arguments_are_rejected():
    try:
        forward_target_index([], (0.0, 0.0, 0.0), 0, 0.45)
        assert False, "empty path should fail"
    except ValueError:
        pass

    for lookahead in (0.0, -1.0, math.inf):
        try:
            forward_target_index([(0.0, 0.0, 0.0)], (0.0, 0.0, 0.0), 0, lookahead)
            assert False, "invalid lookahead should fail"
        except ValueError:
            pass
