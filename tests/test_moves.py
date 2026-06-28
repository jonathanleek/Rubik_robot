"""Parity tests: the refactored MoveTranslator must reproduce the original
firmware's move-string generation exactly.

The ``_reference_*`` functions below are a deliberately faithful, line-for-line
port of ``move_cube`` / ``correct_right`` / ``correct_left`` /
``create_master_string`` from the original ``rubik_robot_en_PCA9685.py``. We
fuzz random move sequences and assert the new code matches.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rubik_robot.moves import MoveTranslator, optimise  # noqa: E402

C180 = 1
ALL_MOVES = [f + m for f in "URFDLB" for m in ("", "2", "'")]


# --------------------------------------------------------------------------- #
# Faithful copy of the original logic (operates on a shared mutable list).
# --------------------------------------------------------------------------- #
def _LMp():
    return "OtaNA" if C180 == 1 else "aMANt"


def _LMm():
    return "MtaNA"


def _LMpp():
    return "aMAOtaNA" if C180 == 1 else "aMANaMANt"


def _LTp():
    return "bOBaNA" if C180 == 1 else "aMAbNB"


def _LTm():
    return "bMBaNA"


def _LTpp():
    return "aMAbOBaNA" if C180 == 1 else "aMAbNBaMAbNB"


def _RMp():
    return "ZtbYB" if C180 == 1 else "bXBYt"


def _RMm():
    return "XtbYB"


def _RMpp():
    return "bXBZtbYB" if C180 == 1 else "bXBYbXBYt"


def _RTp():
    return "aZAbYB" if C180 == 1 else "bXBaYA"


def _RTm():
    return "aXAbYB"


def _RTpp():
    return "bXBaZAbYB" if C180 == 1 else "bXBaYAbXBaYA"


def _correct_right(arr):
    for x in range(len(arr)):
        arr[x] = arr[x].replace("U", "X")
        arr[x] = arr[x].replace("F", "U")
        arr[x] = arr[x].replace("D", "F")
        arr[x] = arr[x].replace("B", "D")
        arr[x] = arr[x].replace("X", "B")


def _correct_left(arr):
    for x in range(len(arr)):
        arr[x] = arr[x].replace("F", "X")
        arr[x] = arr[x].replace("B", "F")
        arr[x] = arr[x].replace("X", "B")
        arr[x] = arr[x].replace("L", "X")
        arr[x] = arr[x].replace("R", "L")
        arr[x] = arr[x].replace("X", "R")


def _move_cube(action, arr):
    if action == "U":
        a1, a2 = _RTpp(), _LMp()
        _correct_right(arr); _correct_right(arr)
        return a1 + "R" + a2
    if action == "U2":
        a1, a2 = _RTpp(), _LMpp()
        _correct_right(arr); _correct_right(arr)
        return a1 + "R" + a2
    if action == "U'":
        a1, a2 = _RTpp(), _LMm()
        _correct_right(arr); _correct_right(arr)
        return a1 + "R" + a2
    if action == "R":
        return _RMp()
    if action == "R2":
        return _RMpp()
    if action == "R'":
        return _RMm()
    if action == "L":
        a1, a2 = _LTpp(), _RMp()
        _correct_left(arr)
        return a1 + "R" + a2
    if action == "L2":
        a1, a2 = _LTpp(), _RMpp()
        _correct_left(arr)
        return a1 + "R" + a2
    if action == "L'":
        a1, a2 = _LTpp(), _RMm()
        _correct_left(arr)
        return a1 + "R" + a2
    if action == "F":
        a1, a2 = _RTm(), _LMp()
        _correct_right(arr); _correct_right(arr); _correct_right(arr)
        return a1 + "R" + a2
    if action == "F2":
        a1, a2 = _RTm(), _LMpp()
        _correct_right(arr); _correct_right(arr); _correct_right(arr)
        return a1 + "R" + a2
    if action == "F'":
        a1, a2 = _RTm(), _LMm()
        _correct_right(arr); _correct_right(arr); _correct_right(arr)
        return a1 + "R" + a2
    if action == "B":
        a1, a2 = _RTp(), _LMp()
        _correct_right(arr)
        return a1 + "R" + a2
    if action == "B2":
        a1, a2 = _RTp(), _LMpp()
        _correct_right(arr)
        return a1 + "R" + a2
    if action == "B'":
        a1, a2 = _RTp(), _LMm()
        _correct_right(arr)
        return a1 + "R" + a2
    if action == "D":
        return _LMp()
    if action == "D2":
        return _LMpp()
    if action == "D'":
        return _LMm()
    raise ValueError(action)


def _reference_master_string(moves):
    arr = list(moves)
    seq = ""
    for y in range(len(arr)):
        seq += _move_cube(arr[y], arr)
    return optimise(seq)


def _reference_unoptimised(moves):
    arr = list(moves)
    seq = ""
    for y in range(len(arr)):
        seq += _move_cube(arr[y], arr)
    return seq


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #
def test_batch_matches_reference_fuzz():
    rng = random.Random(1234)
    for _ in range(2000):
        n = rng.randint(0, 25)
        moves = [rng.choice(ALL_MOVES) for _ in range(n)]
        expected = _reference_master_string(moves)
        got = MoveTranslator(c180=C180).translate_batch(moves)
        assert got == expected, f"mismatch for {moves}:\n exp={expected}\n got={got}"


def test_per_move_matches_unoptimised_batch():
    """Issuing moves one at a time (no cross-move optimisation) must equal the
    original's unoptimised concatenation, proving orientation tracking is
    correct across separate calls."""
    rng = random.Random(99)
    for _ in range(2000):
        n = rng.randint(0, 25)
        moves = [rng.choice(ALL_MOVES) for _ in range(n)]
        expected = _reference_unoptimised(moves)
        t = MoveTranslator(c180=C180)
        got = "".join(t.translate(m) for m in moves)
        assert got == expected, f"mismatch for {moves}:\n exp={expected}\n got={got}"


def test_every_single_move_from_home():
    for m in ALL_MOVES:
        expected = _reference_unoptimised([m])
        got = MoveTranslator(c180=C180).translate(m)
        assert got == expected, f"single move {m}: exp={expected} got={got}"


if __name__ == "__main__":
    test_batch_matches_reference_fuzz()
    test_per_move_matches_unoptimised_batch()
    test_every_single_move_from_home()
    print("all parity tests passed")
