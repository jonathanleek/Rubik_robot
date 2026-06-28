"""Translation of cube moves into low-level servo action strings.

This is a faithful, refactored port of the move logic in the original
``rubik_robot_en_PCA9685.py`` (``move_cube`` / ``create_master_string`` /
``correct_left`` / ``correct_right``). It is pure-Python and hardware-free.

Two facts about this robot drive the whole design:

1. The robot has two grippers on opposite faces. It can only physically turn a
   layer that one of its grippers currently holds, so most logical moves are
   implemented as "reorient the cube, then turn". Each macro below expands a
   cube move into a string of single-character servo actions.

2. Reorienting the cube changes where every *other* face is. The robot must
   therefore track the cube's current orientation and remap each incoming
   logical move accordingly. ``Orientation`` below holds that state so that
   moves sent one-at-a-time over the API compose exactly like a batch.

Single-action alphabet (consumed by ``Robot.run_actions``)::

    A  left gripper close        a  left gripper open
    B  right gripper close       b  right gripper open
    M  left wrist -> 0 deg       N  left wrist -> 90 deg     O  left wrist -> 180 deg
    X  right wrist -> 0 deg      Y  right wrist -> 90 deg    Z  right wrist -> 180 deg
    R  re-grip                   t  decrement remaining-move counter
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .config import C180

# --------------------------------------------------------------------------- #
# Macro move tables: logical wrist/gripper gymnastics -> action strings.
# (Names match the original: L/R arm, M=level wrist, T=cube wrist, p/m/pp.)
# --------------------------------------------------------------------------- #


def _l_level_plus(c180: int) -> str:
    return "OtaNA" if c180 else "aMANt"


def _l_level_minus(c180: int) -> str:
    return "MtaNA"


def _l_level_plusplus(c180: int) -> str:
    return "aMAOtaNA" if c180 else "aMANaMANt"


def _l_cube_plus(c180: int) -> str:
    return "bOBaNA" if c180 else "aMAbNB"


def _l_cube_minus(c180: int) -> str:
    return "bMBaNA"


def _l_cube_plusplus(c180: int) -> str:
    return "aMAbOBaNA" if c180 else "aMAbNBaMAbNB"


def _r_level_plus(c180: int) -> str:
    return "ZtbYB" if c180 else "bXBYt"


def _r_level_minus(c180: int) -> str:
    return "XtbYB"


def _r_level_plusplus(c180: int) -> str:
    return "bXBZtbYB" if c180 else "bXBYbXBYt"


def _r_cube_plus(c180: int) -> str:
    return "aZAbYB" if c180 else "bXBaYA"


def _r_cube_minus(c180: int) -> str:
    return "aXAbYB"


def _r_cube_plusplus(c180: int) -> str:
    return "bXBaZAbYB" if c180 else "bXBaYAbXBaYA"


# --------------------------------------------------------------------------- #
# Orientation tracking
# --------------------------------------------------------------------------- #

# A "right" correction is a roll about the R-L axis: U->B->D->F->U.
_RIGHT_MAP = {"U": "B", "F": "U", "D": "F", "B": "D", "L": "L", "R": "R"}
# A "left" correction is a 180 deg turn about the U-D axis: F<->B, L<->R.
_LEFT_MAP = {"F": "B", "B": "F", "L": "R", "R": "L", "U": "U", "D": "D"}


@dataclass
class Orientation:
    """Cumulative remap from a *logical* face letter to the *physical* face.

    Starts as the identity (cube sitting in its home frame). Each executed move
    composes its reorientation in, exactly mirroring how the original code
    mutated the remaining ``solve_array`` in place.
    """

    remap: dict = field(default_factory=lambda: {f: f for f in "URFDLB"})

    def physical(self, face: str) -> str:
        """Map a logical face letter to the face the robot must act on now."""
        return self.remap[face]

    def apply(self, op: str) -> None:
        table = _RIGHT_MAP if op == "right" else _LEFT_MAP
        self.remap = {logical: table[phys] for logical, phys in self.remap.items()}

    def reset(self) -> None:
        self.remap = {f: f for f in "URFDLB"}

    def copy(self) -> "Orientation":
        return Orientation(dict(self.remap))


# Per-physical-move expansion: face letter -> (builder, reorientation ops).
# The builder takes c180 and returns the action string; ``reorient`` lists the
# corrections to compose into the orientation after the move. Mirrors the
# branches of the original ``move_cube`` one-for-one.
def _expand(physical_move: str, c180: int):
    face = physical_move[0]
    mod = physical_move[1:]  # "", "2", or "'"

    if face == "U":
        a1 = _r_cube_plusplus(c180)
        a2 = {"": _l_level_plus, "2": _l_level_plusplus, "'": _l_level_minus}[mod](c180)
        return a1 + "R" + a2, ["right", "right"]
    if face == "R":
        a1 = {"": _r_level_plus, "2": _r_level_plusplus, "'": _r_level_minus}[mod](c180)
        return a1, []
    if face == "L":
        a1 = _l_cube_plusplus(c180)
        a2 = {"": _r_level_plus, "2": _r_level_plusplus, "'": _r_level_minus}[mod](c180)
        return a1 + "R" + a2, ["left"]
    if face == "F":
        a1 = _r_cube_minus(c180)
        a2 = {"": _l_level_plus, "2": _l_level_plusplus, "'": _l_level_minus}[mod](c180)
        return a1 + "R" + a2, ["right", "right", "right"]
    if face == "B":
        a1 = _r_cube_plus(c180)
        a2 = {"": _l_level_plus, "2": _l_level_plusplus, "'": _l_level_minus}[mod](c180)
        return a1 + "R" + a2, ["right"]
    if face == "D":
        a1 = {"": _l_level_plus, "2": _l_level_plusplus, "'": _l_level_minus}[mod](c180)
        return a1, []
    raise ValueError(f"unknown move: {physical_move!r}")


# Cancellation rules from the original ``create_master_string``: each removes a
# mechanically redundant pair of motions (a turn immediately undone, or a
# close/open that nets to nothing). Applied only when optimising a full batch.
_OPTIMISATIONS = [
    ("MtaNAaMAbOBaNA", "MtbOBaNA"),
    ("XtbYBbXBaZAbYB", "XtaZAbYB"),
    ("OtaNAaMA", "OtaMA"),
    ("ZtbYBbXB", "ZtbXB"),
    ("OtaMAbOBaNA", "OtbMBaNA"),
    ("ZtbXBaZAbYB", "ZtaXAbYB"),
    ("Aa", ""),
    ("Bb", ""),
    ("MtaNM", "Mt"),
    ("XtbYX", "Xt"),
]


def optimise(sequence: str) -> str:
    """Remove redundant gripper/wrist motions from a full action sequence."""
    for old, new in _OPTIMISATIONS:
        sequence = sequence.replace(old, new)
    return sequence


VALID_MOVES = {
    f + m for f in "URFDLB" for m in ("", "2", "'")
}


class MoveTranslator:
    """Stateful translator from logical cube moves to servo action strings.

    Holds the cube orientation so individual moves issued separately still
    compose correctly. Use :meth:`translate` for one move (keeps optimisation
    *off* across calls, which only costs a few redundant wiggles) or
    :meth:`translate_batch` for a whole sequence (byte-identical to the
    original ``create_master_string``).
    """

    def __init__(self, c180: int = C180, orientation: Orientation | None = None):
        self.c180 = c180
        self.orientation = orientation or Orientation()

    def reset_orientation(self) -> None:
        self.orientation.reset()

    def translate(self, move: str) -> str:
        """Translate a single logical move, advancing the orientation state."""
        if move not in VALID_MOVES:
            raise ValueError(f"invalid move {move!r}; expected one of {sorted(VALID_MOVES)}")
        physical = self.orientation.physical(move[0]) + move[1:]
        actions, reorient = _expand(physical, self.c180)
        for op in reorient:
            self.orientation.apply(op)
        return actions

    def translate_batch(self, moves: list[str], optimise_result: bool = True) -> str:
        """Translate a sequence of moves, advancing orientation as it goes.

        With ``optimise_result`` true this reproduces the original
        ``create_master_string`` output exactly.
        """
        sequence = "".join(self.translate(m) for m in moves)
        return optimise(sequence) if optimise_result else sequence
