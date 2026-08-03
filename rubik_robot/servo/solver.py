"""
Cube move translation and sequence optimization.

Translates standard Rubik's cube notation (U, R', F2, etc.) into
sequences of servo action codes that the robot can execute. Also
optimizes the resulting action strings by removing redundant
gripper and wrist movements.

Rubik's cube notation:
    U/U'/U2 - Up face (clockwise / counter-clockwise / 180 degrees)
    D/D'/D2 - Down face
    R/R'/R2 - Right face
    L/L'/L2 - Left face
    F/F'/F2 - Front face
    B/B'/B2 - Back face

The robot can only directly turn the D face (left arm) and R face
(right arm). All other faces require rotating the cube first to bring
the target face into position, then turning it.
"""

from rubik_robot.servo.moves import (
    LMp, LMm, LMpp, LTp, LTm, LTpp,
    RMp, RMm, RMpp, RTp, RTm, RTpp,
)


def correct_right(solve_array):
    """Apply a 90-degree clockwise cube rotation correction.

    When the cube is physically rotated 90 degrees around the right
    axis, the face labels need to be remapped so subsequent moves
    still refer to the correct physical faces.

    Remapping: U->B, F->U, D->F, B->D (right-hand rotation)

    IMPORTANT: This function mutates solve_array in place. It is called
    during move_cube() processing, and the mutation affects how subsequent
    moves in the same sequence are interpreted. This sequential dependency
    is intentional and must be preserved.

    Args:
        solve_array: List of move strings to remap (modified in place).
    """
    for i in range(len(solve_array)):
        solve_array[i] = solve_array[i].replace("U", "X")
        solve_array[i] = solve_array[i].replace("F", "U")
        solve_array[i] = solve_array[i].replace("D", "F")
        solve_array[i] = solve_array[i].replace("B", "D")
        solve_array[i] = solve_array[i].replace("X", "B")


def correct_left(solve_array):
    """Apply a 180-degree cube rotation correction.

    When the cube is physically rotated 180 degrees (flipped), the
    face labels need to be swapped: F<->B and L<->R.

    IMPORTANT: This function mutates solve_array in place.

    Args:
        solve_array: List of move strings to remap (modified in place).
    """
    for i in range(len(solve_array)):
        solve_array[i] = solve_array[i].replace("F", "X")
        solve_array[i] = solve_array[i].replace("B", "F")
        solve_array[i] = solve_array[i].replace("X", "B")

        solve_array[i] = solve_array[i].replace("L", "X")
        solve_array[i] = solve_array[i].replace("R", "L")
        solve_array[i] = solve_array[i].replace("X", "R")


def move_cube(action, solve_array, c180, regrip_before_r=False):
    """Translate a single Rubik's notation move into an action string.

    This function determines which macro moves are needed to execute
    the given cube move, and also applies the necessary orientation
    corrections to solve_array so that future moves are interpreted
    correctly.

    The robot's physical layout means:
    - D face moves are done directly by the left arm (no cube rotation needed)
    - R face moves are done directly by the right arm (no cube rotation needed)
    - All other faces require rotating the cube to bring them into
      the D or R position first

    Args:
        action: Rubik's notation move string (e.g., "U", "R'", "F2").
        solve_array: List of remaining moves (mutated by correct_*).
        c180: Whether the robot has 180-degree wrist capability.
        regrip_before_r: Whether to prepend a regrip before R/D moves
            (True for GPIO variant, False for PCA9685).

    Returns:
        Action string to execute, or "finished" if the move is not recognized.
    """
    regrip_prefix = "R" if regrip_before_r else ""

    if action == "U":
        a1 = RTpp(c180)
        a2 = LMp(c180)
        correct_right(solve_array)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "U2":
        a1 = RTpp(c180)
        a2 = LMpp(c180)
        correct_right(solve_array)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "U'":
        a1 = RTpp(c180)
        a2 = LMm(c180)
        correct_right(solve_array)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "R":
        a1 = RMp(c180)
        return regrip_prefix + a1

    elif action == "R2":
        a1 = RMpp(c180)
        return regrip_prefix + a1

    elif action == "R'":
        a1 = RMm(c180)
        return regrip_prefix + a1

    elif action == "L":
        a1 = LTpp(c180)
        a2 = RMp(c180)
        correct_left(solve_array)
        return a1 + "R" + a2

    elif action == "L2":
        a1 = LTpp(c180)
        a2 = RMpp(c180)
        correct_left(solve_array)
        return a1 + "R" + a2

    elif action == "L'":
        a1 = LTpp(c180)
        a2 = RMm(c180)
        correct_left(solve_array)
        return a1 + "R" + a2

    elif action == "F":
        a1 = RTm(c180)
        a2 = LMp(c180)
        correct_right(solve_array)
        correct_right(solve_array)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "F2":
        a1 = RTm(c180)
        a2 = LMpp(c180)
        correct_right(solve_array)
        correct_right(solve_array)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "F'":
        a1 = RTm(c180)
        a2 = LMm(c180)
        correct_right(solve_array)
        correct_right(solve_array)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "B":
        a1 = RTp(c180)
        a2 = LMp(c180)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "B2":
        a1 = RTp(c180)
        a2 = LMpp(c180)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "B'":
        a1 = RTp(c180)
        a2 = LMm(c180)
        correct_right(solve_array)
        return a1 + "R" + a2

    elif action == "D":
        a1 = LMp(c180)
        return regrip_prefix + a1

    elif action == "D2":
        a1 = LMpp(c180)
        return regrip_prefix + a1

    elif action == "D'":
        a1 = LMm(c180)
        return regrip_prefix + a1

    else:
        return "finished"


def create_master_string(solve_array, c180, regrip_before_r=False):
    """Convert a list of Rubik's notation moves into an optimized action string.

    First translates each move into action codes via move_cube(), then
    applies a series of string replacements to remove redundant servo
    movements (e.g., opening and immediately closing a gripper, or
    turning a wrist to a position and immediately turning back).

    IMPORTANT: solve_array is mutated during this process by correct_right()
    and correct_left() calls inside move_cube(). Pass a copy if you need
    to preserve the original.

    Args:
        solve_array: List of Rubik's notation moves (e.g., ["R", "U'", "F2"]).
            This list is mutated during processing.
        c180: Whether the robot has 180-degree wrist capability.
        regrip_before_r: Whether to prepend regrip before R/D moves.

    Returns:
        Optimized action string ready for execution.
    """
    sequence = ""

    for move in solve_array:
        action_str = move_cube(move, solve_array, c180, regrip_before_r)
        sequence += action_str

    # --- Optimization passes ---
    # Each replacement removes a redundant servo movement that would
    # waste time without affecting the cube state.

    # Remove redundant gripper turns at 180 degrees
    sequence = sequence.replace("MtaNAaMAbOBaNA", "MtbOBaNA")
    sequence = sequence.replace("XtbYBbXBaZAbYB", "XtaZAbYB")

    # Remove redundant break gripper turns at 180 degrees
    sequence = sequence.replace("OtaNAaMA", "OtaMA")
    sequence = sequence.replace("ZtbYBbXB", "ZtbXB")

    # Remove redundant gripper turning at 180 degrees
    sequence = sequence.replace("OtaMAbOBaNA", "OtbMBaNA")
    sequence = sequence.replace("ZtbXBaZAbYB", "ZtaXAbYB")

    # Remove redundant gripper close/open pairs
    sequence = sequence.replace("Aa", "")
    sequence = sequence.replace("Bb", "")

    # Remove redundant gripper turns at 90 degrees
    sequence = sequence.replace("MtaNM", "Mt")
    sequence = sequence.replace("XtbYX", "Xt")

    return sequence
