"""
Cube scanning choreography.

Controls the physical process of rotating the cube to photograph all
six faces. The robot holds the cube and rotates it through a specific
sequence of positions, taking a photograph at each position.

The scanning sequence captures the faces in this order:
    face1.jpg = R (right)
    face2.jpg = F (front)
    face4.jpg = L (left)
    face5.jpg = B (back)
    face3.jpg = D (down)
    face0.jpg = U (up)

The cube is rotated using the right arm's turn mechanism (RTp) and the
left arm's turn mechanism (LTm), with regrips between each capture to
ensure the cube is properly seated.
"""

from rubik_robot.config import HOME
from rubik_robot.servo.moves import single_action, regrip


def get_cube(driver, config, cal, state, camera):
    """Scan all six faces of the cube by rotating and photographing.

    The scanning sequence uses a combination of right-arm and left-arm
    rotations to expose each face to the camera. Between each rotation,
    a regrip ensures the cube is properly held.

    The sequence for a 180-degree capable robot (c180=True) differs
    slightly in the final rotation to photograph the U face.

    Args:
        driver: ServoDriver instance for controlling servos.
        config: HardwareConfig with servo parameters.
        cal: CalibrationValues with tune offsets.
        state: ServoState tracking current positions.
        camera: Camera instance for capturing images.
    """
    regrip(driver, config, cal)

    # Face 1 (R face) - starting position
    camera.capture(HOME + "Cube/face1.jpg")

    # Rotate cube 90 degrees via right arm turn (RTp equivalent)
    single_action("b", driver, config, cal, state)
    single_action("X", driver, config, cal, state)
    single_action("B", driver, config, cal, state)
    single_action("a", driver, config, cal, state)
    single_action("Y", driver, config, cal, state)
    single_action("A", driver, config, cal, state)

    regrip(driver, config, cal)

    # Face 2 (F face)
    camera.capture(HOME + "Cube/face2.jpg")

    # Rotate cube 90 degrees again
    single_action("b", driver, config, cal, state)
    single_action("X", driver, config, cal, state)
    single_action("B", driver, config, cal, state)
    single_action("a", driver, config, cal, state)
    single_action("Y", driver, config, cal, state)
    single_action("A", driver, config, cal, state)

    regrip(driver, config, cal)

    # Face 4 (L face)
    camera.capture(HOME + "Cube/face4.jpg")

    # Rotate cube 90 degrees again
    single_action("b", driver, config, cal, state)
    single_action("X", driver, config, cal, state)
    single_action("B", driver, config, cal, state)
    single_action("a", driver, config, cal, state)
    single_action("Y", driver, config, cal, state)
    single_action("A", driver, config, cal, state)

    regrip(driver, config, cal)

    # Face 5 (B face)
    camera.capture(HOME + "Cube/face5.jpg")

    # Rotate cube via left arm turn (LTm equivalent) to expose bottom
    single_action("b", driver, config, cal, state)
    single_action("M", driver, config, cal, state)
    single_action("B", driver, config, cal, state)
    single_action("a", driver, config, cal, state)
    single_action("N", driver, config, cal, state)
    single_action("A", driver, config, cal, state)

    regrip(driver, config, cal)

    # Rotate cube 90 degrees via right arm
    single_action("b", driver, config, cal, state)
    single_action("X", driver, config, cal, state)
    single_action("B", driver, config, cal, state)
    single_action("a", driver, config, cal, state)
    single_action("Y", driver, config, cal, state)
    single_action("A", driver, config, cal, state)

    regrip(driver, config, cal)

    # Face 3 (D face)
    camera.capture(HOME + "Cube/face3.jpg")

    # Rotate 180 degrees to expose the top (U) face as TWO 90-degree
    # rotations with a regrip between, rather than a single 180-degree flip
    # -- even on c180 hardware. The single flip holds the cube solo on one
    # wrist through a full 180-degree arc, so any error in how the cube is
    # centered on the wrist axis is doubled and lands the U face skewed,
    # corrupting the scan (the U face was the consistent scan failure). Two
    # 90-degree reorientations keep each arc small and the intermediate
    # regrip re-centers the cube. Both paths net the same 180-degree
    # orientation, so the color-order correction in color.py is unchanged.
    single_action("b", driver, config, cal, state)
    single_action("X", driver, config, cal, state)
    single_action("B", driver, config, cal, state)
    single_action("a", driver, config, cal, state)
    single_action("Y", driver, config, cal, state)
    single_action("A", driver, config, cal, state)

    regrip(driver, config, cal)

    single_action("b", driver, config, cal, state)
    single_action("X", driver, config, cal, state)
    single_action("B", driver, config, cal, state)
    single_action("a", driver, config, cal, state)
    single_action("Y", driver, config, cal, state)
    single_action("A", driver, config, cal, state)

    regrip(driver, config, cal)

    # Face 0 (U face)
    camera.capture(HOME + "Cube/face0.jpg")
