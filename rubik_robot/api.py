"""FastAPI application exposing the robot's hardware operations.

Endpoints are intentionally thin: they validate input, call a single
``Robot`` method (each of which is internally serialised by a lock), and return
JSON. The heavy lifting — solving, scrambles, training, pattern targets — is the
client's responsibility.

Build the app with :func:`create_app`, injecting a real or mock ``Robot``.
"""

from __future__ import annotations

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from . import __version__
from .config import Calibration
from .moves import VALID_MOVES
from .robot import Robot


# --------------------------------------------------------------------------- #
# Request / response models
# --------------------------------------------------------------------------- #
class MoveRequest(BaseModel):
    move: str = Field(..., description="A single move in Singmaster notation, e.g. \"U\", \"R'\", \"F2\".")


class MovesRequest(BaseModel):
    moves: list[str] = Field(..., description="A sequence of moves to execute.")
    reset_orientation: bool = Field(
        False,
        description="Reset cube-orientation tracking to home before executing "
        "(use for scrambles starting from a homed cube, not after a scan).",
    )


class ActionRequest(BaseModel):
    actions: str = Field(..., description="Raw single-action characters (AaBbMNOXYZRt). For calibration/debug.")


class CalibrationUpdate(BaseModel):
    left_grip_tune: int | None = None
    left_wrist_tune: int | None = None
    right_grip_tune: int | None = None
    right_wrist_tune: int | None = None
    load: int | None = None
    sleep: float | None = None
    regrip: int | None = None


class ScanResponse(BaseModel):
    facelets: str = Field(..., description="54-char cube state in kociemba order (URFDLB).")


def create_app(robot: Robot) -> FastAPI:
    app = FastAPI(
        title="Rubik Robot API",
        version=__version__,
        description="Hardware control surface for the two-arm Rubik's cube robot.",
    )

    @app.get("/health")
    def health() -> dict:
        return {
            "status": "ok",
            "version": __version__,
            "c180": robot.c180,
            "left_wrist": robot.l_pos,
            "right_wrist": robot.r_pos,
        }

    @app.post("/home")
    def home() -> dict:
        robot.home()
        return {"status": "homed"}

    @app.post("/grip")
    def grip() -> dict:
        robot.grip()
        return {"status": "gripped"}

    @app.post("/regrip")
    def regrip() -> dict:
        robot.regrip()
        return {"status": "regripped"}

    @app.post("/scan", response_model=ScanResponse)
    def scan() -> ScanResponse:
        return ScanResponse(facelets=robot.scan())

    @app.post("/move")
    def move(req: MoveRequest) -> dict:
        if req.move not in VALID_MOVES:
            raise HTTPException(status_code=422, detail=f"invalid move: {req.move!r}")
        actions = robot.move(req.move)
        return {"move": req.move, "actions": actions}

    @app.post("/moves")
    def moves(req: MovesRequest) -> dict:
        invalid = [m for m in req.moves if m not in VALID_MOVES]
        if invalid:
            raise HTTPException(status_code=422, detail=f"invalid moves: {invalid}")
        actions = robot.move_sequence(req.moves, reset_orientation=req.reset_orientation)
        return {"count": len(req.moves), "actions": actions}

    @app.post("/orientation/reset")
    def reset_orientation() -> dict:
        robot.reset_orientation()
        return {"status": "orientation reset"}

    @app.post("/action")
    def action(req: ActionRequest) -> dict:
        unknown = set(req.actions) - set("AaBbMNOXYZRt")
        if unknown:
            raise HTTPException(status_code=422, detail=f"unknown action chars: {sorted(unknown)}")
        with robot._lock:
            robot.run_actions(req.actions)
        return {"actions": req.actions}

    @app.get("/tune", response_model=Calibration)
    def get_tune() -> Calibration:
        return robot.cal

    @app.put("/tune", response_model=Calibration)
    def put_tune(update: CalibrationUpdate) -> Calibration:
        changes = {k: v for k, v in update.model_dump().items() if v is not None}
        if not changes:
            raise HTTPException(status_code=422, detail="no calibration fields provided")
        return robot.update_calibration(**changes)

    return app
