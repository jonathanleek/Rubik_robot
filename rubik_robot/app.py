"""
Flask API for the Rubik Robot.

This module defines the REST API that external systems use to control
the robot. The API allows scanning the cube, executing moves, scrambling,
and managing calibration -- all over HTTP.

Concurrency: A hardware lock prevents multiple operations from running
simultaneously. If the robot is busy (e.g., scanning), requests that
need hardware access return HTTP 409 (Conflict) with the current status.
The GET /status endpoint never blocks and always returns immediately.

All responses are JSON. Successful operations return HTTP 200.
Errors return appropriate HTTP status codes with an "error" field.
"""

from flask import Flask, request, jsonify

from rubik_robot.robot import RobotController


def create_app(driver_type="pca9685"):
    """Create and configure the Flask application.

    Sets up the RobotController and registers all API routes. The
    controller is initialized immediately, which includes setting up
    hardware, so this function should only be called on the target Pi.

    Args:
        driver_type: "gpio" or "pca9685" -- selects the hardware driver.

    Returns:
        Configured Flask application ready to serve requests.
    """
    app = Flask(__name__)

    # Initialize the robot controller
    robot = RobotController(driver_type=driver_type)
    robot.initialize()

    # -------------------------------------------------------------------
    # Helper: acquire lock or return 409
    # -------------------------------------------------------------------

    def with_lock(operation_name):
        """Try to acquire the hardware lock.

        Args:
            operation_name: Name of the operation (for error messages).

        Returns:
            None if lock was acquired, or a (response, status_code) tuple
            if the robot is busy.
        """
        if not robot.acquire_lock():
            return jsonify({
                "error": f"Robot is busy (current status: {robot.status}). "
                         f"Cannot start: {operation_name}.",
                "status": robot.status,
            }), 409
        return None

    # -------------------------------------------------------------------
    # API Routes
    # -------------------------------------------------------------------

    @app.route("/scan", methods=["GET"])
    def scan():
        """Scan the cube and return its current state.

        Photographs all six faces, analyzes sticker colors, and returns
        a 54-character state string in Kociemba URFDLB notation.

        Returns:
            200: {"state": "UURR...", "time": 12.5}
            409: {"error": "Robot is busy...", "status": "moving"}
            500: {"error": "Scan failed: ..."}
        """
        busy = with_lock("scan")
        if busy:
            return busy
        try:
            result = robot.scan()
            return jsonify(result)
        except Exception as e:
            return jsonify({"error": f"Scan failed: {str(e)}"}), 500
        finally:
            robot.release_lock()

    @app.route("/move", methods=["POST"])
    def move():
        """Execute a list of Rubik's notation moves.

        Request body (JSON):
            {"moves": ["R", "U'", "F2"]}

        Each move must be a standard Rubik's notation string:
        U, U', U2, D, D', D2, R, R', R2, L, L', L2, F, F', F2, B, B', B2

        Returns:
            200: {"moves_executed": 3, "time": 8.2}
            400: {"error": "Missing 'moves' field..."}
            409: {"error": "Robot is busy..."}
            500: {"error": "Move failed: ..."}
        """
        data = request.get_json()
        if not data or "moves" not in data:
            return jsonify({
                "error": "Missing 'moves' field. "
                         "Expected: {\"moves\": [\"R\", \"U'\", \"F2\"]}"
            }), 400

        moves = data["moves"]
        if not isinstance(moves, list) or len(moves) == 0:
            return jsonify({
                "error": "'moves' must be a non-empty list of move strings."
            }), 400

        # Validate each move
        valid_moves = {
            "U", "U'", "U2", "D", "D'", "D2",
            "R", "R'", "R2", "L", "L'", "L2",
            "F", "F'", "F2", "B", "B'", "B2",
        }
        invalid = [m for m in moves if m not in valid_moves]
        if invalid:
            return jsonify({
                "error": f"Invalid moves: {invalid}. "
                         f"Valid moves are: {sorted(valid_moves)}"
            }), 400

        busy = with_lock("move")
        if busy:
            return busy
        try:
            result = robot.execute_moves(moves)
            return jsonify(result)
        except Exception as e:
            return jsonify({"error": f"Move failed: {str(e)}"}), 500
        finally:
            robot.release_lock()

    @app.route("/execute", methods=["POST"])
    def execute():
        """Execute a raw servo action string.

        This is a low-level endpoint for direct servo control. The
        action string is a sequence of single-character action codes
        (see servo/moves.py for the code reference).

        Request body (JSON):
            {"actions": "bXBaYA"}

        Returns:
            200: {"actions_executed": 6, "time": 3.1}
            400: {"error": "Missing 'actions' field..."}
            409: {"error": "Robot is busy..."}
            500: {"error": "Execute failed: ..."}
        """
        data = request.get_json()
        if not data or "actions" not in data:
            return jsonify({
                "error": "Missing 'actions' field. "
                         "Expected: {\"actions\": \"bXBaYA\"}"
            }), 400

        actions = data["actions"]
        if not isinstance(actions, str) or len(actions) == 0:
            return jsonify({
                "error": "'actions' must be a non-empty string of action codes."
            }), 400

        # Validate action codes
        valid_actions = set("AaBbMNOXYZRt")
        invalid = [c for c in actions if c not in valid_actions]
        if invalid:
            return jsonify({
                "error": f"Invalid action codes: {set(invalid)}. "
                         f"Valid codes: {sorted(valid_actions)}"
            }), 400

        busy = with_lock("execute")
        if busy:
            return busy
        try:
            import time
            start = time.time()

            from rubik_robot.servo.moves import single_action, regrip
            robot.status = "moving"
            robot.display.show("Execute", f"{len(actions)} actions")

            regrip(robot.driver, robot.config, robot.calibration)
            for action in actions:
                single_action(action, robot.driver, robot.config,
                              robot.calibration, robot.servo_state)

            from rubik_robot.servo.moves import home_servos
            home_servos(robot.driver, robot.config, robot.calibration,
                        robot.servo_state)

            elapsed = round(time.time() - start, 2)
            robot.status = "idle"
            robot.display.show("Done", "")

            return jsonify({
                "actions_executed": len(actions),
                "time": elapsed,
            })
        except Exception as e:
            return jsonify({"error": f"Execute failed: {str(e)}"}), 500
        finally:
            robot.release_lock()

    @app.route("/home", methods=["POST"])
    def home():
        """Return all servos to their home (neutral) position.

        Returns:
            200: {"status": "homed"}
            409: {"error": "Robot is busy..."}
        """
        busy = with_lock("home")
        if busy:
            return busy
        try:
            result = robot.home()
            return jsonify(result)
        except Exception as e:
            return jsonify({"error": f"Home failed: {str(e)}"}), 500
        finally:
            robot.release_lock()

    @app.route("/scramble", methods=["POST"])
    def scramble():
        """Scramble the cube with random moves.

        Request body (JSON):
            {"count": 20}

        The count is clamped to the range [1, SCRAMBLE_MAX].

        Returns:
            200: {"moves": ["R", "U'", ...], "count": 20, "time": 45.3}
            400: {"error": "Missing 'count' field..."}
            409: {"error": "Robot is busy..."}
            500: {"error": "Scramble failed: ..."}
        """
        data = request.get_json()
        if not data or "count" not in data:
            return jsonify({
                "error": "Missing 'count' field. "
                         "Expected: {\"count\": 20}"
            }), 400

        try:
            count = int(data["count"])
        except (ValueError, TypeError):
            return jsonify({
                "error": "'count' must be an integer."
            }), 400

        busy = with_lock("scramble")
        if busy:
            return busy
        try:
            result = robot.scramble(count)
            return jsonify(result)
        except Exception as e:
            return jsonify({"error": f"Scramble failed: {str(e)}"}), 500
        finally:
            robot.release_lock()

    @app.route("/status", methods=["GET"])
    def status():
        """Get the current robot status.

        This endpoint never blocks -- it returns immediately even if
        the robot is busy with another operation.

        Returns:
            200: {
                "status": "idle",
                "driver": "pca9685",
                "calibration": { ... }
            }
        """
        return jsonify(robot.get_status())

    @app.route("/calibration", methods=["GET"])
    def get_calibration():
        """Get current servo calibration values.

        Returns:
            200: {
                "left_grip_tune": 4,
                "left_wrist_tune": 2,
                "right_grip_tune": 6,
                "right_wrist_tune": 0,
                "load": 30,
                "sleep": 0.45,
                "regrip_enabled": true
            }
        """
        return jsonify(robot.get_calibration())

    @app.route("/calibration", methods=["POST"])
    def set_calibration():
        """Update servo calibration values and save to disk.

        Only include the fields you want to change -- omitted fields
        keep their current values.

        Request body (JSON):
            {"left_grip_tune": 4, "sleep": 0.45}

        Returns:
            200: {full updated calibration values}
            400: {"error": "No JSON body provided"}
            409: {"error": "Robot is busy..."}
        """
        data = request.get_json()
        if not data:
            return jsonify({"error": "No JSON body provided."}), 400

        busy = with_lock("calibration")
        if busy:
            return busy
        try:
            result = robot.set_calibration(data)
            return jsonify(result)
        except Exception as e:
            return jsonify({"error": f"Calibration failed: {str(e)}"}), 500
        finally:
            robot.release_lock()

    @app.route("/calibration/test", methods=["POST"])
    def test_servo():
        """Move a single servo to a specific position for testing.

        Useful during remote calibration to test individual servo
        positions without going through the full calibration sequence.

        Request body (JSON):
            {"servo": "left_grip", "value": 10}

        Valid servo names: left_grip, left_turn, right_grip, right_turn

        Returns:
            200: {"servo": "left_grip", "value": 10}
            400: {"error": "Missing fields..."}
            409: {"error": "Robot is busy..."}
            500: {"error": "Test failed: ..."}
        """
        data = request.get_json()
        if not data or "servo" not in data or "value" not in data:
            return jsonify({
                "error": "Missing 'servo' and/or 'value' fields. "
                         "Expected: {\"servo\": \"left_grip\", \"value\": 10}"
            }), 400

        try:
            value = int(data["value"])
        except (ValueError, TypeError):
            return jsonify({"error": "'value' must be an integer."}), 400

        busy = with_lock("servo test")
        if busy:
            return busy
        try:
            result = robot.test_servo(data["servo"], value)
            return jsonify(result)
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        except Exception as e:
            return jsonify({"error": f"Test failed: {str(e)}"}), 500
        finally:
            robot.release_lock()

    return app
