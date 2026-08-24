# =============================================================================
#  Core Constants
#  General configuration shared across the hang_the_hook system.
# =============================================================================


# ── Simulation ───────────────────────────────────────────────────────────────
SIM_MODE = False


# ── Flight Configuration ─────────────────────────────────────────────────────
RTL_ALTITUDE   = 2.0   # Return-to-launch altitude (meters)
TAKEOFF_HEIGHT = 2.0  # Default take-off height   (meters)


# ── Image Handler ────────────────────────────────────────────────────────────
IMAGE_WIDTH          = 640
IMAGE_HEIGHT         = 480
SIM_IMAGE_COMPRESSED = False

# Image source options:
#   "webcam"                        → in-built camera of personal machine
#   "/down_camera"                  → MAVRos topic for SIM drone camera
#   "/mavros/camera/image_captured" → real drone
IMAGE_SOURCE = "webcam"

FRAME_WIDTH  = 1280
FRAME_HEIGHT = 980


# ── PID – Eixo X (Lateral) ──────────────────────────────────────────────────
PID_X_KP              = 0.5
PID_X_KI              = 0.0
PID_X_KD              = 0.0001
PID_X_OUTPUT_LIMITS   = (-0.7, 0.7)
PID_X_INTEGRAL_LIMITS = (-0.5, 0.5)

# ── PID – Eixo Y (Longitudinal) ─────────────────────────────────────────────
PID_Y_KP              = 0.1
PID_Y_KI              = 0.0
PID_Y_KD              = 0.0001
PID_Y_OUTPUT_LIMITS   = (-0.7, 0.7)
PID_Y_INTEGRAL_LIMITS = (-0.5, 0.5)

# ── PID – Yaw (Ângulo) ──────────────────────────────────────────────────────
PID_YAW_KP              = 0.5
PID_YAW_KI              = 0.0
PID_YAW_KD              = 0.001
PID_YAW_OUTPUT_LIMITS   = (-0.65, 0.65)
PID_YAW_INTEGRAL_LIMITS = (-0.05, 0.05)
