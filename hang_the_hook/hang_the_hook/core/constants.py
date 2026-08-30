# =============================================================================
#  Core Constants
#  General configuration shared across the hang_the_hook system.
# =============================================================================


# ── Simulation ───────────────────────────────────────────────────────────────
SIM_MODE = False


# ── Flight Configuration ─────────────────────────────────────────────────────
RTL_ALTITUDE   = 2.0   # Return-to-launch altitude (meters)
TAKEOFF_HEIGHT = 2.0   # Default take-off height   (meters)
MAX_ALTITUDE_REFERENCE_LOSS = 10 # Total of tries drone can take to verify altitude source
PWM_VALUE_CLOSE = 1000


# ── Image Handler ────────────────────────────────────────────────────────────
IMAGE_WIDTH      = 640
IMAGE_HEIGHT     = 480
IMAGE_COMPRESSED = False

# Image source per mode:
#   SIM  → "/down_camera"                   (tópico ROS do Gazebo)
#   REAL → "webcam"                         (câmera onboard via OpenCV)
#          "/mavros/camera/image_captured"  (tópico ROS do drone real)
SIM_IMAGE_SOURCE  = "/down_camera"
REAL_IMAGE_SOURCE = "webcam"

IMAGE_SOURCE = SIM_IMAGE_SOURCE if SIM_MODE else REAL_IMAGE_SOURCE


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
