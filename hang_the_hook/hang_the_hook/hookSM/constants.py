# ============================================================================
#  Hook State-Machine Constants
# ============================================================================

# ── PID Gains – X Axis ──────────────────────────────────────────────────────
KP_X = 0.0014
KI_X = 0.0
KD_X = 0.0001

# ── PID Gains – Y Axis ──────────────────────────────────────────────────────
KP_Y = 0.0014
KI_Y = 0.0
KD_Y = 0.0001

# ── PID Gains – Yaw ─────────────────────────────────────────────────────────
KP_YAW = 0.002
KI_YAW = 0.0
KD_YAW = 0.0001

# ── Tolerances ──────────────────────────────────────────────────────────────
CENTER_TOLERANCE   = 30     # pixels
ANGULAR_TOLERANCE  = 10     # degrees

# ── State-Machine Outcomes ──────────────────────────────────────────────────
FIND_HOSE = 'find_hose'
DESCEND   = 'descend'
ALIGN     = 'align'
RTL       = 'return_to_launch'

# ── Altitude & Movement ─────────────────────────────────────────────────────
MAX_ALT   = 5.5                # meters  – ceiling for search ascent
ALT_INCR  = 0.2                # meters  – step size when climbing
DESCEND_STEP = 0.1             # meters  – height decreasing after each DESCEND iteration
DROP_DIST = 1.8                # meters  – forward distance before drop
SEARCHING_STEP_DISTANCE = 1.0  # meters  – searching cross arm lenght
ALIGNMENT_RETRY_PAUSE = 3.0    # seconds – time between each alignment retry iteration
SHIFT     = 0.0                # previously requiring no hook-camera compensation

# ── Servo / PWM ─────────────────────────────────────────────────────────────
AUX_OUT   = 7
PWM_VALUE = 1500

# ── Counters ────────────────────────────────────────────────────────────────
ALIGNMENT_MIN_FRAMES = 20
ALIGNMENT_MAX_LOSS = 20
ALIGNMENT_MAX_RETRIES = 5
ALIGNMENT_MAX_NONE_LINE_DETECTION = 10
DESCEND_MAX_NONE_LINE_DETECTION = 10
