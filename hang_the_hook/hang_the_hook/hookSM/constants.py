# =============================================================================
#  Hook State-Machine Constants
#  Tuning parameters and outcomes for the hook-hanging pipeline.
# =============================================================================


# ── PID Gains – X Axis ──────────────────────────────────────────────────────
KP_X = 0.02
KI_X = 0.0
KD_X = 0.0

# ── PID Gains – Y Axis ──────────────────────────────────────────────────────
KP_Y = 0.02
KI_Y = 0.0
KD_Y = 0.0

# ── PID Gains – Yaw ─────────────────────────────────────────────────────────
KP_YAW = 0.02
KI_YAW = 0.0
KD_YAW = 0.0


# ── Timeouts & Tolerances ───────────────────────────────────────────────────
TIMEOUT_ALIGN      = 15.0   # seconds
CENTER_TOLERANCE   = 30     # pixels
ANGULAR_TOLERANCE  = 10     # degrees


# ── State-Machine Outcomes ───────────────────────────────────────────────────
FIND_HOSE = "find_hose"
DESCEND   = "descend"
ALIGN     = "align"


# ── Altitude & Movement ─────────────────────────────────────────────────────
MAX_ALT   = 5.5   # meters – ceiling for search ascent
ALT_INCR  = 0.5   # meters – step size when climbing
DROP_DIST = 1.8   # meters – forward distance before drop
SHIFT     = 0.0   # TODO: lateral offset compensation


# ── Servo / PWM ─────────────────────────────────────────────────────────────
AUX_OUT   = 0
PWM_VALUE = 0
