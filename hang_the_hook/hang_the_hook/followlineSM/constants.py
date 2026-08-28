# =============================================================================
#  Follow-Line State-Machine Constants
#  Tuning parameters and outcomes for the line-following pipeline.
# =============================================================================


# ── PID Gains – Angle ────────────────────────────────────────────────────────
ANGLE_KP = 0.013
ANGLE_KI = 0.0
ANGLE_KD = 0.001

# ── PID Gains – Center X ────────────────────────────────────────────────────
CX_KP = 0.0017
CX_KI = 0.0
CX_KD = 0.0001


# ── Frame Dimensions ────────────────────────────────────────────────────────
FRAME_WIDTH  = 640   # pixels
FRAME_HEIGHT = 480   # pixels


# ── Detection Thresholds ────────────────────────────────────────────────────
CENTER_VARIATION         = 10    # pixels – max offset from center before correction
MIN_BLUE_FRAMES          = 5     # consecutive frames to confirm blue line
MIN_RED_FRAMES           = 10    # consecutive frames to confirm red line
FRAMES_TO_CONFIRM_HOSE   = 5     # consecutive frames to confirm hose presence
HOSE_COUNTER             = 0     # runtime counter (initial value)


# ── Speed ────────────────────────────────────────────────────────────────────
FOWARD_SPEED_BLUE_LINE = 0.5   # m/s – cruise speed while following blue line


# ── Seek-Line Recovery (growing square search) ─────────────────────────────
SEEK_SQUARE_BASE_SIDE = 2.0    # m – side length of the first square
SEEK_SQUARE_SPEED     = 0.367  # m/s – forward speed during square legs
SEEK_MAX_SQUARES      = 3      # number of growing squares before aborting
SEEK_SQUARE_GROWTH    = 1.5    # side-length multiplier per iteration
MAX_LOST_FRAMES       = 15     # consecutive frames without detection to trigger seek
SEARCH_TIMEOUT        = 20.0   # s – time without any detection in SEARCH before triggering seek


# ── State-Machine Outcomes ───────────────────────────────────────────────────
FOUND_RED  = "found_red"
FOUND_BLUE = "found_blue"
SEARCH     = "search"
SEEK       = "seek"
