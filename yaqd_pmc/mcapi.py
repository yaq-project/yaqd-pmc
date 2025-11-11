"""

    NAME
        mcapi.py - motion control API function prototypes & macros for python

    DESCRIPTION
        Include this class library in your python source to provide prototypes
        for the motion control API functions.

    import mcapi

    RELEASE HISTORY
    Copyright (c) 2015-2020 by Precision Micro Control Corp. All rights reserved.

        $Id: mcapi.py 1040 2020-11-30 22:00:32Z brian $

    Version 4.4.1       23-Jun-15       Programmer: R. Brian Gaynor
        - First release

    Version 4.5.0       10-Nov-17       Programmer: R. Brian Gaynor
        - Removed unneeded paramter from Reset() declaration
        - Use namespaces for imports (best practice)

    Version 4.5.1       23-Nov-20       Programmer: Brian Gaynor
        - No changes

"""
import ctypes, platform, sys

#
# Motion Control API manifest constants
#
MC_ALL_AXES             = 0                 # Function should operate on all axes at once. Not valid for all functions, see function description.

MC_ABSOLUTE             = 0                 # Argument is an absolute position, used as an parameter to many MCAPI functions.
MC_RELATIVE             = 1                 # Argument is a relative position, used as an parameter to many MCAPI functions.

MC_BLOCK_COMPOUND       = 0                 # Block is a compound command forBlockBegin( ).
MC_BLOCK_TASK           = 1                 # Block is a task on multitasking controllers for BlockBegin( ).
MC_BLOCK_MACRO          = 2                 # Block is a macro definition forBlockBegin( ).
MC_BLOCK_RESETM         = 3                 # Block resets macro memory for BlockBegin( ).
MC_BLOCK_CANCEL         = 4                 # Cancels a block command forBlockBegin( ).
MC_BLOCK_CONTR_USER     = 5                 # Block is a user defined contour path motion for BlockBegin( ).
MC_BLOCK_CONTR_LIN      = 6                 # Block is a linear contour path motion for BlockBegin( ).
MC_BLOCK_CONTR_CC       = 7                 # Block is a clockwise arc contour path motion for BlockBegin( ).
MC_BLOCK_CONTR_CCW      = 8                 # Block is a counter-clockwise arc contour path motion for BlockBegin( ).

MC_CAPTURE_ACTUAL       = 16                # Specifies captured actual position data for GetCaptureData( ) and GetAxisConfiguration( ).
MC_CAPTURE_ERROR        = 32                # Specifies captured following error data for GetCaptureData( ) and GetAxisConfiguration( ).
MC_CAPTURE_OPTIMAL      = 64                # Specifies captured optimal position data data for GetCaptureData( ) and GetAxisConfiguration( ).
MC_CAPTURE_TORQUE       = 128               # Specifies captured torque data for GetCaptureData( ) and GetAxisConfiguration( ).
MC_CAPTURE_ADVANCED     = 256               # Specifies axis supports advanced capture modes (delay and period) for and GetAxisConfiguration( ).
MC_CAPTURE_AUXILIARY    = 512               # Specifies captured auxiliary encoder data for GetCaptureData( ) and GetAxisConfiguration( ) (MCAPI version 3.4.0 or later).
MC_CAPTURE_STATUS       = 1024              # Specifies captured status word data for GetCaptureData( ) and GetAxisConfiguration( ) (MCAPI version 3.4.0 or later).

MC_COMPARE_DISABLE      = 0                 # Disables the compare output for ConfigureCompare( ).
MC_COMPARE_ENABLE       = 1                 # Same as MC_COMPARE_STATIC.
MC_COMPARE_STATIC       = 1                 # Set compare output to static mode for ConfigureCompare( ).
MC_COMPARE_TOGGLE       = 2                 # Set compare output to toggle mode for ConfigureCompare( ).
MC_COMPARE_ONESHOT      = 3                 # Set compare output to one-shot mode for ConfigureCompare( ).
MC_COMPARE_STROBE       = 5                 # Set compare output to strobe mode for ConfigureCompare( ).
MC_COMPARE_INVERT       = 0x0080            # Inverts active level of compare output for ConfigureCompare( ).

MC_COUNT_CAPTURE        = 1                 # GetCount( ) should retrieve the number of captured positions from high-speed capture mode.
MC_COUNT_COMPARE        = 2                 # GetCount( ) should retrieve the number of successful comparisons from high-speed compare mode.
MC_COUNT_CONTOUR        = 4                 # GetCount( ) should retrieve the index of the currently executing contour move from contouring mode.
MC_COUNT_FILTER         = 8                 # GetCount( ) should retrieve the number of digital filter coefficients currently loaded.
MC_COUNT_FILTERMAX      = 16                # GetCount( ) should retrieve the maximum number of digital filter coefficients supported.
MC_COUNT_RECORD         = 32                # GetCount( ) should retrieve the current number of position recording points recorded so far (MCAPI version 4.4.0 or later).

MC_CURRENT_FULL         = 1                 # Selects/indicates full current stepper operation in GetMotionConfigEx( ) / SetMotionConfigEx( ).
MC_CURRENT_HALF         = 2                 # Selects/indicates half current stepper operation in GetMotionConfigEx( ) / SetMotionConfigEx( ).

MC_DIO_FIXED            = 256               # Channel is a fixed input or output and cannot be changed using ConfigureDigitalIO( ).
MC_DIO_INPUT            = 1                 # Configures the channel for input.
MC_DIO_OUTPUT           = 2                 # Configures the channel for output.
MC_DIO_HIGH             = 4                 # Configures the channel for positive logic level.
MC_DIO_LOW              = 8                 # Configures the channel for negative logic level.
MC_DIO_LATCH            = 16                # Configures the (input) channel for latched operation.
MC_DIO_LATCHABLE        = 512               # Input channel is capable of latched operation.
MC_DIO_STEPPER          = 1024              # Input channel has been dedicated to driving a stepper motor (DC2-PC or DC2-STN).

MC_DIR_POSITIVE         = 1                 # Selects the positive travel direction for Direction( ).
MC_DIR_NEGATIVE         = 2                 # Selects the negative travel direction for Direction( ).

MC_ENC_FAULT_PRI        = 1                 # Enables encoder fault detection for the primary encoder with EnableEncoderFault( ).
MC_ENC_FAULT_AUX        = 2                 # Enables encoder fault detection for the auxiliary encoder with EnableEncoderFault( ).

MC_INT_NORMAL           = 0                 # Selects/indicates the normal (always on) operation of the integral term in GetFilterConfigEx( ) / SetFilterConfigEx( ).
MC_INT_FREEZE           = 1                 # Selects/indicates freeze the integral term while moving in GetFilterConfigEx( ) / SetFilterConfigEx( ).
MC_INT_ZERO             = 2                 # Selects/indicates zero and freeze the integral term while moving in GetFilterConfigEx( ) / SetFilterConfigEx( ).

MC_IM_OPENLOOP          = 0                 # Selects/indicates open-loop mode stepper operation in GetModuleInputMode( ) / SetModuleInputMode( ).
MC_IM_CLOSEDLOOP        = 1                 # Selects/indicates closed-loop mode stepper operation in GetModuleInputMode( ) / SetModuleInputMode( ).

MC_LIMIT_ABRUPT         = 4                 # Limit stopping mode is set to abrupt (PID loop stops axis as quickly as possible), used with GetLimits( ) / SetLimits( ).
MC_LIMIT_BOTH           = 3                 # Enables both the positive and negative limits, used with GetLimits( ) / SetLimits( ).
MC_LIMIT_INVERT         = 0x0080            # Inverts the polarity of the hardware limit switch inputs, used with GetLimits( ) / SetLimits( ).
MC_LIMIT_MINUS          = 2                 # Enables the negative limit, used with GetLimits( ) / SetLimits( ).
MC_LIMIT_OFF            = 0                 # Limit stopping mode is set to turn the motor off when a limit is tripped, used with GetLimits( ) / SetLimits( ).
MC_LIMIT_PLUS           = 1                 # Enables the positive limit, used with GetLimits( ) / SetLimits( ).
MC_LIMIT_SMOOTH         = 8                 # Limit stopping mode is set to smooth (axis executes pre-programmed deceleration), used with GetLimits( ) / SetLimits( )..

MC_LRN_POSITION         = 1                 # Indicates LearnPoint( ) should learn the current actual position for the specified axis.
MC_LRN_TARGET           = 2                 # Indicates LearnPoint( ) should learns the current target position for the specified axis.

MC_MAX_ID               = 15                # Controller ID must be less than or equal to this value.

MC_MODE_CONTOUR         = 0                 # Contouring mode operation, see GetOperatingMode( ) / SetOperatingMode( ).
MC_MODE_GAIN            = 1                 # Gain mode operation, see GetOperatingMode( ) / SetOperatingMode( ).
MC_MODE_POSITION        = 2                 # Position mode operation, see GetOperatingMode( ) / SetOperatingMode( ).
MC_MODE_TORQUE          = 3                 # Torque mode operation, see GetOperatingMode( ) / SetOperatingMode( ).
MC_MODE_VELOCITY        = 4                 # Unable to determine current mode of operation, see GetOperatingMode( ).
MC_MODE_UNKNOWN         = 5                 # Velocity mode operation, see GetOperatingMode( ) / SetOperatingMode( ).

MC_MODULE_TYPE          = 0x000f
MC_MODULE_SUBTYPE       = 0x00f0

MC_OM_BIPOLAR           = 0                 # Servo axis set to /in bipolar operation, see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_UNIPOLAR          = 1                 # Servo axis set to/in unipolar operation, see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_PULSE_DIR         = 0                 # Stepper axis set to/in pulse and direction output, see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_CW_CCW            = 1                 # Stepper axis set to/in clockwise and counter-clockwise operation, see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_BI_PWM            = 2                 # Servo axis set to/in bipolar PWM operation, see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_UNI_PWM           = 3                 # Servo axis set to/in unipolar PWM operation, see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_PIEZO             = 4                 # Servo axis set to/in piezo operation, see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_SINE_NOENC        = 0x8000            # Sine commutation mode (encoder off), see GetModuleOutputMode( ) / SetModuleOutputMode( ).
MC_OM_SINE_ENC          = 0xC000            # Sine commutation mode (encoder on), see GetModuleOutputMode( ) / SetModuleOutputMode( ).

MC_OPEN_ASCII           = 1                 # Open controller ASCII interface parameter for MCOpen( ).
MC_OPEN_BINARY          = 2                 # Open controller ASCII interface parameter for MCOpen( ).
MC_OPEN_EXCLUSIVE       = 0x8000            # Combine with MC_OPEN_ASCII or MCOPEN_BINARY for exclusive access to interface.

MC_PHASE_STD            = 0                 # Select/indicate standard encoder phasing in GetServoOutputPhase( ) / GetServoOutputPhase( ).
MC_PHASE_REV            = 1                 # Select/indicate reverse encoder phasing in GetServoOutputPhase( ) / GetServoOutputPhase( ).

MC_PROF_UNKNOWN         = 0                 # Returned when GetProfile( ) cannot determine the current profile setting.
MC_PROF_TRAPEZOID       = 1                 # Selects/indicates a trapezoidal accel/decel profile for GetProfile( ) / SetProfile( ).
MC_PROF_SCURVE          = 2                 # Selects/indicates that an S-curve accel/decel profile for GetProfile( ) / SetProfile( ).
MC_PROF_PARABOLIC       = 4                 # Selects/indicates that a parabolic accel/decel profile for GetProfile( ) / SetProfile( ).

MC_RATE_UNKNOWN         = 0                 # Returned when GetFilterConfigEx( ) cannot determine the current update rate.
MC_RATE_LOW             = 1                 # Selects/indicates the low range for feedback/step/trajectory rate, see GetFilterConfigEx( ) / SetFilterConfigEx( ) / GetTrajectoryRate( ) / SetTrajectoryRate( ).
MC_RATE_MEDIUM          = 2                 # Selects/indicates the medium range for feedback/step/trajectory rate, see GetFilterConfigEx( ) / SetFilterConfigEx( ) / GetTrajectoryRate( ) / SetTrajectoryRate( ).
MC_RATE_HIGH            = 4                 # Selects/indicates the high range for feedback/step/trajectory rate, see GetFilterConfigEx( ) / SetFilterConfigEx( ) / GetTrajectoryRate( ) / SetTrajectoryRate( ).

MC_STAT_BUSY            = 0
MC_STAT_MTR_ENABLE      = 1
MC_STAT_MODE_VEL        = 2
MC_STAT_TRAJ            = 3
MC_STAT_DIR             = 4
MC_STAT_JOG_ENAB        = 5
MC_STAT_HOMED           = 6
MC_STAT_ERROR           = 7
MC_STAT_LOOK_INDEX      = 8
MC_STAT_LOOK_EDGE       = 9
MC_STAT_BREAKPOINT      = 10
MC_STAT_FOLLOWING       = 11
MC_STAT_AMP_ENABLE      = 12
MC_STAT_AMP_FAULT       = 13
MC_STAT_PLIM_ENAB       = 14
MC_STAT_PLIM_TRIP       = 15
MC_STAT_MLIM_ENAB       = 16
MC_STAT_MLIM_TRIP       = 17
MC_STAT_PSOFT_ENAB      = 18
MC_STAT_PSOFT_TRIP      = 19
MC_STAT_MSOFT_ENAB      = 20
MC_STAT_MSOFT_TRIP      = 21
MC_STAT_INP_INDEX       = 22
MC_STAT_INP_HOME        = 23
MC_STAT_INP_AMP         = 24
MC_STAT_INP_AUX         = 25
MC_STAT_INP_PLIM        = 26
MC_STAT_INP_MLIM        = 27
MC_STAT_INP_USER1       = 28
MC_STAT_INP_USER2       = 29
MC_STAT_PHASE           = 30
MC_STAT_FULL_STEP       = 31
MC_STAT_HALF_STEP       = 32
MC_STAT_JOGGING         = 33
MC_STAT_PJOG_ENAB       = 34
MC_STAT_PJOG_ON         = 35
MC_STAT_MJOG_ENAB       = 36
MC_STAT_MJOG_ON         = 37
MC_STAT_INP_PJOG        = 38
MC_STAT_INP_MJOG        = 39
MC_STAT_STOPPING        = 40
MC_STAT_PROG_DIR        = 41
MC_STAT_AT_TARGET       = 42
MC_STAT_ACCEL           = 43
MC_STAT_MODE_POS        = 44
MC_STAT_MODE_TRQE       = 45
MC_STAT_MODE_ARC        = 46
MC_STAT_MODE_CNTR       = 47
MC_STAT_MODE_SLAVE      = 48
MC_STAT_LMT_ABORT       = 49
MC_STAT_LMT_STOP        = 50
MC_STAT_CAPTURE         = 51
MC_STAT_RECORD          = 52
MC_STAT_SYNC            = 53
MC_STAT_MODE_LIN        = 54
MC_STAT_INDEX_FOUND     = 55
MC_STAT_POS_CAPT        = 56
MC_STAT_NULL            = 57
MC_STAT_EDGE_FOUND      = 58
MC_STAT_PRI_ENC_FAULT   = 59
MC_STAT_AUX_ENC_FAULT   = 60
MC_STAT_LOOK_AUX_IDX    = 61
MC_STAT_AUX_IDX_FND     = 62

MC_STEP_FULL            = 1                 # Selects full step stepper motor operation in GetMotionConfigEx( ) / SetMotionConfigEx( )
MC_STEP_HALF            = 2                 # Selects half step stepper motor operation in GetMotionConfigEx( ) / SetMotionConfigEx( )

MC_THREAD               = 0x100

MC_TYPE_NONE            = 0                 # Specifies no data for functions that accept varaible data types.
MC_TYPE_REG             = 1                 # Specifies data in controller register for functions that accept varaible data types. 
MC_TYPE_LONG            = 2                 # Specifies long integer (32-bit) data type for functions that accept varaible data types.
MC_TYPE_FLOAT           = 3                 # Specifies floating point (32-bit) data type for functions that accept varaible data types.
MC_TYPE_DOUBLE          = 4                 # Specifies double precision (64-bit) data type for functions that accept varaible data types.
MC_TYPE_STRING          = 5                 # Specifies string data type for functions that accept varaible data types.

MC_TYPE_SERVO           = 1                 # Axis is a servo motor.
MC_TYPE_STEPPER         = 2                 # Axis is a stepper motor. 

#
# Controller specific manifest constants
#                     
NO_CONTROLLER           = 0
DCXPC100                = 1
DCXAT100                = 2
DCXAT200                = 3
DC2PC100                = 4
DC2STN                  = 5
DCXAT300                = 6
DCXPCI300               = 7
DCXPCI100               = 8
MFXPCI1000              = 9
MFXETH1000              = 10

MC100                   = 5
MC110                   = 4
MC150                   = 6
MC160                   = 7
MC200                   = 0
MC210                   = 16
MC260                   = 1
MC300                   = 2
MC302                   = 22
MC320                   = 162
MC360                   = 3
MC362                   = 23
MC400                   = 8
MC500                   = 12
MF300                   = 10
MF310                   = 9
NO_MODULE               = 15
MFXSERVO                = 252
MFXSTEPPER              = 253
DC2SERVO                = 254
DC2STEPPER              = 255

#
# Error code group masks
#
MCERRMASK_UNSUPPORTED   = 0x00000001        # Function not supported error mask.
MCERRMASK_HANDLE        = 0x00000002        # Bad handle error mask.
MCERRMASK_AXIS          = 0x00000004        # Bad axis number error mask.
MCERRMASK_PARAMETER     = 0x00000008        # Bad parameter error mask.
MCERRMASK_IO            = 0x00000010        # I/O problem error mask.
MCERRMASK_SYSTEM        = 0x00000020        # System level errors error mask.
MCERRMASK_STANDARD      = 0xFFFFFFFE        # Most common MCErrorNotify settings error mask, includes all errors except UNSUPPORTED.

#
# Error codes
#
MCERR_NOERROR           =  0                # Error code:  no error.
MCERR_NO_CONTROLLER     =  1                # Error code:  no controller assigned for this ID, MCERRMASK_SYSTEM group.
MCERR_OUT_OF_HANDLES    =  2                # Error code:  driver out of handles, MCERRMASK_SYSTEM group.
MCERR_OPEN_EXCLUSIVE    =  3                # Error code:  cannot open - exclusive, MCERRMASK_SYSTEM group.
MCERR_MODE_UNAVAIL      =  4                # Error code:  controller already open in different mode, MCERRMASK_SYSTEM group.
MCERR_UNSUPPORTED_MODE  =  5                # Error code:  controller doesn't support this mode, MCERRMASK_SYSTEM group.
MCERR_INIT_DRIVER       =  6                # Error code:  couldn't initialize the device driver, MCERRMASK_SYSTEM group.
MCERR_NOT_PRESENT       =  7                # Error code:  controller hardware not present, MCERRMASK_SYSTEM group.
MCERR_ALLOC_MEM         =  8                # Error code:  memory allocation error, MCERRMASK_SYSTEM group.
MCERR_WINDOWSERROR      =  9                # Error code:  windows function reported an error, MCERRMASK_SYSTEM group.
MCERR_OS_ERROR          =  9                # Error code:  operating sytem function reported an error, MCERRMASK_SYSTEM group.
MCERR_NOTSUPPORTED      = 11                # Error code:  controller doesn't support this feature, MCERRMASK_UNSUPPORTED group.
MCERR_OBSOLETE          = 12                # Error code:  function is obsolete, MCERRMASK_UNSUPPORTED group.
MCERR_CONTROLLER        = 13                # Error code:  invalid controller handle, MCERRMASK_HANDLE group errors.
MCERR_WINDOW            = 14                # Error code:  invalid window handle, MCERRMASK_HANDLE group errors.
MCERR_AXIS_NUMBER       = 15                # Error code:  axis number out of range, MCERRMASK_AXIS group.
MCERR_AXIS_TYPE         = 16                # Error code:  axis type doesn't support this feature, MCERRMASK_AXIS group.
MCERR_ALL_AXES          = 17                # Error code:  cannot select "ALL AXES" for function, MCERRMASK_AXIS group.
MCERR_AXIS_ACTIVE       = 31                # Error code:  axis was enabled or moving, MCERRMASK_AXIS group.
MCERR_RANGE             = 18                # Error code:  parameter was out of range, MCERRMASK_PARAMETER group.
MCERR_CONSTANT          = 19                # Error code:  constant value inappropriate, MCERRMASK_PARAMETER group.
MCERR_NOT_INITIALIZED   = 30                # Error code:  feature not initialized, MCERRMASK_PARAMETER group.
MCERR_UNKNOWN_REPLY     = 20                # Error code:  unexpected or unknown reply, MCERRMASK_IO group.
MCERR_NO_REPLY          = 21                # Error code:  controller failed to reply, MCERRMASK_IO group.
MCERR_REPLY_SIZE        = 22                # Error code:  reply size incorrect, MCERRMASK_IO group.
MCERR_REPLY_AXIS        = 23                # Error code:  wrong axis for reply, MCERRMASK_IO group.
MCERR_REPLY_COMMAND     = 24                # Error code:  reply is for different command, MCERRMASK_IO group.
MCERR_TIMEOUT           = 25                # Error code:  controller failed to respond, MCERRMASK_IO group.
MCERR_BLOCK_MODE        = 26                # Error code:  block mode error, MCERRMASK_IO group.
MCERR_COMM_PORT         = 27                # Error code:  communications port (RS232) error, MCERRMASK_IO group.
MCERR_CANCEL            = 28                # Error code:  operation was canceled, MCERRMASK_IO group.
MCERR_NOT_FOUND         = 29                # Error code:  restore operation could not find data, MCERRMASK_IO group.
MCERR_SOCKET            = 32                # Error code:  tcp/ip socket error, MCERRMASK_IO group.

# controller handle type
HCTRLR = ctypes.c_short

#
# Axis configuration structure
#
class MCAXISCONFIG(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("cbSize", ctypes.c_int),
                ("ModuleType", ctypes.c_int),
                ("ModuleLocation", ctypes.c_int),
                ("MotorType", ctypes.c_int),
                ("CaptureModes", ctypes.c_int),
                ("CapturePoints", ctypes.c_int),
                ("CaptureAndCompare", ctypes.c_int),
                ("HighRate", ctypes.c_double),
                ("MediumRate", ctypes.c_double),
                ("LowRate", ctypes.c_double),
                ("HighStepMin", ctypes.c_double),
                ("HighStepMax", ctypes.c_double),
                ("MediumStepMin", ctypes.c_double),
                ("MediumStepMax", ctypes.c_double),
                ("LowStepMin", ctypes.c_double),
                ("LowStepMax", ctypes.c_double),
                ("AuxEncoder", ctypes.c_int)]
    def __init__(self):
        self.cbSize = sys.getsizeof(self)

#
# Commutation parameters structure
#
class MCCOMMUTATION(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("cbSize", ctypes.c_int),
                ("PhaseA", ctypes.c_double),
                ("PhaseB", ctypes.c_double),
                ("Divisor", ctypes.c_int),
                ("PreScale", ctypes.c_int),
                ("Repeat", ctypes.c_int)]
    def __init__(self):
        self.cbSize = sys.getsizeof(self)

#
# Contouring parameters structure
#
class MCCONTOUR(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("VectorAccel", ctypes.c_double),
                ("VectorDecel", ctypes.c_double),
                ("VectorVelocity", ctypes.c_double),
                ("VelocityOverride", ctypes.c_double)]

#
# PID Filter parameters structure (extended)
#
class MCFILTEREX(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("cbSize", ctypes.c_int),
                ("Gain", ctypes.c_double),
                ("IntegralGain", ctypes.c_double),
                ("IntegrationLimit", ctypes.c_double),
                ("IntegralOption", ctypes.c_int),
                ("DerivativeGain", ctypes.c_double),
                ("DerSamplePeriod", ctypes.c_double),
                ("FollowingError", ctypes.c_double),
                ("VelocityGain", ctypes.c_double),
                ("AccelGain", ctypes.c_double),
                ("DecelGain", ctypes.c_double),
                ("EncoderScaling", ctypes.c_double),
                ("UpdateRate", ctypes.c_int),
                ("PositionDeadband", ctypes.c_double),
                ("DelayAtTarget", ctypes.c_double),
                ("OutputOffset", ctypes.c_double),
                ("OutputDeadband", ctypes.c_double)]
    def __init__(self):
        self.cbSize = sys.getsizeof(self)

#
# Jog control parameters structure (added in MCAPI version 4.1.0)
#
class MCJOGEX(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("cbSize", ctypes.c_int),
                ("Acceleration", ctypes.c_double),
                ("MinVelocity", ctypes.c_double),
                ("Deadband", ctypes.c_double),
                ("Gain", ctypes.c_double),
                ("Offset", ctypes.c_double),
                ("Channel", ctypes.c_int)]
    def __init__(self):
        self.cbSize = sys.getsizeof(self)

#
# Motion parameters structure (extended)
#
class MCMOTIONEX(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("cbSize", ctypes.c_int),
                ("Acceleration", ctypes.c_double),
                ("Deceleration", ctypes.c_double),
                ("Velocity", ctypes.c_double),
                ("MinVelocity", ctypes.c_double),
                ("Direction", ctypes.c_int),
                ("Torque", ctypes.c_double),
                ("Deadband", ctypes.c_double),
                ("DeadbandDelay", ctypes.c_double),
                ("StepSize", ctypes.c_int),
                ("Current", ctypes.c_int),
                ("HardLimitMode", ctypes.c_int),
                ("SoftLimitMode", ctypes.c_int),
                ("SoftLimitLow", ctypes.c_double),
                ("SoftLimitHigh", ctypes.c_double),
                ("EnableAmpFault", ctypes.c_int)]
    def __init__(self):
        self.cbSize = sys.getsizeof(self)

#
# Controller configuration structure (extended)
#
class MCPARAMEX(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("cbSize", ctypes.c_int),
                ("ID", ctypes.c_int),
                ("ControllerType", ctypes.c_int),
                ("NumberAxes", ctypes.c_int),
                ("MaximumAxes", ctypes.c_int),
                ("MaximumModules", ctypes.c_int),
                ("Precision", ctypes.c_int),
                ("DigitalIO", ctypes.c_int),
                ("AnalogInput", ctypes.c_int),
                ("AnalogOutput", ctypes.c_int),
                ("PointStorage", ctypes.c_int),
                ("CanDoScaling", ctypes.c_int),
                ("CanDoContouring", ctypes.c_int),
                ("CanChangeProfile", ctypes.c_int),
                ("CanChangeRates", ctypes.c_int),
                ("SoftLimits", ctypes.c_int),
                ("MultiTasking", ctypes.c_int),
                ("AmpFault", ctypes.c_int),
                ("AnalogInpMin", ctypes.c_double),
                ("AnalogInpMax", ctypes.c_double),
                ("AnalogInpRes", ctypes.c_int),
                ("AnalogOutMin", ctypes.c_double),
                ("AnalogOutMax", ctypes.c_double),
                ("AnalogOutRes", ctypes.c_int),
                ("OutputMode", ctypes.c_int),
                ("AtTarget", ctypes.c_int),
                ("OutputControl", ctypes.c_int),
                ("LineModeAscii", ctypes.c_int)]
    def __init__(self):
        self.cbSize = sys.getsizeof(self)

#
# Scaling factors data structure
#
class MCSCALE(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("Constant", ctypes.c_double),
                ("Offset", ctypes.c_double),
                ("Rate", ctypes.c_double),
                ("Scale", ctypes.c_double),
                ("Zero", ctypes.c_double),
                ("Time", ctypes.c_double)]

#
# Status word data structure
#
class MCSTATUSEX(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("cbSize", ctypes.c_int),
                ("Status", ctypes.c_uint),
                ("AuxStatus", ctypes.c_uint),
                ("ProfileStatus", ctypes.c_uint),
                ("ModeStatus", ctypes.c_uint)]
    def __init__(self):
        self.cbSize = sys.getsizeof(self)

#
# Define a Mcapi exception class
#
class McapiException(Exception):
    def __init__(self, arg):
        # Set some exception infomation
        self.msg = arg

#
# The main MCapi object (all the public access is in this class)
#
class Mcapi:
    # Motion Control API 
    def __init__(self):
        # windows
        if platform.architecture()[1] == 'WindowsPE':
            if platform.architecture()[0] == '32bit':
                self._dll = ctypes.windll.mcapi32
            else:
                self._dll = ctypes.windll.mcapi64
        # ELF is linux, assume default library location (this should be made smarter!)
        elif platform.architecture()[1] == 'ELF':
            self._dll = ctypes.cdll.LoadLibrary(find_library("mcapi"))
        else:
            print("Unsupported platform", platform.architecture())
        self._handle = ctypes.c_short(0)

        #
        # MCAPI function argument and return type declarations. It would be nice if these could be in their 
        # respective member function but there is about a 12% performance penalty if they are run every time a
        # function is called (because python is interpreted). In __init__ they only run once.
        #
        self._dll.MCAbort.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCAbort.restype = None
        self._dll.MCArcCenter.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short, ctypes.c_double]
        self._dll.MCArcEndAngle.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short, ctypes.c_double]
        self._dll.MCArcRadius.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCBlockBegin.argtypes = [HCTRLR, ctypes.c_int, ctypes.c_int]
        self._dll.MCBlockEnd.argtypes = [HCTRLR, ctypes.POINTER(ctypes.c_int)]
        self._dll.MCCancelTask.argtypes = [HCTRLR, ctypes.c_int]
        self._dll.MCCaptureData.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int, ctypes.c_double, ctypes.c_double]
        self._dll.MCClose.argtypes = [HCTRLR]
        self._dll.MCClose.restype = ctypes.c_short
        self._dll.MCConfigureCompare.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_double]
        self._dll.MCConfigureDigitalIO.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort]
        self._dll.MCConfigureDigitalIO.restype = ctypes.c_short
        self._dll.MCContourDistance.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCDecodeStatus.argtypes = [HCTRLR, ctypes.c_uint, ctypes.c_int]
        self._dll.MCDecodeStatusEx.argtypes = [HCTRLR, ctypes.POINTER(MCSTATUSEX), ctypes.c_int]
        self._dll.MCDirection.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort]
        self._dll.MCDirection.restype = None
        self._dll.MCEdgeArm.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCEnableAxis.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short]
        self._dll.MCEnableAxis.restype = None
        self._dll.MCEnableBacklash.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double, ctypes.c_short]
        self._dll.MCEnableCapture.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int]
        self._dll.MCEnableCompare.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int]
        self._dll.MCEnableDigitalFilter.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int]
        self._dll.MCEnableDigitalIO.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short]
        self._dll.MCEnableDigitalIO.restype = None
        self._dll.MCEnableEncoderFault.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int]
        self._dll.MCEnableGearing.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_double, ctypes.c_short]
        self._dll.MCEnableGearing.restype = None
        self._dll.MCEnableJog.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short]
        self._dll.MCEnableJog.restype = None
        self._dll.MCEnableSync.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short]
        self._dll.MCEnableSync.restype = None
        self._dll.MCFindAuxEncIdx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCFindEdge.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCFindIndex.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCGetAccelerationEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetAnalogEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_uint)]
        self._dll.MCGetAuxEncIdxEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetAuxEncPosEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetAxisConfiguration.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCAXISCONFIG)]
        self._dll.MCGetBreakpointEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetCaptureData.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetCaptureSettings.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)]
        self._dll.MCGetConfigurationEx.argtypes = [HCTRLR, ctypes.POINTER(MCPARAMEX)]
        self._dll.MCGetContourConfig.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCCONTOUR)]
        self._dll.MCGetContourConfig.restype = ctypes.c_short
        self._dll.MCGetCount.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        self._dll.MCGetDecelerationEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetDigitalFilter.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        self._dll.MCGetDigitalIOConfig.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort)]
        self._dll.MCGetDigitalIOEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_uint)]
        self._dll.MCGetError.argtypes = [HCTRLR]
        self._dll.MCGetError.restype = ctypes.c_short
        self._dll.MCGetFilterConfigEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCFILTEREX)]
        self._dll.MCGetFollowingError.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetGain.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetIndexEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetInstalledModules.argtypes = [HCTRLR, ctypes.POINTER(ctypes.c_long), ctypes.c_int]
        self._dll.MCGetJogConfigEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCJOGEX)]
        self._dll.MCGetLimits.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_short), ctypes.POINTER(ctypes.c_short), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetModuleInputMode.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_int)]
        self._dll.MCGetModuleOutputMode.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort)]
        self._dll.MCGetMotionConfigEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCMOTIONEX)]
        self._dll.MCGetOperatingMode.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_int)]
        self._dll.MCGetOptimalEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetPositionEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetProfile.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort)]
        self._dll.MCGetRegister.argtypes = [HCTRLR, ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
        self._dll.MCGetScale.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCSCALE)]
        self._dll.MCGetScale.restype = ctypes.c_short
        self._dll.MCGetServoOutputPhase.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort)]
        self._dll.MCGetStatus.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCGetError.restype = ctypes.c_uint
        self._dll.MCGetStatusEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCSTATUSEX)]
        self._dll.MCGetTargetEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetTorque.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetTrajectoryRate.argtypes = [HCTRLR, ctypes.POINTER(ctypes.c_int)]
        self._dll.MCGetVectorVelocity.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetVelocityActual.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetVelocityEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetVelocityOverride.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCGetVersion.argtypes = [HCTRLR]
        self._dll.MCGetVersion.restype = ctypes.c_uint
        self._dll.MCGoEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCGoHome.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCGoHome.restype = None
        self._dll.MCIndexArm.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCInterruptOnPosition.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int, ctypes.c_double]
        self._dll.MCIsAtTarget.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCIsDigitalFilter.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCIsEdgeFound.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCIsIndexFound.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCIsStopped.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCLearnPoint.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int, ctypes.c_ushort]
        self._dll.MCMacroCall.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCMacroCall.restype = None
        self._dll.MCMoveAbsolute.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCMoveAbsolute.restype = None
        self._dll.MCMoveRelative.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCMoveRelative.restype = None
        self._dll.MCMoveToPoint.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int]
        self._dll.MCOpen.argtypes = [ctypes.c_short, ctypes.c_ushort, ctypes.c_char_p]
        self._dll.MCOpen.restype = HCTRLR
        self._dll.MCReopen.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCRepeat.argtypes = [HCTRLR, ctypes.c_int]
        self._dll.MCReset.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCReset.restype = None
        self._dll.MCSetAcceleration.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCSetAcceleration.restype = None
        self._dll.MCSetAnalogEx.argtypes = [HCTRLR, ctypes.c_int, ctypes.c_uint]
        self._dll.MCSetAuxEncPos.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCSetAuxEncPos.restype = None
        self._dll.MCSetCommutation.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCCOMMUTATION)]
        self._dll.MCSetContourConfig.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCCONTOUR)]
        self._dll.MCSetContourConfig.restype = ctypes.c_short
        self._dll.MCSetDeceleration.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCSetDeceleration.restype = None
        self._dll.MCSetDigitalFilter.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        self._dll.MCSetFilterConfigEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCFILTEREX)]
        self._dll.MCSetGain.argtypes = [HCTRLR, ctypes.c_int, ctypes.c_double]
        self._dll.MCSetJogConfigEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCJOGEX)]
        self._dll.MCSetLimits.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short, ctypes.c_short, ctypes.c_double, ctypes.c_double]
        self._dll.MCSetModuleInputMode.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int]
        self._dll.MCSetModuleOutputMode.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort]
        self._dll.MCSetModuleOutputMode.restype = None
        self._dll.MCSetMotionConfigEx.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCMOTIONEX)]
        self._dll.MCSetOperatingMode.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ushort]
        self._dll.MCSetOperatingMode.restype = None
        self._dll.MCSetPosition.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCSetPosition.restype = None
        self._dll.MCSetProfile.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort]
        self._dll.MCSetProfile.restype = None
        self._dll.MCSetRegister.argtypes = [HCTRLR, ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
        self._dll.MCSetScale.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.POINTER(MCSCALE)]
        self._dll.MCSetScale.restype = ctypes.c_short
        self._dll.MCSetServoOutputPhase.restype = ctypes.c_short
        self._dll.MCSetServoOutputPhase.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort]
        self._dll.MCSetServoOutputPhase.restype = None
        self._dll.MCSetTimeoutEx.argtypes = [HCTRLR, ctypes.c_double, ctypes.POINTER(ctypes.c_double)]
        self._dll.MCSetTorque.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCSetTrajectoryRate.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_int]
        self._dll.MCSetVectorVelocity.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCSetVelocity.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCSetVelocity.restype = None
        self._dll.MCSetVelocityOverride.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCStop.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCStop.restype = None
        self._dll.MCTranslateErrorEx.argtypes = [ctypes.c_short, ctypes.c_char_p, ctypes.c_int]
        self._dll.MCWait.argtypes = [HCTRLR, ctypes.c_double]
        self._dll.MCWait.restype = None
        self._dll.MCWaitForDigitalIO.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short]
        self._dll.MCWaitForDigitalIO.restype = None
        self._dll.MCWaitForEdge.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_short]
        self._dll.MCWaitForIndex.argtypes = [HCTRLR, ctypes.c_ushort]
        self._dll.MCWaitForPosition.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCWaitForPosition.restype = None
        self._dll.MCWaitForRelative.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCWaitForRelative.restype = None
        self._dll.MCWaitForStop.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCWaitForStop.restype = None
        self._dll.MCWaitForTarget.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_double]
        self._dll.MCWaitForTarget.restype = None

        self._dll.pmccmdex.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_int]
        self._dll.pmccmdrpyex.argtypes = [HCTRLR, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
        self._dll.pmcgetc.argtypes = [HCTRLR]
        self._dll.pmcgetc.restype = ctypes.c_short
        self._dll.pmcgetramex.argtypes = [HCTRLR, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]
        self._dll.pmcgets.argtypes = [HCTRLR, ctypes.c_void_p, ctypes.c_short]
        self._dll.pmcgets.restype = ctypes.c_short
        self._dll.pmclock.argtypes = [HCTRLR, ctypes.c_uint]
        self._dll.pmclookupvar.argtypes = [HCTRLR, ctypes.c_uint, ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint)]
        self._dll.pmcputc.argtypes = [HCTRLR, ctypes.c_short]
        self._dll.pmcputc.restype = ctypes.c_short
        self._dll.pmcputramex.argtypes = [HCTRLR, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]
        self._dll.pmcputs.argtypes = [HCTRLR, ctypes.c_char_p]
        self._dll.pmcputs.restype = ctypes.c_short
        self._dll.pmcrdy.argtypes = [HCTRLR]
        self._dll.pmcrdy.restype = ctypes.c_short
        self._dll.pmcrpyex.argtypes = [HCTRLR, ctypes.c_void_p, ctypes.c_uint]
        self._dll.pmcunlock.argtypes = [HCTRLR]
        self._dll.pmcunlock.restype = None

    def Abort(self, axis):
        """Command an axis to execute an emergency stop."""
        self._dll.MCAbort(self._handle, axis)
        self.ProcessException()

    def ArcCenter(self, axis, type, position):
        """Define the center of an arc for contour path motion."""
        self.ProcessException(self._dll.MCArcCenter(self._handle, axis, type, position))

    def ArcEndAngle(self, axis, type, angle):
        """Define the ending angle of an arc in contour path motion."""
        self.ProcessException(self._dll.MCArcEndAngle(self._handle, axis, type, angle))

    def ArcRadius(self, axis, radius):
        """Return the radius of an arc in contour path motion."""
        self.ProcessException(self._dll.MCArcRadius(self._handle, axis, radius))

    def BlockBegin(self, mode, num):
        """Mark the beginning of a block command sequence."""
        self.ProcessException(self._dll.MCBlockBegin(self._handle, mode, num))

    def BlockEnd(self):
        """End a block command sequence and transmit the compound command, task, macro, or contour path."""
        task = ctypes.c_int(0)
        self.ProcessException(self._dll.MCBlockEnd(self._handle, ctypes.byref(task)))
        return task.value

    def CancelTask(self, task):
        """Cancel executing background task on a multi-tasking controller. """
        self.ProcessException(self._dll.MCCancelTask(self._handle, task))

    def CaptureData(self, axis, points, period, delay):
        """Configure data recording for an axis."""
        self.ProcessException(self._dll.MCCaptureData(self._handle, axis, points, period, delay))

    def Close(self):
        """Close the existing Motion Control API handle for this object."""
        if self._handle > 0:
            self._dll.MCClose(self._handle)
            self._handle = ctypes.c_short(0)

    def ConfigureCompare(self, axis, values, num, inc, mode, period):
        """Configure axis high-speed position compare mode operation

        Declare 'values' as values = ctypes.c_double * n"""
        self.ProcessException(self._dll.MCConfigureCompare(self._handle, axis, values, num. inc, mode, period))

    def ConfigureDigitalIO(self, axis, mode):
        """Configure digital I/O channel for input or output, and for high or low true logic."""
        self._dll.MCConfigureDigitalIO(self._handle, axis, mode)
        self.ProcessException()

    def ContourDistance(self, axis, distance):
        """Specify distance for user defined contour path motions."""
        self.ProcessException(self._dll.MCContourDistance(self._handle, axis, distance))

    def DecodeStatus(self, status, bit):
        """Test flags in the controller status word in a controller model independent way.""" 
        state = self._dll.MCDecodeStatus(self._handle, status, bit)
        self.ProcessException()
        return state

    def DecodeStatusEx(self, status, bit):
        """Test flags in the MCSTATUS structure in a controller model independent way.""" 
        state = self._dll.MCDecodeStatusEx(self._handle, status, bit)
        self.ProcessException()
        return state

    def Direction(self, axis, dir):
        """Set travel direction of axis in velocity mode."""
        self._dll.MCDirection(self._handle, axis, dir)
        self.ProcessException()

    def EdgeArm(self, axis, position):
        """Arm edge capture function of home input on an open-loop  stepper."""
        self.ProcessException(self._dll.MCEdgeArm(self._handle, axis, position))

    def EnableAxis(self, axis, state):
        """Set or clear the enable state of axis."""
        self._dll.MCEnableAxis(self._handle, axis, state)
        self.ProcessException()

    def EnableBacklash(self, axis, backlash, state):
        """Enable (and configure) or disable backlash compensation for axis."""
        self.ProcessException(self._dll.MCEnableBacklash(self._handle, axis, backlash, state))

    def EnableCapture(self, axis, count):
        """Begin/end high-speed position capture for axis."""
        self.ProcessException(self._dll.MCEnableCapture(self._handle, axis, count))

    def EnableCompare(self, axis, count):
        """Enable/disables high-speed compare mode for axis."""
        self.ProcessException(self._dll.MCEnableCompare(self._handle, axis, count))

    def EnableDigitalFilter(self, axis, state):
        """Enable/disable the digital filter for axis."""
        self.ProcessException(self._dll.MCEnableDigitalFilter(self._handle, axis, state))

    def EnableDigitalIO(self, axis, state):
        """Turn the specified digital I/O channel on or off."""
        self._dll.MCEnableDigitalIO(self._handle, axis, state)
        self.ProcessException()

    def EnableEncoderFault(self, axis, state):
        """Enable/disable encoder fault detection."""
        self.ProcessException(self._dll.MCEnableEncoderFault(self._handle, axis, state))

    def EnableGearing(self, axis, master, ratio, state):
        """Enable/disable electronic gearing for the master axis / slave axis pair."""
        self._dll.MCEnableGearing(self._handle, axis, master, ratio, state)
        self.ProcessException()

    def EnableJog(self, axis, state):
        """Enable/disable jogging for axis."""
        self._dll.MCEnableJog(self._handle, axis, state)
        self.ProcessException()

    def EnableSync(self, axis, state):
        """Enable/disable synchronized motion for contour path motion for axis."""
        self._dll.MCEnableSync(self._handle, axis, state)
        self.ProcessException()

    def FindAuxEncIdx(self, axis, position):
        """Initialize auxiliary encoder to a given position, relative to the auxiliary encoder index."""
        self.ProcessException(self._dll.MCFindAuxEncIdx(self._handle, axis, position))

    def FindEdge(self, axis, position):
        """Initialize a stepper motor at a given position, relative to the coarse home input."""
        self.ProcessException(self._dll.MCFindEdge(self._handle, axis, position))

    def FindIndex(self, axis, position):
        """Initialize a servo motor at a given position, relative to the index input."""
        self.ProcessException(self._dll.MCFindIndex(self._handle, axis, position))

    def GetAccelerationEx(self, axis):
        """Get the current programmed acceleration value for the given axis."""
        accel = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetAccelerationEx(self._handle, axis, accel))
        return accel.value

    def GetAnalogEx(self, axis):
        """Get the current input state of an analog input channel."""
        value = ctypes.c_uint(0)
        self.ProcessException(self._dll.MCGetAnalogEx(self._handle, axis, value))
        return value.value

    def GetAuxEncIdxEx(self, axis):
        """Get the position where the auxiliary encoder's index pulse was observed."""
        index = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetAuxEncIdxEx(self._handle, axis, index))
        return index.value

    def GetAuxEncPosEx(self, axis):
        """Get the current position of the auxiliary encoder."""
        pos = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetAuxEncPosEx(self._handle, axis, pos))
        return pos.value

    def GetAxisConfiguration(self, axis, axiscfg):
        """Get the configuration for the specified axis."""
        self.ProcessException(self._dll.MCGetAxisConfiguration(self._handle, axis, axiscfg))

    def GetBreakpointEx(self, axis):
        """Get the current axis breakpoint position."""
        breakpoint = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetBreakpointEx(self._handle, axis, breakpoint))
        return breakpoint.value

    def GetCaptureData(self, axis, type, start, points):
        """Get data collected by the most recent CaptureData() call."""
        temp = (ctypes.c_double * points)()
        self.ProcessException(self._dll.MCGetCaptureData(self._handle, axis, type, start, points, ctypes.cast(temp, ctypes.POINTER(ctypes.c_double))))
        # convert data to a friendly python list
        data = []
        for i in range(0, points):
            if type & MC_CAPTURE_STATUS:
                data.append(int(temp[i]))
            else:
                data.append(temp[i])
        return data

    def GetCaptureSettings(self, axis):
        """Get the current data recording settings for axis.

        Returns settings as tuple: (points, period, delay, index)"""
        points = ctypes.c_int()
        period = ctypes.c_double()
        delay = ctypes.c_double()
        index = ctypes.c_int()
        self.ProcessException(self._dll.MCGetCaptureSettings(self._handle, axis, points, period, delay, index))
        return (points.value, period.value, delay.value, index.value)

    def GetConfigurationEx(self, param):
        """Get the controller configuration."""
        self.ProcessException(self._dll.MCGetConfigurationEx(self._handle, param))

    def GetContourConfig(self, axis, contour):
        """Get the contouring configuration for axis."""
        self._dll.MCGetContourConfig(self._handle, axis, contour)
        self.ProcessException()

    def GetCount(self, axis, type):
        """Get various count values for axis.

        type is one of type MC_COUNT_CAPTURE, MC_COUNT_COMPARE, MC_COUNT_CONTOUR,
        MC_COUNT_FILTER, MC_COUNT_FILTERMAX, or MC_COUNT_RECORD"""
        count = ctypes.c_int(0)
        self.ProcessException(self._dll.MCGetCount(self._handle, axis, type, count))
        return count.value

    def GetDecelerationEx(self, axis):
        """Get the current programmed deceleration value for axis."""
        decel = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetDecelerationEx(self._handle, axis, decel))
        return decel.value

    def GetDigitalFilter(self, axis, coeff, num, actual):
        """Get the digital filter coefficients for axis."""
        self.ProcessException(self._dll.MCGetDigitalFilter(self._handle, axis, coeff, num, actual))

    def GetDigitalIOConfig(self, channel):
        """Get the current configuration (in / out / high / low) of the a digital I/O channel."""
        mode = ctypes.c_ushort(0)
        self.ProcessException(self._dll.MCGetDigitalIOConfig(self._handle, axis, mode))
        return mode.value

    def GetDigitalIOEx(self, channel):
        """Get the current state of the specified digital I/O channel."""
        state = ctypes.c_uint(0)
        self.ProcessException(self._dll.MCGetDigitalIOEx(self._handle, channel, state))
        return state.value

    def GetError(self):
        """Get the most recent MCAPI function call error code."""
        return self._dll.MCGetError(self._handle)

    def GetFilterConfigEx(self, axis, filter):
        """Get the current PID filter configuration for a servo motor or closed-loop stepper."""
        self.ProcessException(self._dll.MCGetFilterConfigEx(self._handle, axis, filter))

    def GetFollowingError(self, axis):
        """Get the current axis following error."""
        error = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetFollowingError(self._handle, axis, error))
        return error.value

    def GetGain(self, axis):
        """Get the current gain setting for axis."""
        gain = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetGain(self._handle, axis, gain))
        return gain.value

    def GetIndexEx(self, axis):
        """Get the position where the encoder index pulse was observed for axis."""
        index = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetIndexEx(self._handle, axis, index))
        return index.value
    
    def GetInstalledModules(self, size = 16):
        """Enumerate the types of modules installed on a motion controller."""
        temp = (ctypes.c_long * size)()
        self.ProcessException(self._dll.MCGetInstalledModules(self._handle, ctypes.cast(temp, ctypes.POINTER(ctypes.c_long)), size))
        # convert data to a friendly python list
        modules = []
        for i in range(0, size):
            modules.append(temp[i])
        return modules

    def GetJogConfigEx(self, axis, jog):
        """Get the current jog configuration block for axis."""
        self.ProcessException(self._dll.MCGetJogConfigEx(self._handle, axis, jog))

    def GetLimits(self, axis):
        """Get the current hard and soft limit settings for axis.

        Returns settings as tuple: (hardlimitmode, softlimitmode, softlimitminus, softlimitplus)"""
        hardlimitmode = ctypes.c_short(0)
        softlimitmode = ctypes.c_short(0)
        softlimitminus = ctypes.c_double(0)
        softlimitplus = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetLimits(self._handle, axis, hardlimitmode, softlimitmode, softlimitminus, softlimitplus))
        return (hardlimitmode.value, softlimitmode.value, softlimitminus.value, softlimitplus.value)

    def GetModuleInputMode(self, axis):
        """Get the current input mode for axis."""
        mode = ctypes.c_int(0)
        self.ProcessException(self._dll.MCGetModuleInputMode(self._handle, axis, mode))
        return mode.value

    def GetModuleOutputMode(self, axis):
        """Get the current output mode for axis."""
        mode = ctypes.c_ushort(0)
        self.ProcessException(self._dll.MCGetModuleOutputMode(self._handle, axis, mode))
        return mode.value

    def GetMotionConfigEx(self, axis, motion):
        """Get the current motion configuration for axis."""
        self.ProcessException(self._dll.MCGetMotionConfigEx(self._handle, axis, motion))

    def GetOperatingMode(self, axis):
        """Get the current operating mode (position, velocity, gain, torque, etc.) for axis."""
        mode = ctypes.c_int(0)
        self.ProcessException(self._dll.MCGetOperatingMode(self._handle, axis, mode))
        return mode.value

    def GetOptimalEx(self, axis):
        """Get the current optimal position from the trajectory generator for axis."""
        optimal = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetOptimalEx(self._handle, axis, optimal))
        return optimal.value

    def GetPositionEx(self, axis):
        """Get the current position for axis."""
        pos = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetPositionEx(self._handle, axis, pos))
        return pos.value

    def GetProfile(self, axis):
        """Get the current acceleration/deceleration profile for axis."""
        profile = ctypes.c_ushort(0)
        self.ProcessException(self._dll.MCGetProfile(self._handle, axis, profile))
        return profile.value

    def GetRegister(self, reg, type):
        """Get the value of the specified general purpose register."""
        if type == MC_TYPE_LONG:
            temp = ctypes.c_long(0)
        elif type == MC_TYPE_FLOAT:
            temp = c_float(0)
        elif type == MC_TYPE_DOUBLE:
            temp = ctypes.c_double(0)
        elif type == MC_TYPE_STRING:
            temp = c_char(0)
        else:
            raise McapiException(self.TranslateErrorEx(MCERR_CONSTANT))
        self.ProcessException(self._dll.MCGetRegister(self._handle, reg, ctypes.byref(temp), type))
        return temp.value

    def GetScale(self, axis, scale):
        """Get the current scaling factors for axis."""
        self._dll.MCGetScale(self._handle, axis, scale)
        self.ProcessException()

    def GetServoOutputPhase(self, axis):
        """Get the current servo output phasing for axis."""
        phase = ctypes.c_ushort(0)
        self.ProcessException(self._dll.MCGetServoOutputPhase(self._handle, axis, phase))
        return phase.value

    def GetStatus(self, axis):
        """Get the primary controller status word for axis."""
        status = self._dll.MCGetStatus(self._handle, axis)
        self.ProcessException()
        return status

    def GetStatusEx(self, axis, status):
        """Get the controller status words for the specified axis."""
        self.ProcessException(self._dll.MCGetStatusEx(self._handle, axis, status))

    def GetTargetEx(self, axis):
        """Get the move target position for axis."""
        target = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetTargetEx(self._handle, axis, target))
        return target.value

    def GetTorque(self, axis):
        """Get the current torque setting for axis."""
        torque = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetTorque(self._handle, axis, torque))
        return torque.value

    def GetTrajectoryRate(self, axis):
        """Get the current trajectory generator rate setting."""
        rate = ctypes.c_int(0)
        self.ProcessException(self._dll.MCGetTrajectoryRate(self._handle, axis, rate))
        return rate.value

    def GetVectorVelocity(self, axis):
        """Get the current vector velocity for contouring mode."""
        vel = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetVectorVelocity(self._handle, axis, vel))
        return vel.value

    def GetVelocityActual(self, axis):
        """Get the current actual velocity for axis."""
        vel = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetVelocityActual(self._handle, axis, vel))
        return vel.value

    def GetVelocityEx(self, axis):
        """>Get the current programmed velocity for axis."""
        vel = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetVelocityEx(self._handle, axis, vel))
        return vel.value

    def GetVelocityOverride(self, axis):
        """Get the current velocity override for axis."""
        vo = ctypes.c_double(0)
        self.ProcessException(self._dll.MCGetVelocityOverride(self._handle, axis, vo))
        return vo.value

    def GetVersion(self):
        """Get version information about the current Motion Control API."""
        ver = self._dll.MCGetVersion(self._handle)
        self.ProcessException()
        return ver.value

    def GoEx(self, axis, param):
        """Initiate a velocity mode motion."""
        self.ProcessException(self._dll.MCGoEx(self._handle, axis, param))

    def GoHome(self, axis):
        """Initiate a home move for axis."""
        self._dll.MCGoHome(self._handle, axis)
        self.ProcessException()

    def Handle(self):
        """Get the underlying MCAPI handle contained by this object."""
        return self._handle

    def IndexArm(self, axis, position):
        """Arm the index capture function of a servo axis."""
        self.ProcessException(self._dll.MCIndexArm(self._handle, axis, position))

    def InterruptOnPosition(self, axis, mode, position):
        """Enable the breakpoint flag of the controller status word."""
        self.ProcessException(self._dll.MCInterruptOnPosition(self._handle, axis, mode, position))

    def IsAtTarget(self, axis, timeout):
        """Check status of the "At Target" status condition for one or all axes."""
        return self._dll.MCIsAtTarget(self._handle, axis, timeout)

    def IsDigitalFilter(self, axis):
        """Check if the digital filter is enabled for one or all axes."""
        return self._dll.MCIsDigitalFilter(self._handle, axis)

    def IsEdgeFound(self, axis, timeout):
        """Check status of the "Edge Found" status condition for one or all axes."""
        return self._dll.MCIsEdgeFound(self._handle, axis, timeout)

    def IsIndexFound(self, axis, timeout):
        """Check status of the "Index Found" status condition for one or all axes."""
        return self._dll.MCIsIndexFound(self._handle, axis, timeout)

    def IsStopped(self, axis, timeout):
        """Check status of the "Trajectory Complete" status condition for one or all axes."""
        return self._dll.MCIsStopped(self._handle, axis, timeout)

    def LearnPoint(self, axis, index, mode):
        """Store the current actual position or target position for axis in point memory."""
        self.ProcessException(self._dll.MCLearnPoint(self._handle, axis, index, mode))

    def MacroCall(self, macro):
        """Execute a previously loaded macro."""
        self._dll.MCMacroCall(self._handle, macro)
        self.ProcessException()

    def MoveAbsolute(self, axis, position):
        """Start an absolute position-mode move for one or all axes."""
        self._dll.MCMoveAbsolute(self._handle, axis, position)
        self.ProcessException()

    def MoveRelative(self, axis, distance):
        """Start a relative position move for one or all axes."""
        self._dll.MCMoveRelative(self._handle, axis, distance)
        self.ProcessException()

    def MoveToPoint(self, axis, index):
        """Start an absolute move to a stored location for for one or all axes."""
        self.ProcessException(self._dll.MCMoveToPoint(self._handle, axis, index))

    def Open(self, id, mode, opts=None):
        """Opens a Motion Control API handle to a controller."""
        self._handle = self._dll.MCOpen(id, mode, opts)
        return self._handle

    def Reopen(self, newmode):
        """Change the mode of an existing motion control API handle."""
        self.ProcessException(self._dll.MCReopen(self._handle, newmode))

    def Repeat(self, count):
        """Insert a repeat command into a block command sequence."""
        self.ProcessException(self._dll.MCRepeat(self._handle, count))

    def Reset(self, axis):
        """Perform a complete reset of axis or the controller."""
        self._dll.MCReset(self._handle, axis)
        self.ProcessException()

    def SetAcceleration(self, axis, rate):
        """Set programmed acceleration value for axis."""
        self._dll.MCSetAcceleration(self._handle, axis, rate)
        self.ProcessException()

    def SetAnalogEx(self, channel, value):
        """Set the output level of an analog channel."""
        self.ProcessException(self._dll.MCSetAnalogEx(self._handle, channel, value))

    def SetAuxEncPos(self, axis, position):
        """Set the current position of the auxiliary encoder."""
        self._dll.MCSetAuxEncPos(self._handle, axis, position)
        self.ProcessException()

    def SetCommutation(self, axis, commutation):
        """set the commutation settings."""
        self.ProcessException(self._dll.MCSetCommutation(self._handle, axis, commutation))

    def SetContourConfig(self, axis, contour):
        """Set the contouring configuration for axis."""
        self._dll.MCSetContourConfig(self._handle, axis, ctypes.byref(contour))
        self.ProcessException()

    def SetDeceleration(self, axis, rate):
        """Set programmed deceleration value for axis."""
        self._dll.MCSetDeceleration(self._handle, axis, rate)
        self.ProcessException()

    def SetDigitalFilter(self, axis, coeff, num):
        """Set the digital filter coefficients for axis."""
        self.ProcessException(self._dll.MCSetDigitalFilter(self._handle, axis, coeff, num))

    def SetFilterConfigEx(self, axis, filter):
        """Configure the PID loop settings for a servo motor or the closed-loop stepper motor."""
        self.ProcessException(self._dll.MCSetFilterConfigEx(self._handle, axis, filter))

    def SetGain(self, channel, gain):
        """Set the proportional gain of a servo's feedback loop."""
        self.ProcessException(self._dll.MCSetGain(self._handle, channel, gain))

    def SetJogConfigEx(self, axis, jog):
        """Set the jog configuration for axis."""
        self.ProcessException(self._dll.MCSetJogConfigEx(self._handle, axis, jog))

    def SetLimits(self, axis, hardlimitmode, softlimitmode, softlimitminus, softlimitplus):
        """Set the hard and soft limit settings for axis."""
        self.ProcessException(self._dll.MCSetLimits(self._handle, axis, hardlimitmode, softlimitmode, softlimitminus, softlimitplus))

    def SetModuleInputMode(self, axis, mode):
        """Set the input mode for axis."""
        self.ProcessException(self._dll.MCSetModuleInputMode(self._handle, axis, mode))

    def SetModuleOutputMode(self, axis, mode):
        """Configure the output of axis."""
        self._dll.MCSetModuleOutputMode(self._handle, axis, mode)
        self.ProcessException()

    def SetMotionConfigEx(self, axis, motion):
        """Configure an axis for motion."""
        self.ProcessException(self._dll.MCSetMotionConfigEx(self._handle, axis, motion))

    def SetOperatingMode(self, axis, caxis, mode):
        """Set axis operating mode (position, velocity, gain, torque, contour)."""
        self._dll.MCSetOperatingMode(self._handle, axis, caxis, mode)
        self.ProcessException()

    def SetPosition(self, axis, position):
        """Set the current position for axis."""
        self._dll.MCSetPosition(self._handle, axis, position)
        self.ProcessException()

    def SetProfile(self, axis, mode):
        """Set the acceleration profile for axis."""
        self._dll.MCSetProfile(self._handle, axis, mode)
        self.ProcessException()

    def SetRegister(self, reg, value, type):
        """Set the value of the specified general purpose register."""
        if type == MC_TYPE_LONG:
            temp = ctypes.c_long(value)
        elif type == MC_TYPE_FLOAT:
            temp = c_float(value)
        elif type == MC_TYPE_DOUBLE:
            temp = ctypes.c_double(value)
        elif type == MC_TYPE_STRING:
            temp = c_char(value)
        else:
            raise McapiException(self.TranslateErrorEx(MCERR_CONSTANT))
        self.ProcessException(self._dll.MCSetRegister(self._handle, reg, ctypes.byref(temp), type))

    def SetScale(self, axis, scale):
        """Set the scale factors for axis."""
        self._dll.MCSetScale(self._handle, axis, scale)
        self.ProcessException()

    def SetServoOutputPhase(self, axis, mode):
        """Set the output phase for the specified servo axis."""
        self._dll.MCSetServoOutputPhase(self._handle, axis, mode)
        self.ProcessException()

    def SetTimeoutEx(self, timeout):
        """Set the MCAPI function timeout for a controller."""
        old = ctypes.c_double(0)
        self.ProcessException(self._dll.MCSetTimeoutEx(self._handle, timeout, old))
        return old.value

    def SetTorque(self, axis, torque):
        """Set the maximum output level for servos."""
        self.ProcessException(self._dll.MCSetTorque(self._handle, axis, torque))

    def SetTrajectoryRate(self, axis, rate):
        """Set the current trajectory generator rate setting."""
        self.ProcessException(self._dll.MCSetTrajectoryRate(self._handle, axis, rate))

    def SetVectorVelocity(self, axis, velocity):
        """Set the vector velocity for a contouring group."""
        self.ProcessException(self._dll.MCSetVectorVelocity(self._handle, axis, velocity))

    def SetVelocity(self, axis, velocity):
        """Set the programmed velocity for an axis."""
        self._dll.MCSetVelocity(self._handle, axis, velocity)
        self.ProcessException()

    def SetVelocityOverride(self, axis, override):
        """Set the velocity override for the specified axis."""
        self.ProcessException(self._dll.MCSetVelocityOverride(self._handle, axis, override))

    def Stop(self, axis):
        """Command axis to stop using the pre-programmed deceleration rate for axis."""
        self._dll.MCStop(self._handle, axis)
        self.ProcessException()

    def TranslateErrorEx(self, error):
        """Translate MCAPI numeric error codes into text."""
        buffer = create_string_buffer(256)
        self._dll.MCTranslateErrorEx(error, buffer, 256)
        return buffer.value.decode()

    def Wait(self, period):
        """Have the motion controller delay the specified number of seconds before continuing command execution."""
        self._dll.MCWait(self._handle, period)
        self.ProcessException()

    def WaitForDigitalIO(self, channel, state):
        """Wait for the specified digital I/O channel to go on or off before continuing command execution."""
        self._dll.MCWaitForDigitalIO(self._handle, channel, state)
        self.ProcessException()

    def WaitForEdge(self, axis, state):
        """Wait for the coarse home input to go to the specified logic level before continuing command execution."""
        self.ProcessException(self._dll.MCWaitForEdge(self._handle, axis, state))

    def WaitForIndex(self, axis):
        """Wait until the index pulse has been observed on a servo axis before continuing command execution."""
        self.ProcessException(self._dll.MCWaitForIndex(self._handle, axis))

    def WaitForPosition(self, axis, position):
        """Wait for the axis to reach the specified position before continuing command execution."""
        self._dll.MCWaitForPosition(self._handle, axis, position)
        self.ProcessException()

    def WaitForRelative(self, axis, distance):
        """>Wait for an axis to reach a position that is specified relative to the target position."""
        self._dll.MCWaitForRelative(self._handle, axis, distance)
        self.ProcessException()

    def WaitForStop(self, axis, period):
        """Wait for axis or all axes to come to a stop."""
        self._dll.MCWaitForStop(self._handle, axis, period)
        self.ProcessException()

    def WaitForTarget(self, axis, period):
        """Wait for axis to reach its target position."""
        self._dll.MCWaitForTarget(self._handle, axis, period)
        self.ProcessException()

    #
    # low level OEM functions
    #
    def pmccmdex(self, axis, command, argument, type):
        """Send a formatted binary command buffer to the controller."""
        if type == MC_TYPE_NONE:
            arg = None
        elif type == MC_TYPE_REG:
            arg = ctypes.c_long(argument)
        elif type == MC_TYPE_LONG:
            arg = ctypes.c_long(argument)
        elif type == MC_TYPE_FLOAT:
            arg = c_float(argument)
        elif type == MC_TYPE_DOUBLE:
            arg = ctypes.c_double(argument)
        elif type == MC_TYPE_STRING:
            arg = c_char(argument)
        else:
            raise McapiException(self.TranslateErrorEx(MCERR_CONSTANT))
        self.ProcessException(self._dll.pmccmdex(self._handle, axis, command, ctypes.byref(arg), type))

    def pmccmdrpyex(self, axis, command, argument, arg_type, rpy_type):
        """>Send a formatted binary command buffer to the controller and read a reply."""
        if arg_type == MC_TYPE_NONE:
            arg = None
        elif arg_type == MC_TYPE_REG:
            arg = ctypes.c_long(argument)
        elif arg_type == MC_TYPE_LONG:
            arg = ctypes.c_long(argument)
        elif arg_type == MC_TYPE_FLOAT:
            arg = c_float(argument)
        elif arg_type == MC_TYPE_DOUBLE:
            arg = ctypes.c_double(argument)
        elif arg_type == MC_TYPE_STRING:
            arg = c_char(argument)
        else:
            raise McapiException(self.TranslateErrorEx(MCERR_CONSTANT))

        if rpy_type == MC_TYPE_LONG:
            rpy = ctypes.c_long(0)
        elif rpy_type == MC_TYPE_FLOAT:
            rpy = c_float(0)
        elif rpy_type == MC_TYPE_DOUBLE:
            rpy = ctypes.c_double(0)
        elif rpy_type == MC_TYPE_STRING:
            rpy = c_char(0)
        else:
            raise McapiException(self.TranslateErrorEx(MCERR_CONSTANT))

        self.ProcessException(self._dll.pmccmdrpyex(self._handle, axis, command, ctypes.byref(arg), arg_type, ctypes.byref(rpy), rpy_type))
        return rpy.value

    def pmcgetc(self):
        """Read a single character from the controller ASCII interface."""
        char = self._dll.pmcgetc(self._handle)
        self.ProcessException()
        return char

    def pmcgetramex(self, offset, buffer, size):
        """Read directly from controller memory beginning at specific location."""
        self.ProcessException(self._dll.pmcgetramex(self._handle, offset, ctypes.byref(buffer), size))

    def pmcgets(self, buffer, size):
        """Read a null-terminated ASCII string from the controller's ASCII interface."""
        count = self._dll.pmcgets(self._handle, buffer, size)
        self.ProcessException()
        return count

    def pmclock(self, wait_msec):
        """Manually signal the controller semaphore."""
        return self._dll.pmclock(self._handle, wait_msec)

    def pmclookupvar(self, varname, address):
        """Lookup the address of a controller variable."""
        self.ProcessException(self._dll.pmclookupvar(self._handle, varname, address))

    def pmcputc(self, char):
        """Write a single character to the controller ASCII interface."""
        count = self._dll.pmcputc(self._handle, char)
        self.ProcessException()
        return count

    def pmcputramex(self, offset, buffer, size):
        """Write data directly into the controller's memory at a specific location."""
        self.ProcessException(self._dll.pmcputramex(self._handle, offset, ctypes.byref(buffer), size))

    def pmcputs(self, buffer):
        """Write a NULL terminated command string to the controller ASCII interface.

        command buffer must be of type byte, e.g. b'TC\r'"""
        count = self._dll.pmcputs(self._handle, buffer)
        self.ProcessException()
        return count

    def pmcrdy(self):
        """Check the specified controller to see if it is ready to accept a binary command buffer."""
        flag = self._dll.pmcrdy(self._handle)
        self.ProcessException()
        return flag

    def pmcrpyex(self, type):
        """Read a binary reply from the controller."""
        if type == MC_TYPE_LONG:
            rpy = ctypes.c_long(0)
        elif type == MC_TYPE_FLOAT:
            rpy = ctypes.c_float(0)
        elif type == MC_TYPE_DOUBLE:
            rpy = ctypes.c_double(0)
        elif type == MC_TYPE_STRING:
            rpy = ctypes.c_char(0)
        else:
            raise McapiException(self.TranslateErrorEx(MCERR_CONSTANT))

        self.ProcessException(self._dll.pmcrpyex(self._handle, ctypes.byref(rpy), type))
        return rpy.value

    def pmcunlock(self):
        """Manually un-signal the controller semaphore."""
        self._dll.pmcunlock(self._handle)

    #
    # Check for errors and raise exception if needed
    #
    def ProcessException(self, error = -1):
        if error == -1:
            error = self.GetError()
        if error != MCERR_NOERROR and error != MCERR_NOTSUPPORTED:
            raise McapiException(self.TranslateErrorEx(error))
        return error
