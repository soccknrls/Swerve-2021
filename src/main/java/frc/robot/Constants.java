package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    
    /* Operator Interaction */
    public static final int JOYSTICK_PORT = 0;
    public static final int DRIVE_AXIS = 1;
    public static final int ROTATION_AXIS = 2;
    public static final int STRAFE_AXIS = 0;
    public static final int DRIVE_AXIS_LIMITER = 6;
    public static final int ROTATION_AXIS_LIMITER = 6;
    public static final int STRAFE_AXIS_LIMITER = 6;

    /* Module CAN Assignments */
    public static final int MODULE_1_DRIVE_MOTOR = 1;
    public static final int MODULE_1_TURN_MOTOR = 2;
    public static final int MODULE_2_DRIVE_MOTOR = 3;
    public static final int MODULE_2_TURN_MOTOR = 4;
    public static final int MODULE_3_DRIVE_MOTOR = 5;
    public static final int MODULE_3_TURN_MOTOR = 6;
    public static final int MODULE_4_DRIVE_MOTOR = 7;
    public static final int MODULE_4_TURN_MOTOR = 8;

    /* Encoder DIO Assignment */
    public static final int[] MODULE_1_ENCODER = new int[] {0,1};
    public static final int[] MODULE_2_ENCODER = new int[] {2,3};
    public static final int[] MODULE_3_ENCODER = new int[] {4,5};
    public static final int[] MODULE_4_ENCODER = new int[] {6,7};

    /* Module Offset Angle */
    public static final double MODULE_1_OFFSET_ANGLE = 0.0;
    public static final double MODULE_2_OFFSET_ANGLE = 0.0;
    public static final double MODULE_3_OFFSET_ANGLE = 0.0;
    public static final double MODULE_4_OFFSET_ANGLE = 0.0;

    /* Encoder Information */
    public static final int ENCODER_CPR = 4096;
    public static final double TURN_ENCODER_DIST_PER_PULSE = (2 * Math.PI) / (double) ENCODER_CPR;
    public static final boolean MODULE_1_DRIVE_REVERSED = false;
    public static final boolean MODULE_1_TURN_REVERSED = false;
    public static final boolean MODULE_2_DRIVE_REVERSED = false;
    public static final boolean MODULE_2_TURN_REVERSED = false;
    public static final boolean MODULE_3_DRIVE_REVERSED = false;
    public static final boolean MODULE_3_TURN_REVERSED = false;
    public static final boolean MODULE_4_DRIVE_REVERSED = false;
    public static final boolean MODULE_4_TURN_REVERSED = false;

    /* Drivetrain Information */
    public static final double MAX_SPEED = -.9;
    public static final double MAX_ROTATION = Math.PI;
    public static final double ROBOT_WIDTH = 19.0;
    public static final double ROBOT_LENGTH = 19.0;

    /* PID Variables */
    public static final double DRIVE_P = 0.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_FF = 0.0;
    public static final double TURN_P = 0.0;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.0;
}
