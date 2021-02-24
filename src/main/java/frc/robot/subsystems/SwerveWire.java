package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveWire extends SubsystemBase{

    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    public SwerveWire() {
    }

    private MK3Module[] modules = new MK3Module[] {
        new MK3Module(new CANSparkMax(Constants.MODULE_1_DRIVE_MOTOR, MotorType.kBrushless),
                      new CANSparkMax(Constants.MODULE_1_TURN_MOTOR, MotorType.kBrushless),
                      Constants.MODULE_1_ENCODER,
                      Constants.MODULE_1_DRIVE_REVERSED,
                      Constants.MODULE_1_TURN_REVERSED,
                      Constants.MODULE_1_OFFSET_ANGLE),
        new MK3Module(new CANSparkMax(Constants.MODULE_2_DRIVE_MOTOR, MotorType.kBrushless),
                      new CANSparkMax(Constants.MODULE_2_TURN_MOTOR, MotorType.kBrushless),
                      Constants.MODULE_2_ENCODER,
                      Constants.MODULE_2_DRIVE_REVERSED,
                      Constants.MODULE_2_TURN_REVERSED,
                      Constants.MODULE_2_OFFSET_ANGLE),
        new MK3Module(new CANSparkMax(Constants.MODULE_3_DRIVE_MOTOR, MotorType.kBrushless),
                      new CANSparkMax(Constants.MODULE_3_TURN_MOTOR, MotorType.kBrushless),
                      Constants.MODULE_3_ENCODER,
                      Constants.MODULE_3_DRIVE_REVERSED,
                      Constants.MODULE_3_TURN_REVERSED,
                      Constants.MODULE_3_OFFSET_ANGLE),
        new MK3Module(new CANSparkMax(Constants.MODULE_4_DRIVE_MOTOR, MotorType.kBrushless),
                      new CANSparkMax(Constants.MODULE_4_TURN_MOTOR, MotorType.kBrushless),
                      Constants.MODULE_4_ENCODER,
                      Constants.MODULE_4_DRIVE_REVERSED,
                      Constants.MODULE_4_TURN_REVERSED,
                      Constants.MODULE_4_OFFSET_ANGLE),
    };
    
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(
            Units.inchesToMeters(Constants.ROBOT_LENGTH / 2),
            Units.inchesToMeters(Constants.ROBOT_WIDTH / 2)
        ),
        new Translation2d(
            Units.inchesToMeters(Constants.ROBOT_LENGTH / 2),
            Units.inchesToMeters(-Constants.ROBOT_WIDTH / 2)
        ),
        new Translation2d(
            Units.inchesToMeters(-Constants.ROBOT_LENGTH / 2),
            Units.inchesToMeters(Constants.ROBOT_WIDTH / 2)
        ),
        new Translation2d(
            Units.inchesToMeters(-Constants.ROBOT_LENGTH / 2),
            Units.inchesToMeters(-Constants.ROBOT_WIDTH / 2)
        )
    );

	public void drive(double driveSpeed, double strafeSpeed, double rotationSpeed, boolean fieldRelative, boolean calGyro) {
        
        if(calGyro) {
            gyro.reset(); 
        }
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeed, strafeSpeed, rotationSpeed, Rotation2d.fromDegrees(-gyro.getAngle()))
                          : new ChassisSpeeds(driveSpeed, strafeSpeed, rotationSpeed)
        );

        SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.MAX_SPEED);

        for(int i = 0; i < states.length; i++) {
            MK3Module module = modules[i];
            SwerveModuleState state = states[i];
            SmartDashboard.putNumber(String.valueOf(i), module.getRawAngle());
            SmartDashboard.putNumber("Gyro Angle: ", gyro.getAngle());
            // System.out.println("Module Angle: " + module.getAngle());
            module.setDesiredState(state);
        }
	}
}