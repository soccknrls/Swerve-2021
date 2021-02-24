package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class MK3Module {

    private CANSparkMax driveMotor, turnMotor;
    private CANEncoder driveEncoder;
    private Encoder turnEncoder;
    private double offsetAngle;
    private final PIDController drivePIDController = new PIDController(1, 0, 0);
    private final ProfiledPIDController turnPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));

    public MK3Module(CANSparkMax driveMotor, 
                     CANSparkMax turnMotor, 
                     int[] turnEncoderPorts, 
                     boolean driveEncoderReversed, 
                     boolean turnEncoderReversed, 
                     double offsetAngle) {

        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.offsetAngle = offsetAngle;
        this.turnEncoder = new Encoder(turnEncoderPorts[0], turnEncoderPorts[1], turnEncoderReversed);
        driveEncoder = driveMotor.getEncoder();

        // driveEncoder.setInverted(driveEncoderReversed);

        turnEncoder.setDistancePerPulse(Constants.TURN_ENCODER_DIST_PER_PULSE);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);
	}

    public double getAngle() {
        return Math.toDegrees(Math.toRadians(turnEncoder.getRate() - Math.toRadians(offsetAngle)));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.get()));
    }

	public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.get()));

        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        // System.out.println("Drive Output: " + driveOutput);
        final var turnOutput = turnPIDController.calculate(turnEncoder.get(), state.angle.getRadians());
        // System.out.println("Turn Outout: " + turnOutput);

        // driveMotor.set(driveOutput);
        // turnMotor.set(turnOutput);
	}
}