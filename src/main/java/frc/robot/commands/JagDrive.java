package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveWire;
import frc.robot.Constants;

public class JagDrive extends CommandBase{
    
    private final SwerveWire swerveWire;
    private final Joystick joystick;

    public JagDrive(SwerveWire swerveWire, Joystick joystick) {
        this.swerveWire = swerveWire;
        this.joystick = joystick;

        addRequirements(swerveWire);
    }

    @Override
    public void execute() {
        final double driveSpeed = joystick.getRawAxis(Constants.DRIVE_AXIS) * Constants.MAX_SPEED;
        SmartDashboard.putNumber("Drive Speed: ", driveSpeed);
        final double strafeSpeed = joystick.getRawAxis(Constants.STRAFE_AXIS) * Constants.MAX_SPEED;
        SmartDashboard.putNumber("Strafe Speed: ",  strafeSpeed);
        final double rotationSpeed = joystick.getRawAxis(Constants.ROTATION_AXIS) * Constants.MAX_ROTATION;
        SmartDashboard.putNumber("Rotation Speed: ", rotationSpeed);

        swerveWire.drive(driveSpeed, strafeSpeed, rotationSpeed, true, false);
    }
}
