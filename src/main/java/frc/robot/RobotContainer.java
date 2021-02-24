package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.JagDrive;
import frc.robot.subsystems.SwerveWire;

public class RobotContainer {

    private final Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);
    private final SwerveWire swerveWire = new SwerveWire();

    public RobotContainer() {
        swerveWire.setDefaultCommand(new JagDrive(swerveWire, joystick));
    }
}
