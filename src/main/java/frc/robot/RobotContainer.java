package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    private final XboxController driver = new XboxController(0);

    private final double deadzone = 0.1;

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> -MathUtil.applyDeadband(driver.getLeftX(), deadzone), () -> MathUtil.applyDeadband(driver.getLeftY(), deadzone), () -> -MathUtil.applyDeadband(driver.getRightX(), deadzone)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Button(driver::getBackButton)
        .whenPressed(drivetrain::zeroGyroscope);
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}