package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final Joystick joystick = new Joystick(0);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /*----Controls----*/
    /* Drive Controls */
    //Up is postive right is postive
    private DoubleSupplier translation = () -> -driver.getLeftY();
    private DoubleSupplier strafe = () -> -driver.getLeftX();
    private DoubleSupplier rotation = () -> driver.getRightX();

    private DoubleSupplier angularJoystickX = rotation;
    private DoubleSupplier angularJoystickY = () -> driver.getRightY();

    //right is postive TODO
    private int rotationAxis = XboxController.Axis.kRightX.value;

    /* Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(joystick, XboxController.Button.kY.value);
    private final JoystickButton zeroEncoders = new JoystickButton(joystick, XboxController.Button.kA.value);
    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // s_Swerve.setDefaultCommand(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> -driver.getLeftY(), 
        //         () -> -driver.getLeftX(), 
        //         () -> driver.getRightX() , 
        //         () -> driver.getRightY(), 
        //         true
        //     )
        // );
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -joystick.getRawAxis(1), 
                () -> -joystick.getRawAxis(0), 
                () -> joystick.getRawAxis(2) , 
                () -> joystick.getRawAxis(3), 
                true
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */
        zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
        zeroEncoders.whenPressed(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d()))); 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new examplePathPlannerAuto(s_Swerve);
    }


    public void resetToAbsloute() {
        s_Swerve.resetToAbsolute();
    }
}
