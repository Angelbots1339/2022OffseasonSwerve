package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.XYPair;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSupX;
    private DoubleSupplier rotationSupY;
    private boolean isFieldRelative;
    private BooleanSupplier turnModeButtonPressed;

    private boolean isTurnModePIDToAngle = false;

    // private SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(TeleopSwerveConstants.turnToAngleKS,
    //         TeleopSwerveConstants.turnToAngleKV, TeleopSwerveConstants.turnToAngleKA);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSupX, DoubleSupplier rotationSupY, BooleanSupplier turnModeButtonPressed, boolean isFieldRelative) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSupX = rotationSupX;
        this.rotationSupY = rotationSupY;
        this.isFieldRelative = isFieldRelative;
        this.turnModeButtonPressed = turnModeButtonPressed;
    }

    private Rotation2d desiredRotation = new Rotation2d(0);
    private Rotation2d lastRotation = new Rotation2d(0);

    @Override
    public void execute() {

      if(turnModeButtonPressed.getAsBoolean()) isTurnModePIDToAngle = !isTurnModePIDToAngle;

      XYPair translationPair = mapJoysticks(new XYPair(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband)));
        
        if(Math.abs(rotationSupX.getAsDouble()) > 0.4 || Math.abs(rotationSupY.getAsDouble()) > 0.4){
            desiredRotation = ConvertJoystickToAngle(rotationSupX.getAsDouble(), rotationSupY.getAsDouble());
            lastRotation = desiredRotation;
        } else {
            desiredRotation = lastRotation;
        }
        
        SmartDashboard.putNumber("joystick", desiredRotation.getDegrees());
        double rotationVal = MathUtil.applyDeadband(rotationSupX.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        if(isTurnModePIDToAngle) {
            s_Swerve.angularDrive(
                new Translation2d(translationPair.X, translationPair.Y).times(Constants.Swerve.maxSpeed),
                desiredRotation,
                isFieldRelative, // Field relative
                true);
        } else{
        s_Swerve.drive(
                new Translation2d(translationPair.X, translationPair.Y).times(Constants.Swerve.maxSpeed),
                (rotationVal * Constants.Swerve.maxAngularVelocity),
                isFieldRelative, // Field relative
                true);
        }
    }

    /**
     * @param x position from joystick 
     * @param y position from joystick
     * @return Rotation2d
     * <pre>
        Joystick input:
                 (y:-)
                   |
                   |
         (x:-)-----|------(x:+)
                   |
                   |
                 (y:+)

     intial angle (-180 -> 180):
               -180/180°
                 (x:-)
                   |
                   |
     -90°(y:-)-----|------(y:+) 90°
                   | ノ
                   |   θ
                 (x:+)
                  0°

transformed(+180) desired angle (0 -> 360): same angle as gyro
                   0°
                 (x:+)
               θ   |
                 / |
      90°(y:+)-----|------(y:-)270°
                   | 
                   |  
                 (x:-)
                  180°
                  </pre>
     */
    public static Rotation2d ConvertJoystickToAngle(double x, double y){
        return new Rotation2d(Math.atan2(x, y) + Math.PI);
    }

    /**
     * 
     * Maps the joystick values from a circle to a square.
     * Otherwise the joystick hardware reads values in a square from (-1, -1) to (1, 1), but because of the circular joystick shape you can never reach the corners of this square.
     * 
     * @param sticks The current X and Y positions for the given stick
     * @return
     */
    public static XYPair mapJoysticks(XYPair sticks) {
      double X = sticks.X * Math.sqrt(1 - ((sticks.Y * sticks.Y) / 2));
      double Y = sticks.Y * Math.sqrt(1 - ((sticks.X * sticks.X) / 2));
      return new XYPair(X, Y);
  }
}
