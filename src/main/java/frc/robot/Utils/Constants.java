package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

    public static class AnalogInputConstants {
        public static final int kAlgaeSensor = 0;
        public static final int kCoralSensorL1 = 1;
        public static final int kCoralSensorL2 = 2;
    }

    public static class CANConstants {
        //Swerve Drive

        //Arm
        public static final int kPivotMotor = 10;
        public static final int kIntakeMotor = 11;

        //L1
        public static final int kMotorL1 = 20;

        //L2

        //Deep Climb
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCopilotControllerPort = 1;
    }

    public static class PositionConstants{
    //TODO: Change values
    //Initialization values
    public static final double kAlgaeDegreeOffset = -60;
    //Algae Arm values
    //Should be in ()
    public static final Rotation2d kAlgaeArmOut = new Rotation2d(Math.toRadians(0));
    public static final Rotation2d kAlgaeArmDown = new Rotation2d(Math.toRadians(-50));
    public static final Rotation2d kAlgaeArmAway = new Rotation2d(Math.toRadians(80));
    //Algae Intake values
    public static final double kAlgaeIntakeIn = 3000;
    public static final double kAlgaeIntakeOut = -3000;
    public static final double kAlgaeIntakeHold = 0;
    //Coral L1
    public static final Rotation2d kDropReadyL1 = new Rotation2d(Math.toRadians(95));
    public static final Rotation2d kDropL1 = new Rotation2d(Math.toRadians(40));
    public static final Rotation2d kHomeL1 = new Rotation2d(Math.toRadians(0));
    public static final Rotation2d kIntakeReadyL1 = new Rotation2d(Math.toRadians(170)); //The chute is 55 degrees
    }
}
