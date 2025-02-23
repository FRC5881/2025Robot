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
        public static final int kPivotMotor = 1;
        public static final int kIntakeMotor = 2;

        //L1
        public static final int kMotorL1 = 5;

        //L2

        //Deep Climb
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCopilotControllerPort = 1;
    }

    public static class PositionConstants{
    //TODO: Change values
    //Testing values
    public static final Rotation2d kAlgaeArmTestDown = new Rotation2d(-Math.PI/2);
    public static final Rotation2d kAlgaeArmHorizontal = new Rotation2d(0.0);
    public static final Rotation2d kAlgaeArmUp = new Rotation2d(Math.PI/2);
    //Algae Arm values
    public static final Rotation2d kAlgaeArmOut = new Rotation2d(Math.toRadians(-30));
    public static final Rotation2d kAlgaeArmDown = new Rotation2d(Math.toRadians(-60));
    public static final Rotation2d kAlgaeArmAway = new Rotation2d(Math.toRadians(80));
    //Algae Intake values
    public static final double kAlgaeIntakeIn = 3000;
    public static final double kAlgaeIntakeOut = -3000;
    public static final double kAlgaeIntakeHold = 0;
    //Coral L1
    public static final Rotation2d kDropReadyL1 = new Rotation2d(Math.toRadians(30));
    public static final Rotation2d kDropL1 = new Rotation2d(Math.toRadians(30));
    public static final Rotation2d kHomeL1 = new Rotation2d(Math.toRadians(-90));
    public static final Rotation2d kIntakeReadyL1 = new Rotation2d(Math.toRadians(50)); //The chute is 55 degrees
    }
}
