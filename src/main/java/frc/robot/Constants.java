package frc.robot;

public class Constants {
    public static class CANConstants {
        // Climber
        public static final int kClimberId = 1;
        public static final int kClimberFollowerId = 1;
        // Intake
        public static final int kIntakeId = 8;
    }

    public static class ClimberConstants {
        public static final double kClimberSpeed = 0.5;
        public static final double kMinClimberSpeed = 0;
        public static final double kMaxTorque = 10;
    }

    public static class IntakeConstants {
        public static final double kIntakeSpeed = 1;
    }
}