package frc.robot;

public class Constants {
    public static class CANConstants {
        // Climber
        public static final int kClimberId = 1;
        public static final int kClimberFollowerId = 1;

        // L2
        public static final int kL2Id = 1;
    }

    public static class ClimberConstants {
        public static final double kClimberSpeed = 0.5;
        public static final double kMinClimberSpeed = 0;
        public static final double kMaxTorque = 10;
    }

    public static class L2Constants {
        public static final double kL2Speed = 1;
    }
}