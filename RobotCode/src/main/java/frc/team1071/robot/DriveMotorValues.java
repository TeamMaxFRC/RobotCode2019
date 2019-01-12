package frc.team1071.robot;

public class DriveMotorValues {
    public double leftDrive;
    public double rightDrive;

    public DriveMotorValues(double leftDrive, double rightDrive) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
    }

    public static DriveMotorValues NEUTRAL = new DriveMotorValues(0, 0);
}