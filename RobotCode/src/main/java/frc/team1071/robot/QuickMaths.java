package frc.team1071.robot;

public class QuickMaths {


    public static double normalizeJoystickWithDeadband(double val, double deadband) {
        val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

        if (val != 0)
            val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}