package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

class Intake {

    // Intake devices.
    private TalonSRX ballIntake;
    private Solenoid hatchIntake;

    // Current percent output for the ball intake.
    double ballIntakePercentOutput;

    /**
     * Initializes the intake, which includes the ball motor and the hatch solenoid.
     */
    public Intake(TalonSRX ballIntake, boolean invertBallIntake, Solenoid hatchIntake) {

        // Store the ball and hatch intake.
        this.ballIntake = ballIntake;
        this.hatchIntake = hatchIntake;

        // Configure the ball intake motor.
        configureBallIntake(invertBallIntake);

    }

    /**
     * Configures the power on the ball intake.
     */
    private void configureBallIntake(boolean invertBallIntake) {

        // Limit the ball intake's current.
        ballIntake.enableCurrentLimit(true);
        ballIntake.configContinuousCurrentLimit(10);
        ballIntake.configPeakCurrentDuration(0);
        ballIntake.configPeakCurrentLimit(0);

        // Invert the ball intake if we're not on the practice robot.
        ballIntake.setInverted(invertBallIntake);

        // Have the hatch solenoid default to closed.
        hatchIntake.set(false);

    }

    /**
     * Gets the current used by the ball intake.
     */
    double getCurrent() {
        return ballIntake.getOutputCurrent();
    }

    /**
     * Gets the current state of the hatch intake limit switch.
     */
    boolean getHatchLimitSwitch() {
        return ballIntake.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * Runs the intake at a standard periodic rate.
     */
    void runIntake() {

        // Set the ball intake to the proper output.
        ballIntake.set(ControlMode.PercentOutput, ballIntakePercentOutput);
    }

    /**
     * Sets the power for the ball intake percent output.
     */
    void setBallIntakePower(double percentOutput) {
        ballIntakePercentOutput = percentOutput;
    }

}
