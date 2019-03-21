package frc.team1071.robot;

import java.text.BreakIterator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

class Climber {

    // Climbing motors.
    private TalonSRX winch;
    private TalonSRX climberWheels;

    // Climbing pistons.
    private Solenoid leftPiston;
    private Solenoid rightPiston;

    // Instance of a navX, for running the PID.
    AHRS navX;

    // Climber PID, based on the navX Y tilt.
    PIDController climberPID;

    // PID constant values.
    private static final double configP = .3;
    private static final double configD = 0.0;
    private static final double offset = 0;

    // Boolean that tracks if the climber is running.
    private boolean climberRunning = false;
    private int stage = 0;

    /**
     * Initializes the climber,.
     */
    public Climber(TalonSRX winch, TalonSRX climberWheels, Solenoid leftPiston, Solenoid rightPiston, AHRS navX) {

        // Store the various inputs for the class to use later.
        this.winch = winch;
        this.climberWheels = climberWheels;

        this.leftPiston = leftPiston;
        this.rightPiston = rightPiston;

        this.navX = navX;

        // Enable power compensation for the winch motor.
        winch.enableVoltageCompensation(true);
        winch.setInverted(false);

        // Default the pistons to the closed state.
        leftPiston.set(false);
        rightPiston.set(false);

    }

    /**
     * Starts running the climber pistons.
     */
    void startClimber() {

        // Actuate the pistons.
        leftPiston.set(true);
        rightPiston.set(true);

        // Mark the climber as running.
        climberRunning = true;
        stage = 0;

    }

    void advanceStage() {
        if (climberRunning) {
            stage++;
        }
    }

    /**
     * Runs the climber, trying to maintain a Y value of 0.
     */
    void runClimber() {

        // Check if the climber is running.
        if (climberRunning) {
            double motorValue = 0;
            switch (stage) {
            case 0:
                // Print the value the motor would be set to.
                motorValue = configP * (navX.getRoll() - offset);

                // Set the winch output to the PID value.
                // winch.set(ControlMode.PercentOutput, configP * navX.getRawGyroY());
                break;
            case 1:
                winch.set(ControlMode.PercentOutput, 0.0);
                climberWheels.set(ControlMode.PercentOutput, 0.75);
                break;
            case 2:
                climberWheels.set(ControlMode.PercentOutput, 0.0);
                leftPiston.set(false);
                rightPiston.set(false);
                break;
            case 3:
                climberWheels.set(ControlMode.PercentOutput, 0.75);
                break;
            case 4:
                climberWheels.set(ControlMode.PercentOutput, 0.0);
                winch.set(ControlMode.PercentOutput, -0.5);
                break;
            case 5:
            default:
                winch.set(ControlMode.PercentOutput, 0.0);
                climberRunning = false;
                stage = 0;
                break;
            }
            winch.set(ControlMode.PercentOutput, motorValue);
            System.out.println("Stage: " + stage + " Roll: " + navX.getRoll() + " Values: " + motorValue);

        } else {
            winch.set(ControlMode.PercentOutput, 0);
            climberWheels.set(ControlMode.PercentOutput, 0);
        }

    }

}
