package frc.team1071.robot;

import java.text.BreakIterator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
    private static final double configP = 0.1;
    private static final double configD = 0.0;
    private static final double offset = -10;

    // Boolean that tracks if the climber is running.
    boolean climberRunning = false;
    private boolean resetWinch = true;
    private int stage = 0;
    private Timer winchTimer = new Timer();
    public double driveWheels = 0.0;

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
        winch.setInverted(true);
        winch.configContinuousCurrentLimit(30);

        // Default the pistons to the closed state.
        leftPiston.set(false);
        rightPiston.set(false);
    }

    void climberInit() {
        climberRunning = false;
    }

    /**
     * Starts running the climber pistons.
     */
    void toggleClimber() {
        if (climberRunning) {
            // Reset the pistons.
            leftPiston.set(false);
            rightPiston.set(false);

            // Mark the climber as not running.
            climberRunning = false;
            climberWheels.set(ControlMode.PercentOutput, 0.0);

            // Start the time for the winch reset.
            winchTimer.reset();
            winchTimer.start();
        } else {
            // Actuate the pistons.
            leftPiston.set(true);
            rightPiston.set(true);

            // Mark the climber as running.
            climberRunning = true;
            stage = 0;
        }
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

        // If the climber isn't running, retract the pistons and stop the lift.
        if (!climberRunning) {

            leftPiston.set(false);
            rightPiston.set(false);
            winch.set(ControlMode.PercentOutput, 0.0);

        } else {

            // Variable for the output power of the winch.
            double winchOutput = 0;

            // This state machine proceeds through the steps of climbing.
            switch (stage) {

            // Just let the pistons go up in the first stage.
            case 0:
                break;

            // Run the winch down and drive the wheels.
            case 1:
                driveWheels = 0.5;
                winchOutput = -1.0;
                break;
            // Pull the pistons up.
            case 2:
                leftPiston.set(false);
                rightPiston.set(false);
                driveWheels = 0.0;
                break;
            // Run the drive wheels after the pistons come up.
            case 3:
                driveWheels = 0.5;
                break;
            // Pull the winch up.
            case 4:
            default:
                winchOutput = 1.0;
                break;
            }

            // Set the output for the winch and the climber wheels.
            winch.set(ControlMode.PercentOutput, winchOutput);
            climberWheels.set(ControlMode.PercentOutput, driveWheels * 2);

            // Print out some data.
            System.out.println("Stage: " + stage + " Roll: " + navX.getRoll() + " Values: " + winchOutput);
        }

    }

}
