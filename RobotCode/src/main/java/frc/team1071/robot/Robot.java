package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import frc.team1071.lib.Task;
import java.util.function.Consumer;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // Boolean that determines if we're on the practice robot, or the real robot.
    static final boolean isPracticeRobot = true;

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Drive Train Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------

    // Create and initialize the drive motors.
    private CANSparkMax leftMaster = new CANSparkMax(17, kBrushless);
    private CANSparkMax leftSlavePrimary = new CANSparkMax(1, kBrushless);
    private CANSparkMax leftSlaveSecondary = new CANSparkMax(2, kBrushless);
    private CANSparkMax rightMaster = new CANSparkMax(13, kBrushless);
    private CANSparkMax rightSlavePrimary = new CANSparkMax(14, kBrushless);
    private CANSparkMax rightSlaveSecondary = new CANSparkMax(15, kBrushless);

    // Create the NavX.
    private AHRS navX = new AHRS(SPI.Port.kMXP);

    // Helper classes.
    private CurvatureDrive driveTrain = new CurvatureDrive(leftMaster, leftSlavePrimary, leftSlaveSecondary,
            rightMaster, rightSlavePrimary, rightSlaveSecondary, navX);

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Lift Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------

    // Create and initialize the elevator motors.
    private TalonSRX elevatorMaster = new TalonSRX(isPracticeRobot ? 4 : 7);
    private TalonSRX elevatorSlaveOne = new TalonSRX(isPracticeRobot ? 7 : 6);
    private TalonSRX elevatorSlaveTwo = new TalonSRX(isPracticeRobot ? 9 : 5);
    private TalonSRX elevatorSlaveThree = new TalonSRX(isPracticeRobot ? 8 : 4);

    // Create and initialize the four bar motors.
    private TalonSRX fourBarMotorMaster = new TalonSRX(isPracticeRobot ? 5 : 9);
    private TalonSRX fourBarMotorSlave = new TalonSRX(isPracticeRobot ? 3 : 10);

    // Create the air brake solenoid and solenoid state
    private Solenoid airBrakeSolenoid = new Solenoid(1);

    // Create the lift helper class.
    private Lift lift = new Lift(elevatorMaster, elevatorSlaveOne, elevatorSlaveTwo, elevatorSlaveThree,
            fourBarMotorMaster, airBrakeSolenoid, fourBarMotorSlave, isPracticeRobot ? 2464 : 280, isPracticeRobot);

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Gatherer Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------

    // Create and initialize the ball gatherer motor.
    private TalonSRX ballIntake = new TalonSRX(isPracticeRobot ? 6 : 8);

    // Create the solenoid and solenoid state for hatch gathering.
    private Solenoid hatchSolenoid = new Solenoid(0);

    private boolean previousHatchSwitchValue = false;
    private int hatchSwitchDebounceCounter = 0;

    // Create the intake subsystem.
    private Intake intake = new Intake(ballIntake, !isPracticeRobot, hatchSolenoid);

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Climber Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------

    // Create and initialize the winch and climber wheels.
    // TODO: Determine the proper IDs for the talons.
    private TalonSRX winch = new TalonSRX(isPracticeRobot ? 10 : 3);
    private TalonSRX climberWheels = new TalonSRX(isPracticeRobot ? 12 : 12);

    // Create and initialize the solenoids for the climber pistons.
    private Solenoid leftClimberPiston = new Solenoid(2);
    private Solenoid rightClimberPiston = new Solenoid(3);

    // Create the climber subsystem.
    private Climber climber = new Climber(winch, climberWheels, leftClimberPiston, rightClimberPiston, navX);

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Testing Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    private Timer testTimer = new Timer();
    int testStage = 0;

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Vision Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------

    // Create the Limelight.
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry tv = table.getEntry("tv");
    private double limelightX, limelightY, limelightArea;
    private boolean limelightTarget;
    private Task backgroundTask;

    // Variables for Limelight targeting function.
    double m_LimelightDriveCommand, m_LimelightSteerCommand;

    // Create the joysticks for the driver and the operator.
    private Joystick driverJoystick = new Joystick(0);
    private double leftRumble, rightRumble, pulse;
    private int rumbleTimer = 0;
    private Joystick operatorJoystick = new Joystick(1);

    // Create and initialize the compressor.
    private Compressor compressor = new Compressor(0);

    private OscSender oscSender = new OscSender();

    /**
     * This function is run when the robot is first started up.
     */
    @Override
    public void robotInit() {

        // Start the compressor. Toggle this value to turn the compressor off.
        compressor.setClosedLoopControl(true);

        if (isPracticeRobot) {

            // Set the PID values for the lift.
            elevatorMaster.config_kF(0, 0.32058916);
            elevatorMaster.config_kP(0, 1.4);
            elevatorMaster.config_kD(0, 2.8);

            // Establish the cruise velocity and max acceleration for motion magic.
            elevatorMaster.configMotionCruiseVelocity(2900);
            elevatorMaster.configMotionAcceleration(5200);

        } else {

            // Set the PID values for the lift.
            elevatorMaster.config_kF(0, 0.32058916);
            elevatorMaster.config_kP(0, 0.30);
            elevatorMaster.config_kD(0, 0.60);

            // Establish the cruise velocity and max acceleration for motion magic.
            elevatorMaster.configMotionCruiseVelocity(2900);
            elevatorMaster.configMotionAcceleration(5200);

        }

        // Reset the lift's encoder position.
        elevatorMaster.setSelectedSensorPosition(0, 0, 10);
        elevatorMaster.set(ControlMode.MotionMagic, 0);

        // --------------------------------------------------------------------------------------------------------------------------------------------------
        // Other Initialization
        // --------------------------------------------------------------------------------------------------------------------------------------------------

        // Disable all telemetry data for the LiveWindow. This is disabled since it is
        // extremely slow and will cause loop overrun.
        LiveWindow.disableAllTelemetry();

        Consumer<Void> taskFunction = (t) -> {
            SendPeriodicOscData();
        };

        backgroundTask = new Task(.05, taskFunction);
    }

    public void SendPeriodicOscData() {
        // Always send out error data.
        oscSender.sendOscErrorData(leftMaster, rightMaster, leftSlavePrimary, rightSlavePrimary, leftSlaveSecondary,
                rightSlaveSecondary);

        // Always send out sensor data.
        oscSender.sendOscSensorData(driveTrain, lift, intake);

        // Always send out the current data.
        oscSender.sendOscCurrentData(driveTrain, lift, intake, compressor);

        // Always send out the Limelight data.
        oscSender.sendOscLimelightData(limelightX, limelightY, limelightArea, limelightTarget);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        // Update the Limelight's values.
        try {
            limelightX = tx.getDouble(0.0);
            limelightY = ty.getDouble(0.0);
            limelightArea = ta.getDouble(0.0);
            limelightTarget = tv.getDouble(0.0) >= 1.0;
        } catch (Exception Ex) {
            System.out.println("Exception getting Limelight data: " + Ex);
        }

        // Set the controller rumble.
        try {
            double rumble = .2 * Math.abs(navX.getRawAccelY() - 0.025);
            // driverJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
            // driverJoystick.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
        } catch (Exception Ex) {
            System.out.println("Exception in setting controller rumble: " + Ex);
        }

        lift.LiftPeriodic();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        teleopInit();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        teleopPeriodic();
    }

    @Override
    public void teleopInit() {

        // Reset the solenoid position.
        hatchSolenoid.set(false);

        // Report that the console is functional.
        oscSender.writeConsole("Robot has enabled!");

        lift.LiftInit();

        // Disable climbing mode if it's enabled.
        climber.climberInit();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        // --------------------------------------------------------------------------------------------------------------
        // Drive Controls
        // --------------------------------------------------------------------------------------------------------------

        // Determine the proper motor values based on the joystick data.
        Update_Limelight_Tracking();

        double driverVertical = 0;
        double driverTwist = 0;

        if (driverJoystick.getRawButton(5)) {

            if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0) != 1) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
            }

            if (limelightTarget) {

                driverVertical = m_LimelightDriveCommand;
                driverTwist = m_LimelightSteerCommand;
            }
        } else {

            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
            if (Math.abs(climber.driveWheels) > 0.0) {
                driverVertical = Math.abs(climber.driveWheels) * 0.75;
                driverTwist = 0.0;
            } else {
                driverVertical = QuickMaths.normalizeJoystickWithDeadband(-driverJoystick.getRawAxis(1), 0.05);
                driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getRawAxis(4), 0.05);
            }
        }

        // TODO: Delete after testing the climber.
        // winch.set(ControlMode.PercentOutput, driverJoystick.getRawAxis(3));

        driveTrain.Run(driverVertical, driverTwist, driverJoystick.getRawButton(6), false,
                driverJoystick.getRawAxis(3));

        // -----------------------------------------------------------------------------------------------------------------------------------------------------
        // Operator Controls
        // -----------------------------------------------------------------------------------------------------------------------------------------------------
        try {

            rumbleUpdate();

            // Only run the operator controls when not in vision tracking mode.
            if (!driverJoystick.getRawButton(5)) {

                // Set lift position based on the button pressed on the stream deck.
                if (operatorJoystick.getRawButton(4)) {
                    lift.setLiftPosition(Lift.LiftPosition.HighHatch);
                } else if (operatorJoystick.getRawButton(9)) {
                    lift.setLiftPosition(Lift.LiftPosition.MiddleHatch);
                } else if (operatorJoystick.getRawButton(14)) {
                    lift.setLiftPosition(Lift.LiftPosition.LowHatch);
                } else if (operatorJoystick.getRawButton(5)) {
                    lift.setLiftPosition(Lift.LiftPosition.HighBall);
                } else if (operatorJoystick.getRawButton(10)) {
                    lift.setLiftPosition(Lift.LiftPosition.MiddleBall);
                } else if (operatorJoystick.getRawButton(15)) {
                    lift.setLiftPosition(Lift.LiftPosition.LowBall);
                } else if (operatorJoystick.getRawButton(13)) {
                    lift.setLiftPosition(Lift.LiftPosition.GatheringHatch);
                } else if (operatorJoystick.getRawButton(8)) {
                    lift.setLiftPosition(Lift.LiftPosition.GatheringBall);
                } else if (operatorJoystick.getRawButton(3)) {
                    lift.setLiftPosition(Lift.LiftPosition.AirBrake);
                }

                // Set the percent output for the ball gatherer.
                if (operatorJoystick.getRawButton(6)) {
                    intake.setBallIntakePower(1.0);
                } else if (operatorJoystick.getRawButton(7)) {
                    intake.setBallIntakePower(-1.0);
                } else {
                    intake.setBallIntakePower(0.2);
                }

                // Update the state of the climber.
                if (operatorJoystick.getRawButtonPressed(2)) {
                    climber.toggleClimber();
                } else if (operatorJoystick.getRawButtonPressed(1)) {
                    climber.advanceStage();
                }

                // Run the climber.
                climber.runClimber();

                // Detect the current state of the magnetic limit switch.
                boolean currentSwitchState = intake.getHatchLimitSwitch();

                // Actuate the solenoid depending on the user button press and the magnetic
                // switch.
                if (operatorJoystick.getRawButtonPressed(11)) {
                    setRumble(50);
                    hatchSolenoid.set(true);
                    hatchSwitchDebounceCounter = 40;
                } else if (hatchSwitchDebounceCounter-- <= 0 && currentSwitchState && !previousHatchSwitchValue
                        && hatchSolenoid.get()) {
                    setRumble(50);
                    hatchSolenoid.set(false);
                    lift.setLiftPosition(Lift.LiftPosition.ActiveGatherHatch);
                } else if (operatorJoystick.getRawButton(12)) {
                    setRumble(50);
                    hatchSolenoid.set(false);
                    lift.setLiftPosition(Lift.LiftPosition.ActiveGatherHatch);
                }

                // Store the last magnetic switch value.
                previousHatchSwitchValue = currentSwitchState;

                // Run the various subsystems.
                intake.runIntake();
                lift.runLift();
            }

        } catch (

        Exception Ex) {
            System.out.println("Exception in operator controls! " + Ex.getMessage());
        }

    }

    @Override
    public void testInit() {
        teleopInit();
        testStage = 0;
        testTimer.reset();
        testTimer.start();
    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {
        // Update the timer.
        var elapsedTime = testTimer.get();
        System.out.println(elapsedTime);
        String stageName = "";

        // Attempt to run the test procedure.
        try {
            if (testStage >= 0) {
                switch (testStage) {
                case 0:
                    stageName = "Shweem";
                    break;
                default:
                    break;
                }
            }
        } catch (Exception e) {
            oscSender.writeConsole("Exception in " + stageName + ": " + e.toString());
            oscSender.writeConsole("Stopping test procedure.");
            testStage = -1;
        }
    }

    /**
     * This function implements a simple method of generating driving and steering
     * commands based on the tracking data from a limelight camera.
     */
    public void Update_Limelight_Tracking() {

        // These numbers must be tuned for your Robot! Be careful!
        final double STEER_K = 0.04; // How hard to turn toward the target.
        final double DRIVE_K = 0.2; // How hard to drive fwd toward the target.
        final double DESIRED_TARGET_AREA = 6.5; // Area of the target when the robot reaches the wall.
        final double MAX_DRIVE = 1.0; // Simple speed limit so we don't drive too fast.

        if (!limelightTarget) {
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        // Start with proportional steering
        double steer_cmd = limelightX * STEER_K;
        double steerMaxInput = 0.4;
        m_LimelightSteerCommand = Math.max(Math.min(steer_cmd, steerMaxInput), -steerMaxInput);

        // Try to drive forward until the target area reaches our desired area.
        double drive_cmd = (DESIRED_TARGET_AREA - limelightArea) * DRIVE_K;

        // Don't let the robot drive too fast into the goal.
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        }

        m_LimelightDriveCommand = drive_cmd;
    }

    public void setRumble(int duration) {
        rumbleTimer = duration;
    }

    public void rumbleUpdate() {
        if (rumbleTimer > 0) {
            pulse = Math.sin(.15 * Math.PI * rumbleTimer);
            leftRumble = pulse;
            rightRumble = pulse;
            rumbleTimer--;
        } else {
            leftRumble = 0;
            rightRumble = 0;
        }
        driverJoystick.setRumble(RumbleType.kLeftRumble, leftRumble);
        driverJoystick.setRumble(RumbleType.kRightRumble, rightRumble);
    }
}