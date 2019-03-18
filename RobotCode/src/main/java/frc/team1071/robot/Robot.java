package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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
    private TalonSRX elevatorSlaveOne = new TalonSRX(isPracticeRobot ? 7 : 4);
    private TalonSRX elevatorSlaveTwo = new TalonSRX(isPracticeRobot ? 5 : 9);
    private TalonSRX elevatorSlaveThree = new TalonSRX(isPracticeRobot ? 8 : 6);

    // Create and initialize the four bar motors.
    private TalonSRX fourBarMotorMaster = new TalonSRX(isPracticeRobot ? 5 : 9);
    private TalonSRX fourBarMotorSlave = new TalonSRX(isPracticeRobot ? 3 : 10);

    // Create the air brake solenoid and solenoid state
    private Solenoid airBrakeSolenoid = new Solenoid(1);

    // Create the lift helper class.
    private Lift lift = new Lift(elevatorMaster, elevatorSlaveOne, elevatorSlaveTwo, elevatorSlaveThree,
            fourBarMotorMaster, airBrakeSolenoid, fourBarMotorSlave, isPracticeRobot ? 2464 : 280);

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Gatherer Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------

    private TalonSRX gathererMotor;

    // ---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Climber Subsystem Initialization
    // ---------------------------------------------------------------------------------------------------------------------------------------------------------

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

    // Variables for Limelight targeting function.
    double m_LimelightDriveCommand, m_LimelightSteerCommand;

    // Create the joysticks for the driver and the operator.
    private Joystick driverJoystick = new Joystick(0);
    private Joystick operatorJoystick = new Joystick(1);

    // Create and initialize the compressor.
    private Compressor compressor = new Compressor(0);

    // Create the solenoid and solenoid state for hatch gathering.
    private Solenoid hatchSolenoid;
    private boolean previousHatchSwitchValue = false;
    private int hatchSwitchDebounceCounter = 0;

    private OscSender oscSender = new OscSender();

    /**
     * This function is run when the robot is first started up.
     */
    @Override
    public void robotInit() {

        // Start the compressor. Toggle this value to turn the compressor off.
        compressor.setClosedLoopControl(false);

        // Initialize the solenoids.
        hatchSolenoid = new Solenoid(0);
        hatchSolenoid.set(false);

        airBrakeSolenoid.set(false);

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

        // Initialize the gatherer.
        if (isPracticeRobot) {
            gathererMotor = new TalonSRX(6);
        } else {
            gathererMotor = new TalonSRX(8);
        }

        // Config Gatherer
        gathererMotor.enableCurrentLimit(true);
        gathererMotor.configContinuousCurrentLimit(4);
        gathererMotor.configPeakCurrentDuration(0);
        gathererMotor.configPeakCurrentLimit(0);

        if (!isPracticeRobot) {
            gathererMotor.setInverted(true);
        }

        // Disable all telemetry data for the LiveWindow. This is disabled since it is
        // extremely slow and will cause loop overrun.
        LiveWindow.disableAllTelemetry();

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
        limelightX = tx.getDouble(0.0);
        limelightY = ty.getDouble(0.0);
        limelightArea = ta.getDouble(0.0);
        limelightTarget = tv.getDouble(0.0) >= 1.0;

        // Always send out error data.
        oscSender.sendOscErrorData(leftMaster, rightMaster, leftSlavePrimary, rightSlavePrimary, leftSlaveSecondary,
                rightSlaveSecondary);

        // Always send out sensor data.
        oscSender.sendOscSensorData(driveTrain, lift, gathererMotor);

        // Always send out the current data.
        oscSender.sendOscCurrentData(driveTrain, lift, gathererMotor, compressor);

        // Always send out the Limelight data.
        oscSender.sendOscLimelightData(limelightX, limelightY, limelightArea, limelightTarget);

        // driverJoystick.setRumble(GenericHID.RumbleType.kLeftRumble,
        // (double)Math.abs(driverJoystick.getRawAxis(1)));
        // driverJoystick.setRumble(GenericHID.RumbleType.kRightRumble,
        // (double)Math.abs(driverJoystick.getRawAxis(5)));

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
            driverVertical = QuickMaths.normalizeJoystickWithDeadband(-driverJoystick.getRawAxis(1), 0.05);
            driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getRawAxis(4), 0.05);

        }

        // driverJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        // driverJoystick.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        driveTrain.Run(driverVertical, driverTwist, driverJoystick.getRawButton(6), false,
                driverJoystick.getRawAxis(3));

        // --------------------------------------------------------------------------------------------------------------
        // Operator Controls
        // --------------------------------------------------------------------------------------------------------------
        try {

            // When buttons are pressed on the stream deck, set lift to hatch positions.
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

                // Run the ball gatherer.
                if (operatorJoystick.getRawButton(6)) {
                    gathererMotor.set(ControlMode.PercentOutput, 0.75);
                } else if (operatorJoystick.getRawButton(7)) {
                    gathererMotor.set(ControlMode.PercentOutput, -1.0);
                } else {
                    gathererMotor.set(ControlMode.PercentOutput, .1);
                }

                // Detect the current state of the magnetic limit switch.
                boolean currentSwitchState = gathererMotor.getSensorCollection().isFwdLimitSwitchClosed();

                // Actuate the solenoid depending on the user button press and the magnetic
                // switch.
                if (operatorJoystick.getRawButtonPressed(11)) {
                    hatchSolenoid.set(true);
                    hatchSwitchDebounceCounter = 40;

                } else if (operatorJoystick.getRawButtonPressed(12)) {
                    hatchSolenoid.set(false);
                    hatchSwitchDebounceCounter = 40;

                } else if (hatchSwitchDebounceCounter-- <= 0 && currentSwitchState && !previousHatchSwitchValue
                        && hatchSolenoid.get()) {
                    hatchSolenoid.set(false);
                    lift.setLiftPosition(Lift.LiftPosition.ActiveGatherHatch);
                } else if (operatorJoystick.getRawButton(1)) {
                    hatchSolenoid.set(false);
                    lift.setLiftPosition(Lift.LiftPosition.ActiveGatherHatch);
                }

                // Store the last magnetic switch value.
                previousHatchSwitchValue = currentSwitchState;

            }

            // Run the lift.
            lift.runLift();

        } catch (Exception Ex) {
            System.out.println("Exception in operator controls! " + Ex.getMessage());
        }

    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {

    }

    /**
     * This function implements a simple method of generating driving and steering
     * commands based on the tracking data from a limelight camera.
     */
    public void Update_Limelight_Tracking() {

        // These numbers must be tuned for your Robot! Be careful!
        final double STEER_K = 0.0275; // How hard to turn toward the target.
        final double DRIVE_K = 0.15; // How hard to drive fwd toward the target.
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
}