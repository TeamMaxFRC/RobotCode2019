package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // Helper classes.
    private CurvatureDrive driveTrain;
    private Lift lift;
    private OscSender oscSender = new OscSender();

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // Is the stream deck in use?
    private static final boolean isStreamDeck = true;
    private static final boolean isPracticeRobot = true;

    // Create the NavX.
    private AHRS navX;

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

    // Create the auxiliary motors.
    private TalonSRX elevatorMaster;
    private TalonSRX elevatorSlaveOne;
    private TalonSRX elevatorSlaveTwo;
    private TalonSRX elevatorSlaveThree;
    private TalonSRX gathererMotor;
    private TalonSRX fourBarMotor;

    // Create the joysticks for the driver and the operator.
    private Joystick driverJoystick = new Joystick(0);
    private Joystick operatorJoystick = new Joystick(1);

    // Create the drive motors.
    private CANSparkMax leftMaster;
    private CANSparkMax rightMaster;
    private CANSparkMax leftSlavePrimary;
    private CANSparkMax leftSlaveSecondary;
    private CANSparkMax rightSlavePrimary;
    private CANSparkMax rightSlaveSecondary;

    // Create and initialize the compressor.
    private Compressor compressor = new Compressor(0);

    // Create the Solenoid and Solenoid state for hatch gathering.
    private DoubleSolenoid hatchSolenoid;
    private boolean previousHatchSwitchValue = false;
    private int hatchSwitchDebounceCounter = 0;

    /**
     * This function is run when the robot is first started up.
     */
    @Override
    public void robotInit() {

        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        // Start the compressor. Toggle this value to turn the compressor off.
        compressor.setClosedLoopControl(true);

        // Initialize all the motor controllers.
        try {

            //--------------------------------------------------------------------------------------------------------------------------------------------------
            // Drive Motors
            //--------------------------------------------------------------------------------------------------------------------------------------------------

            // Initialize the drive motors.
            leftMaster = new CANSparkMax(17, kBrushless);
            leftSlavePrimary = new CANSparkMax(1, kBrushless);
            leftSlaveSecondary = new CANSparkMax(2, kBrushless);

            rightMaster = new CANSparkMax(13, kBrushless);
            rightSlavePrimary = new CANSparkMax(14, kBrushless);
            rightSlaveSecondary = new CANSparkMax(15, kBrushless);

            //--------------------------------------------------------------------------------------------------------------------------------------------------
            // Lift Motors
            //--------------------------------------------------------------------------------------------------------------------------------------------------

            // Initialize the lift motors.
            if (isPracticeRobot) {
                elevatorMaster = new TalonSRX(4);
                elevatorSlaveOne = new TalonSRX(7);
                elevatorSlaveTwo = new TalonSRX(9);
                elevatorSlaveThree = new TalonSRX(8);
                fourBarMotor = new TalonSRX(5);
            } else {
                elevatorMaster = new TalonSRX(7);
                elevatorSlaveOne = new TalonSRX(4);
                elevatorSlaveTwo = new TalonSRX(5);
                elevatorSlaveThree = new TalonSRX(6);
                fourBarMotor = new TalonSRX(9);
            }

            // Create the lift helper class.
            lift = new Lift(elevatorMaster, elevatorSlaveOne, elevatorSlaveTwo, elevatorSlaveThree, fourBarMotor, 1151);

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

            //--------------------------------------------------------------------------------------------------------------------------------------------------
            // Other Initialization
            //--------------------------------------------------------------------------------------------------------------------------------------------------

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

            gathererMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

            // Initialize the NavX.
            navX = new AHRS(SPI.Port.kMXP);

            driveTrain = new CurvatureDrive(leftMaster, leftSlavePrimary, leftSlaveSecondary, rightMaster, rightSlavePrimary, rightSlaveSecondary, navX);

            //Initialize the Solenoid.
            hatchSolenoid = new DoubleSolenoid(0, 1);
            hatchSolenoid.set(DoubleSolenoid.Value.kReverse);

        } catch (Exception Ex) {
            System.out.println("General Initialization Exception: " + Ex.getMessage());
        }

    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        // Update the Limelight's values.
        limelightX = tx.getDouble(0.0);
        limelightY = ty.getDouble(0.0);
        limelightArea = ta.getDouble(0.0);
        limelightTarget = tv.getDouble(0.0) >= 1.0;

        // Always send out error data.
        oscSender.sendOscErrorData(leftMaster, rightMaster, leftSlavePrimary, rightSlavePrimary, leftSlaveSecondary, rightSlaveSecondary);

        // Always send out sensor data.
        oscSender.sendOscSensorData(driveTrain, lift, gathererMotor);

        // Always send out the current data.
        oscSender.sendOscCurrentData(driveTrain, lift, gathererMotor, compressor);

        // Always send out the Limelight data.
        oscSender.sendOscLimelightData(limelightX, limelightY, limelightArea, limelightTarget);

    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

        // Reset the lift's encoder position.
        elevatorMaster.setSelectedSensorPosition(0, 0, 10);
        elevatorMaster.set(ControlMode.MotionMagic, 0);

        // Reset the solenoid position.
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);

        // Report that the console is functional.
        oscSender.writeConsole("Robot has enabled!");
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        //--------------------------------------------------------------------------------------------------------------
        // Drive Controls
        //--------------------------------------------------------------------------------------------------------------

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

        driveTrain.Run(driverVertical, driverTwist, driverJoystick.getRawButton(6), false, driverJoystick.getRawAxis(3));

        //--------------------------------------------------------------------------------------------------------------
        // Operator Controls
        //--------------------------------------------------------------------------------------------------------------
        try {

            //When buttons are pressed on the stream deck, set lift to hatch positions.
            if (isStreamDeck && !driverJoystick.getRawButton(5)) {

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

                // Actuate the solenoid depending on the user button press and the magnetic switch.
                if (operatorJoystick.getRawButtonPressed(11)) {
                    hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
                    hatchSwitchDebounceCounter = 40;

                } else if (operatorJoystick.getRawButtonPressed(12)) {
                    hatchSolenoid.set(DoubleSolenoid.Value.kForward);
                    hatchSwitchDebounceCounter = 40;

                } else if (hatchSwitchDebounceCounter-- <= 0 && currentSwitchState && !previousHatchSwitchValue && hatchSolenoid.get() == DoubleSolenoid.Value.kReverse) {
                    hatchSolenoid.set(DoubleSolenoid.Value.kForward);
                    lift.setLiftPosition(Lift.LiftPosition.ActiveGatherHatch);

                }

                // Store the last magnetic switch value.
                previousHatchSwitchValue = currentSwitchState;

            } else {

                //Disable when tuning PIDs
                //when the right bumper is pressed, set the lift to ball positions.
                if (operatorJoystick.getRawButton(6)) {

                    // If the 'A' button is pressed, then set the ball gathering height.
                    if (operatorJoystick.getRawButton(1)) {
                        lift.setLiftPosition(Lift.LiftPosition.GatheringHatch);

                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarGatheringPositionBall, 0, 0);
                    }

                    // If the 'B' button is pressed, then set the low ball position.
                    else if (operatorJoystick.getRawButton(2)) {
                        lift.setLiftPosition(Lift.LiftPosition.LowBall);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarLowScoreBall, 0, 0);
                    }

                    // If the 'X' button is pressed, then set the middle ball position.
                    else if (operatorJoystick.getRawButton(3)) {
                        lift.setLiftPosition(Lift.LiftPosition.MiddleBall);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarMiddleScoreBall, 0, 0);
                    }

                    // If the 'Y' button is pressed, then set the high ball position.
                    else if (operatorJoystick.getRawButtonPressed(4)) {
                        lift.setLiftPosition(Lift.LiftPosition.HighBall);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarHighScoreBall, 0, 0);
                    }

                }

                // When the right bumper isn't pressed, set the lift to hatch positions.
                else {

                    // If the 'A' button is pressed, then set the hatch gathering position.
                    if (operatorJoystick.getRawButtonPressed(1)) {
                        lift.setLiftPosition(Lift.LiftPosition.GatheringHatch);
                    }

                    // If the 'B' button is pressed, set the low hatch position.
                    else if (operatorJoystick.getRawButtonPressed(2)) {
                        lift.setLiftPosition(Lift.LiftPosition.LowHatch);
                    }

                    // If the 'X' button is pressed, set the middle hatch position.
                    else if (operatorJoystick.getRawButtonPressed(3)) {
                        lift.setLiftPosition(Lift.LiftPosition.MiddleHatch);
                    }

                    // If the 'Y' button is pressed, set the high hatch position.
                    else if (operatorJoystick.getRawButtonPressed(4)) {
                        lift.setLiftPosition(Lift.LiftPosition.HighHatch);
                    }
                }

                // Detect the current state of the magnetic limit switch.
                boolean currentSwitchState = gathererMotor.getSensorCollection().isFwdLimitSwitchClosed();

                // Actuate the solenoid depending on the user button press and the magnetic switch.
                if (operatorJoystick.getRawButtonPressed(5)) {

                    if (hatchSolenoid.get() == DoubleSolenoid.Value.kForward) {
                        hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
                    } else {
                        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
                    }

                    hatchSwitchDebounceCounter = 40;

                } else if (hatchSwitchDebounceCounter-- <= 0 && currentSwitchState && !previousHatchSwitchValue && hatchSolenoid.get() == DoubleSolenoid.Value.kReverse) {
                    hatchSolenoid.set(DoubleSolenoid.Value.kForward);
                    lift.setLiftPosition(Lift.LiftPosition.ActiveGatherHatch);
                }

                // Store the last magnetic switch value.
                previousHatchSwitchValue = currentSwitchState;

                // Run the cargo gatherer based how hard the triggers are being pressed.
                if (operatorJoystick.getRawAxis(2) > 0.1) {
                    gathererMotor.set(ControlMode.PercentOutput, operatorJoystick.getRawAxis(2));
                } else if (operatorJoystick.getRawAxis(3) > 0.1) {
                    gathererMotor.set(ControlMode.PercentOutput, -operatorJoystick.getRawAxis(3));
                } else {
                    gathererMotor.set(ControlMode.PercentOutput, 0.1);
                }

            }

            // Run the lift.
            lift.runLift();

            // Send the four bar logging data.
            oscSender.sendFourBarData(lift);

        } catch (Exception Ex) {
            System.out.println("Exception in operator controls! " + Ex.getMessage());
        }

    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {

    }

    /**
     * This function implements a simple method of generating driving and steering commands
     * based on the tracking data from a limelight camera.
     */
    public void Update_Limelight_Tracking() {
        // These numbers must be tuned for your Robot!  Be careful!
        // TODO: These values need to be adjusted for our robot.
        final double STEER_K = 0.03;                   // how hard to turn toward the target
        final double DRIVE_K = 0.1;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 7.5;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                  // Simple speed limit so we don't drive too fast

        if (!limelightTarget) {
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        // Start with proportional steering
        double steer_cmd = limelightX * STEER_K;
        double steerMaxInput = 0.4;
        m_LimelightSteerCommand = Math.max(Math.min(steer_cmd, steerMaxInput), -steerMaxInput);

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - limelightArea) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }
}