package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
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

    private CurvatureDrive driveTrain;
    private FourBar fourBarLift;

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
    private TalonSRX liftMaster;
    private TalonSRX liftSlavePrimary;
    private TalonSRX liftSlaveSecondary;
    private TalonSRX liftSlaveTertiary;
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

    // Create an OSC helper that will send all the data.
    OscHelper oscHelper = new OscHelper();

    /**
     * Function that configures a lift motor's power.
     *
     * @param liftTalon The talon being power limited.
     */
    private static void configLiftMotorPower(TalonSRX liftTalon) {
        liftTalon.configPeakCurrentLimit(0);
        liftTalon.configPeakCurrentDuration(0);
        liftTalon.configContinuousCurrentLimit(15);
        liftTalon.enableCurrentLimit(true);
        liftTalon.configVoltageCompSaturation(12);
        liftTalon.enableVoltageCompensation(true);
    }

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
            //----------------------------------------------------------------------------------------------------------
            // Drive Motors
            //----------------------------------------------------------------------------------------------------------

            // Initialize the drive motors.
            leftMaster = new CANSparkMax(17, kBrushless);
            leftSlavePrimary = new CANSparkMax(1, kBrushless);
            leftSlaveSecondary = new CANSparkMax(2, kBrushless);

            rightMaster = new CANSparkMax(13, kBrushless);
            rightSlavePrimary = new CANSparkMax(14, kBrushless);
            rightSlaveSecondary = new CANSparkMax(15, kBrushless);

            //----------------------------------------------------------------------------------------------------------
            // Lift Motors
            //----------------------------------------------------------------------------------------------------------

            // Initialize the lift motors.
            if (isPracticeRobot) {
                liftMaster = new TalonSRX(4);
                liftSlavePrimary = new TalonSRX(7);
                liftSlaveSecondary = new TalonSRX(9);
                liftSlaveTertiary = new TalonSRX(8);
            } else {
                liftMaster = new TalonSRX(7);
                liftSlavePrimary = new TalonSRX(4);
                liftSlaveSecondary = new TalonSRX(5);
                liftSlaveTertiary = new TalonSRX(6);
            }

            liftMaster.enableVoltageCompensation(true);
            liftSlavePrimary.enableVoltageCompensation(true);
            liftSlaveSecondary.enableVoltageCompensation(true);
            liftSlaveTertiary.enableVoltageCompensation(true);

            // Configure lift motor power.
            configLiftMotorPower(liftMaster);
            configLiftMotorPower(liftSlavePrimary);
            configLiftMotorPower(liftSlaveSecondary);
            configLiftMotorPower(liftSlaveTertiary);

            // Have lift slaves follow the master.
            liftSlavePrimary.follow(liftMaster);
            liftSlaveSecondary.follow(liftMaster);
            liftSlaveTertiary.follow(liftMaster);

            // Invert the lift motors.
            liftMaster.setInverted(true);
            liftSlavePrimary.setInverted(true);
            liftSlaveSecondary.setInverted(true);
            liftSlaveTertiary.setInverted(true);

            // Invert the lift encoder.
            liftMaster.setSensorPhase(true);

            if (isPracticeRobot) {
                // Set the PID values for the lift.
                liftMaster.config_kF(0, 0.32058916);
                liftMaster.config_kP(0, 1.4);
                liftMaster.config_kD(0, 2.8);

                // Establish the cruise velocity and max acceleration for motion magic.
                liftMaster.configMotionCruiseVelocity(2900);
                liftMaster.configMotionAcceleration(5200);
            } else {
                // Set the PID values for the lift.
                liftMaster.config_kF(0, 0.32058916);
                liftMaster.config_kP(0, 0.30);
                liftMaster.config_kD(0, 0.60);

                // Establish the cruise velocity and max acceleration for motion magic.
                liftMaster.configMotionCruiseVelocity(2900);
                liftMaster.configMotionAcceleration(5200);
            }

            //----------------------------------------------------------------------------------------------------------
            // Four Bar Motor
            //----------------------------------------------------------------------------------------------------------

            // Initialize the four bar motor.
            if (isPracticeRobot) {
                fourBarMotor = new TalonSRX(5);
            } else {
                fourBarMotor = new TalonSRX(9);
            }

            fourBarLift = new FourBar(fourBarMotor, 1151);

            //----------------------------------------------------------------------------------------------------------
            // Other Initialization
            //----------------------------------------------------------------------------------------------------------

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

        // System.out.println(fourBarMotor.getPosition());
        // System.out.println(fourBarMotor.getSelectedSensorPosition());
        //System.out.println(liftMaster.getSelectedSensorPosition());

        fourBarLift.RobotPeriodic();

        // Always send out error data.
        oscHelper.sendOscErrorData(leftMaster, rightMaster, leftSlavePrimary, rightSlavePrimary, leftSlaveSecondary, rightSlaveSecondary);

        // Always send out sensor data.
        oscHelper.sendOscSensorData(liftMaster, fourBarLift, gathererMotor, driveTrain);

        // Always send out the current data.
        oscHelper.sendOscCurrentData(leftMaster, rightMaster, leftSlavePrimary, rightSlavePrimary, leftSlaveSecondary, rightSlaveSecondary, liftMaster, liftSlavePrimary, liftSlaveSecondary, liftSlaveTertiary, fourBarMotor, gathererMotor, compressor);

        // Always send out the Limelight data.
        oscHelper.sendOscLimelightData(limelightX, limelightY, limelightArea, limelightTarget);
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
        liftMaster.setSelectedSensorPosition(0, 0, 10);
        liftMaster.set(ControlMode.MotionMagic, 0);

        // Reset the solenoid position.
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);

        fourBarLift.TeleopInit();

        // Report that the console is functional.
        oscHelper.writeConsole("Robot has enabled!");
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
            // Four bar positions in degrees.
            double fourBarGatheringPositionDegrees = 30;

            double fourBarLowBallDegrees = 110;
            double fourBarMiddleBallDegrees = 110;
            double fourBarHighBallDegrees = 110;

            double fourBarLowHatchDegrees = 50;
            double fourBarMiddleHatchDegrees = 50;
            double fourBarHighHatchDegrees = 50;

            // Lift ball set positions.
            int liftGatheringPositionBall = 0;
            int liftLowScoreBall = 0;
            int liftMiddleScoreBall = 15000;
            int liftHighScoreBall = 25300;

            // Lift hatch set positions.
            int liftGatheringPositionHatch = 0;
            int liftLowScoreHatch = 0;
            int liftMiddleScoreHatch = 15000;
            int liftHighScoreHatch = 25200;
            int liftInstantGatherHatch = 2000;

            // Lift set to gathering position.
            int liftGatheringPosition = 0;

            //When buttons are pressed on the stream deck, set lift to hatch positions.
            if (isStreamDeck && !driverJoystick.getRawButton(5)) {

                // If the '4' button is pressed, then lift sets to top hatch position.
                if (operatorJoystick.getRawButtonPressed(4)) {
                    liftMaster.set(ControlMode.MotionMagic, liftHighScoreHatch);
                    fourBarLift.SetPositionDegrees(fourBarHighHatchDegrees);
                }

                // If the '9' button is pressed, then lift sets to middle hatch position.
                if (operatorJoystick.getRawButtonPressed(9)) {
                    liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreHatch);
                    fourBarLift.SetPositionDegrees(fourBarMiddleHatchDegrees);
                }

                // If the '14' button is pressed, then lift sets to low hatch position.
                if (operatorJoystick.getRawButtonPressed(14)) {
                    liftMaster.set(ControlMode.MotionMagic, liftLowScoreHatch);
                    fourBarLift.SetPositionDegrees(fourBarLowHatchDegrees);
                }

                //If the '5' button is pressed, then the lift sets to top ball position.
                if (operatorJoystick.getRawButtonPressed(5)) {
                    liftMaster.set(ControlMode.MotionMagic, liftHighScoreBall);
                    fourBarLift.SetPositionDegrees(fourBarHighBallDegrees);
                }

                //If the '10' button is pressed, then the lift sets to the middle ball position.
                if (operatorJoystick.getRawButtonPressed(10)) {
                    liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreBall);
                    fourBarLift.SetPositionDegrees(fourBarMiddleBallDegrees);
                }

                //If the '15' button is pressed, then the lift sets to the low ball position.
                if (operatorJoystick.getRawButtonPressed(15)) {
                    liftMaster.set(ControlMode.MotionMagic, liftLowScoreBall);
                    fourBarLift.SetPositionDegrees(fourBarLowBallDegrees);
                }

                //When the '13' button is pressed, the lift sets to gathering position.
                if (operatorJoystick.getRawButtonPressed(13)) {
                    liftMaster.set(ControlMode.MotionMagic, liftGatheringPosition);
                    fourBarLift.SetPositionDegrees(fourBarGatheringPositionDegrees);
                }

                // When buttons are pressed, the gatherer will gather in '1' or launch out '2'.
                if (operatorJoystick.getRawButton(1)) {
                    gathererMotor.set(ControlMode.PercentOutput, 0.75);
                } else if (operatorJoystick.getRawButton(2)) {
                    gathererMotor.set(ControlMode.PercentOutput, -1.0);
                } else {
                    gathererMotor.set(ControlMode.PercentOutput, .1);
                }

                // Detect the current state of the magnetic limit switch.
                boolean currentSwitchState = gathererMotor.getSensorCollection().isFwdLimitSwitchClosed();

                // Actuate the solenoid depending on the user button press and the magnetic switch.
                if (operatorJoystick.getRawButtonPressed(6)) {
                    hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
                    hatchSwitchDebounceCounter = 40;

                } else if (operatorJoystick.getRawButtonPressed(7)) {
                    hatchSolenoid.set(DoubleSolenoid.Value.kForward);
                    hatchSwitchDebounceCounter = 40;

                } else if (hatchSwitchDebounceCounter-- <= 0 && currentSwitchState && !previousHatchSwitchValue && hatchSolenoid.get() == DoubleSolenoid.Value.kReverse) {
                    hatchSolenoid.set(DoubleSolenoid.Value.kForward);
                    liftMaster.set(ControlMode.MotionMagic, liftInstantGatherHatch);

                }

                // Store the last magnetic switch value.
                previousHatchSwitchValue = currentSwitchState;

            } else {

                //Disable when tuning PIDs
                //when the right bumper is pressed, set the lift to ball positions.
                if (operatorJoystick.getRawButton(6)) {

                    // If the 'A' button is pressed, then set the ball gathering height.
                    if (operatorJoystick.getRawButtonPressed(1)) {
                        liftMaster.set(ControlMode.MotionMagic, liftGatheringPositionBall);

                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarGatheringPositionBall, 0, 0);
                    }

                    // If the 'B' button is pressed, then set the low ball position.
                    if (operatorJoystick.getRawButtonPressed(2)) {
                        liftMaster.set(ControlMode.MotionMagic, liftLowScoreBall);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarLowScoreBall, 0, 0);
                    }

                    // If the 'X' button is pressed, then set the middle ball position.
                    if (operatorJoystick.getRawButtonPressed(3)) {
                        liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreBall);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarMiddleScoreBall, 0, 0);
                    }

                    // If the 'Y' button is pressed, then set the high ball position.
                    if (operatorJoystick.getRawButtonPressed(4)) {
                        liftMaster.set(ControlMode.MotionMagic, liftHighScoreBall);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarHighScoreBall, 0, 0);
                    }

                }

                // When the right bumper isn't pressed, set the lift to hatch positions.
                else {

                    // If the 'A' button is pressed, then set the hatch gathering position.
                    if (operatorJoystick.getRawButtonPressed(1)) {
                        liftMaster.set(ControlMode.MotionMagic, liftGatheringPositionHatch);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarGatheringPositionHatch, 0, 0);
                    }

                    // If the 'B' button is pressed, set the low hatch position.
                    if (operatorJoystick.getRawButtonPressed(2)) {
                        liftMaster.set(ControlMode.MotionMagic, liftLowScoreHatch);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarLowScoreHatch, 0, 0);
                    }

                    // If the 'X' button is pressed, set the middle hatch position.
                    if (operatorJoystick.getRawButtonPressed(3)) {
                        liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreHatch);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarMiddleScoreHatch, 0, 0);
                    }

                    // If the 'Y' button is pressed, set the high hatch position.
                    if (operatorJoystick.getRawButtonPressed(4)) {
                        liftMaster.set(ControlMode.MotionMagic, liftHighScoreHatch);
                        // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarHighScoreHatch, 0, 0);
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
                    liftMaster.set(ControlMode.MotionMagic, liftInstantGatherHatch);
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

            fourBarLift.RunFourBar();

        } catch (
                Exception Ex) {
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