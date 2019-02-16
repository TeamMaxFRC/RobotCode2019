package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team195.motorcontrol.CKTalonSRX;
import frc.team195.motorcontrol.MCControlMode;
import frc.team195.motorcontrol.PDPBreaker;
import frc.team254.InterpolatingDouble;

import java.net.InetAddress;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    // Create the NavX.
    private AHRS navX;
    // Create the OSC sender on the robot.
    private OSCPortOut oscWirelessSender;
    private OSCPortOut oscWiredSender;
    // Create the Limelight.
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry tv = table.getEntry("tv");
    private double limelightX, limelightY, limelightArea;
    private boolean limelightTarget;
    // Create the auxiliary motors.
    private TalonSRX liftMaster;
    private TalonSRX liftSlavePrimary;
    private TalonSRX liftSlaveSecondary;
    private TalonSRX liftSlaveTertiary;
    private TalonSRX gathererMotor;
    private CKTalonSRX fourBarMotor;
    // Helper class for calculating drive motor speeds.
    private DriveHelper driveHelper = new DriveHelper();
    // Create the joysticks for the driver and the operator.
    private Joystick driverJoystick = new Joystick(0);
    private Joystick operatorJoystick = new Joystick(1);
    // Create the drive motors.
    private Compressor compressor = new Compressor(0);
    private CANSparkMax leftMaster;
    private CANSparkMax rightMaster;
    //Create and initialize the compressor.

    //Create the Solenoid and Solenoid state for hatch gathering.
    private DoubleSolenoid hatchSolenoid;
    private boolean previousHatchSwitchValue = false;
    private int hatchSwitchDebounceCounter = 0;

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
            leftMaster = new CANSparkMax(0, kBrushless);

            CANSparkMax leftSlavePrimary = new CANSparkMax(1, kBrushless);
            CANSparkMax leftSlaveSecondary = new CANSparkMax(2, kBrushless);

            leftMaster.setSmartCurrentLimit(55);
            leftSlavePrimary.setSmartCurrentLimit(55);
            leftSlaveSecondary.setSmartCurrentLimit(55);

            leftMaster.setOpenLoopRampRate(1);
            leftSlavePrimary.setOpenLoopRampRate(1);
            leftSlaveSecondary.setOpenLoopRampRate(1);

            leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            leftSlavePrimary.setIdleMode(CANSparkMax.IdleMode.kCoast);
            leftSlaveSecondary.setIdleMode(CANSparkMax.IdleMode.kCoast);

            rightMaster = new CANSparkMax(13, kBrushless);
            CANSparkMax rightSlavePrimary = new CANSparkMax(14, kBrushless);
            CANSparkMax rightSlaveSecondary = new CANSparkMax(15, kBrushless);

            rightMaster.setSmartCurrentLimit(55);
            rightSlavePrimary.setSmartCurrentLimit(55);
            rightSlaveSecondary.setSmartCurrentLimit(55);

            rightMaster.setOpenLoopRampRate(1);
            rightSlavePrimary.setOpenLoopRampRate(1);
            rightSlaveSecondary.setOpenLoopRampRate(1);

            rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            rightSlavePrimary.setIdleMode(CANSparkMax.IdleMode.kCoast);
            rightSlaveSecondary.setIdleMode(CANSparkMax.IdleMode.kCoast);

            // Have the drive slaves follow their respective masters.
            leftSlavePrimary.follow(leftMaster);
            leftSlaveSecondary.follow(leftMaster);

            rightSlavePrimary.follow(rightMaster);
            rightSlaveSecondary.follow(rightMaster);

            // Invert the right side of the drive train.
            rightMaster.setInverted(true);
            rightSlavePrimary.setInverted(true);
            rightSlaveSecondary.setInverted(true);

            //----------------------------------------------------------------------------------------------------------
            // Lift Motors
            //----------------------------------------------------------------------------------------------------------

            // Initialize the lift motors.
            liftMaster = new TalonSRX(4);
            liftSlavePrimary = new TalonSRX(7);
            liftSlaveSecondary = new TalonSRX(9);
            liftSlaveTertiary = new TalonSRX(8);

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

            // Set the PID values for the lift.
            liftMaster.config_kF(0, 0.32058916);
            liftMaster.config_kP(0, 1.4);
            liftMaster.config_kD(0, 2.8);

            // Establish the cruise velocity and max acceleration for motion magic.
            liftMaster.configMotionCruiseVelocity(2900);
            liftMaster.configMotionAcceleration(5200);

            //----------------------------------------------------------------------------------------------------------
            // Four Bar Motor
            //----------------------------------------------------------------------------------------------------------

            // Initialize the four bar motor.
            fourBarMotor = new CKTalonSRX(5, false, PDPBreaker.B30A);

            // Set the encoder mode to absolute position.
            fourBarMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            fourBarMotor.setInverted(true);
            fourBarMotor.setSensorPhase(true);

            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.00), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.01), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.02), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.03), new InterpolatingDouble(0.15));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.04), new InterpolatingDouble(0.17));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.05), new InterpolatingDouble(0.17));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.06), new InterpolatingDouble(0.18));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.07), new InterpolatingDouble(0.19));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.08), new InterpolatingDouble(0.11));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.09), new InterpolatingDouble(0.16));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.1), new InterpolatingDouble(0.17));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.11), new InterpolatingDouble(0.17));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.12), new InterpolatingDouble(0.16));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.13), new InterpolatingDouble(0.14));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.14), new InterpolatingDouble(0.14));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.15), new InterpolatingDouble(0.14));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.16), new InterpolatingDouble(0.21));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.17), new InterpolatingDouble(0.14));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.18), new InterpolatingDouble(0.14));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.19), new InterpolatingDouble(0.14));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.2), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.21), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.22), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.23), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.24), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.25), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.26), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.27), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.28), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.29), new InterpolatingDouble(0.12));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.3), new InterpolatingDouble(0.1));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.31), new InterpolatingDouble(0.05));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.32), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.33), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.34), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.35), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.36), new InterpolatingDouble(0.00));
            fourBarMotor.motionVoodooArbFFLookup.put(new InterpolatingDouble(0.37), new InterpolatingDouble(0.00));

            fourBarMotor.setPIDF(2.4, 0, 16, 1);
            fourBarMotor.setMotionParameters(50, 100);

            fourBarMotor.getSensorCollection().syncQuadratureWithPulseWidth(3386, 4830, true);

            //Absolute val of Min encoder position
            fourBarMotor.setAbsoluteEncoderOffset(0.1743164063);

            fourBarMotor.configForwardSoftLimitThreshold(fourBarMotor.convertRotationsToNativeUnits(0.1767578125));
            fourBarMotor.configForwardSoftLimitEnable(true);
            fourBarMotor.configReverseSoftLimitThreshold(fourBarMotor.convertRotationsToNativeUnits(-0.1743164063));
            fourBarMotor.configReverseSoftLimitEnable(true);

            fourBarMotor.setControlMode(MCControlMode.MotionVoodooArbFF);
            //TuneablePIDOSC t = new TuneablePIDOSC("FourBar", 5805, true, fourBarMotor);

            //----------------------------------------------------------------------------------------------------------
            // Other Initialization
            //----------------------------------------------------------------------------------------------------------

            // Initialize the gatherer.
            gathererMotor = new TalonSRX(6);
            gathererMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

            // Initialize the NavX.
            navX = new AHRS(SPI.Port.kMXP);

            //Initialize the Solenoid.
            hatchSolenoid = new DoubleSolenoid(0, 1);
            hatchSolenoid.set(DoubleSolenoid.Value.kReverse);

        } catch (Exception Ex) {
            System.out.println("General Initialization Exception: " + Ex.getMessage());
        }

        // Try to open the OSC sockets.
        try {
            oscWirelessSender = new OSCPortOut(InetAddress.getByName("10.10.71.9"), 5803);
            oscWiredSender = new OSCPortOut(InetAddress.getByName("10.10.71.5"), 5803);
        } catch (Exception Ex) {
            System.out.println("OSC Initialization Exception: " + Ex.getMessage());
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
        limelightTarget = tv.getDouble(0.0) == 1;
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

    /**
     * This function writes the provided line to the console.
     *
     * @param Line The string being written to the dashboard console.
     */
    public void writeConsole(String Line) {

        try {
            // Create the console message.
            OSCMessage ConsoleMessage = new OSCMessage();

            // Set the address and arguments for the message.
            ConsoleMessage.setAddress("/Robot/Console/Text");
            ConsoleMessage.addArgument(Line);

            // Send the console message.
            oscWirelessSender.send(ConsoleMessage);
            oscWiredSender.send(ConsoleMessage);

        } catch (Exception Ex) {
            System.out.println("Console Sending Exception: " + Ex.getMessage());
        }
    }

    @Override
    public void teleopInit() {

        // Reset the lift's encoder position.
        liftMaster.setSelectedSensorPosition(0);

        // Reset the solenoid position
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sends drive information to be logged by the dashboard.
     */
    public void SendDriveData() {

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("DriveTrain");

        // Append the robot timestamp for the data.
        OSCMessage timestamp = new OSCMessage();
        timestamp.setAddress("/timestamp");
        timestamp.addArgument(Timer.getFPGATimestamp());

        // Append the left encoder data.
        CANEncoder leftEncoder = leftMaster.getEncoder();
        OSCMessage leftVelocity = new OSCMessage();
        leftVelocity.setAddress("/LeftVelocity");
        leftVelocity.addArgument(leftEncoder.getVelocity());

        // Append the right encoder data.
        CANEncoder rightEncoder = rightMaster.getEncoder();
        OSCMessage rightVelocity = new OSCMessage();
        rightVelocity.setAddress("/RightVelocity");
        rightVelocity.addArgument(rightEncoder.getVelocity());

        // Append the bus voltage.
        OSCMessage Voltage = new OSCMessage();
        Voltage.setAddress("/Voltage");
        Voltage.addArgument(leftMaster.getBusVoltage());

        // Append the fused heading from the Nav X.
        OSCMessage fusedHeading = new OSCMessage();
        fusedHeading.setAddress("/FusedHeading");
        fusedHeading.addArgument((double) navX.getFusedHeading());

        // Add these packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(timestamp);
        bundle.addPacket(leftVelocity);
        bundle.addPacket(Voltage);
        bundle.addPacket(rightVelocity);
        bundle.addPacket(fusedHeading);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the drive log data! " + ex.getMessage());
        }
    }

    /**
     * Calls the necessary helpers to send all the relevant OSC data.
     */
    public void SendOSCData() {
        SendDriveData();
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
        double driverVertical = QuickMaths.normalizeJoystickWithDeadband(-driverJoystick.getRawAxis(1), 0.05);
        double driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getRawAxis(4), 0.05);

        if (driverJoystick.getRawButton(6)) {
            driverTwist *= 0.7;
        }

        driverVertical = Math.copySign(driverVertical * driverVertical, driverVertical);
        driverTwist = Math.copySign(driverTwist * driverTwist, driverTwist);

        DriveMotorValues motorValues = driveHelper.calculateOutput(driverVertical, driverTwist, driverJoystick.getRawButton(6), false, 0.75);

        try {
            leftMaster.set(motorValues.leftDrive);
            rightMaster.set(motorValues.rightDrive);
            System.out.println("Left Duty: " + leftMaster.getAppliedOutput() + " Right Duty: " + rightMaster.getAppliedOutput());
        } catch (Exception Ex) {
            System.out.println("Drive Motor Exception: " + Ex.getMessage());
        }

        //--------------------------------------------------------------------------------------------------------------
        // Operator Controls
        //--------------------------------------------------------------------------------------------------------------
        try {

            // TODO: Properly comment, or clean up this code.
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(operatorJoystick.getRawButton(8) ? 1 : 0);

            //All set points must be in rotations. Measure using getPosition().
            // Four bar ball set positions.
            double fourBarGatheringPositionBall = -0.123;
            double fourBarLowScoreBall = 0.12;
            double fourBarMiddleScoreBall = 0.12;
            double fourBarHighScoreBall = 0.12;

            // Four bar hatch set positions.
            double fourBarGatheringPositionHatch = -0.10;
            double fourBarLowScoreHatch = -0.10;
            double fourBarMiddleScoreHatch = -0.10;
            double fourBarHighScoreHatch = -0.05;

            // Lift ball set positions.
            int liftGatheringPositionBall = 0;
            int liftLowScoreBall = 0;
            int liftMiddleScoreBall = 15000;
            int liftHighScoreBall = 25200;

            // Lift hatch set positions.
            int liftGatheringPositionHatch = 0;
            int liftLowScoreHatch = 0;
            int liftMiddleScoreHatch = 15000;
            int liftHighScoreHatch = 25200;
            int liftInstantGatherHatch = 2000;

            //Disable when tuning PIDs
            //when the right bumper is pressed, set the lift to ball positions.
            if (operatorJoystick.getRawButton(6)) {

                // If the 'A' button is pressed, then set the ball gathering height.
                if (operatorJoystick.getRawButtonPressed(1)) {
                    liftMaster.set(ControlMode.MotionMagic, liftGatheringPositionBall);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarGatheringPositionBall, 0, 0);
                }

                // If the 'B' button is pressed, then set the low ball position.
                if (operatorJoystick.getRawButtonPressed(2)) {
                    liftMaster.set(ControlMode.MotionMagic, liftLowScoreBall);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarLowScoreBall, 0, 0);
                }

                // If the 'X' button is pressed, then set the middle ball position.
                if (operatorJoystick.getRawButtonPressed(3)) {
                    liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreBall);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarMiddleScoreBall, 0, 0);
                }

                // If the 'Y' button is pressed, then set the high ball position.
                if (operatorJoystick.getRawButtonPressed(4)) {
                    liftMaster.set(ControlMode.MotionMagic, liftHighScoreBall);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarHighScoreBall, 0, 0);
                }

            }

            // When the right bumper isn't pressed, set the lift to hatch positions.
            else {

                // If the 'A' button is pressed, then set the hatch gathering position.
                if (operatorJoystick.getRawButtonPressed(1)) {
                    liftMaster.set(ControlMode.MotionMagic, liftGatheringPositionHatch);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarGatheringPositionHatch, 0, 0);
                }

                // If the 'B' button is pressed, set the low hatch position.
                if (operatorJoystick.getRawButtonPressed(2)) {
                    liftMaster.set(ControlMode.MotionMagic, liftLowScoreHatch);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarLowScoreHatch, 0, 0);
                }

                // If the 'X' button is pressed, set the middle hatch position.
                if (operatorJoystick.getRawButtonPressed(3)) {
                    liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreHatch);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarMiddleScoreHatch, 0, 0);
                }

                // If the 'Y' button is pressed, set the high hatch position.
                if (operatorJoystick.getRawButtonPressed(4)) {
                    liftMaster.set(ControlMode.MotionMagic, liftHighScoreHatch);
                    fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarHighScoreHatch, 0, 0);
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
                gathererMotor.set(ControlMode.PercentOutput, 0);
            }

        } catch (Exception Ex) {
            System.out.println("Exception in operator controls! " + Ex.getMessage());
        }

        //----------------------------------------------------------------------------------------------------------
        // Other Periodic Operations
        //----------------------------------------------------------------------------------------------------------

        // Send OSC data to the dashboard.
        SendOSCData();

        try {

            // Create messages for the current motor values.
            OSCMessage leftMotorValueMessage = new OSCMessage();
            OSCMessage rightMotorValueMessage = new OSCMessage();

            // Set the current motor values in the OSC messages.
            OSCMessage leftMasterCurrentMessage = new OSCMessage();
            OSCMessage rightMasterCurrentMessage = new OSCMessage();
            OSCMessage ControllerButtonsMessage = new OSCMessage();

            // Create message for navX gyro values
            OSCMessage navXGyroMessage = new OSCMessage();

            // Send navX Gyro values
            // navXGyroMessage.setAddress("/Robot/NavX/Gyro");
            // navXGyroMessage.addArgument(navX.getFusedHeading());

            // Send the current motor values
            leftMotorValueMessage.setAddress("/Robot/Motors/Left/Value");
            leftMotorValueMessage.addArgument(motorValues.leftDrive);

            rightMotorValueMessage.setAddress("/Robot/Motors/Right/Value");
            rightMotorValueMessage.addArgument(motorValues.rightDrive);

            // Send the current values for the lift motors.
            OSCMessage liftMasterCurrent = new OSCMessage();
            OSCMessage liftPrimaryCurrent = new OSCMessage();
            OSCMessage liftSecondaryCurrent = new OSCMessage();
            OSCMessage liftTertiaryCurrent = new OSCMessage();

            liftMasterCurrent.setAddress("/Robot/Motors/liftMaster/Current");
            liftMasterCurrent.addArgument(liftMaster.getOutputCurrent());

            liftPrimaryCurrent.setAddress("/Robot/Motors/liftSlavePrimary/Current");
            liftPrimaryCurrent.addArgument(liftSlavePrimary.getOutputCurrent());

            liftSecondaryCurrent.setAddress("/Robot/Motors/liftSlaveSecondary/Current");
            liftSecondaryCurrent.addArgument(liftSlaveSecondary.getOutputCurrent());

            liftTertiaryCurrent.setAddress("/Robot/Motors/liftSlaveTertiary/Current");
            liftTertiaryCurrent.addArgument(liftSlaveTertiary.getOutputCurrent());
//
//            oscWiredSender.send(liftMasterCurrent);
//            oscWirelessSender.send(liftMasterCurrent);
//            oscWiredSender.send(liftPrimaryCurrent);
//            oscWirelessSender.send(liftPrimaryCurrent);
//            oscWiredSender.send(liftSecondaryCurrent);
//            oscWirelessSender.send(liftSecondaryCurrent);
//            oscWiredSender.send(liftTertiaryCurrent);
//            oscWirelessSender.send(liftTertiaryCurrent);

            // Send the current values for the lift motors.
            OSCMessage liftMasterVoltage = new OSCMessage();
            OSCMessage liftPrimaryVoltage = new OSCMessage();
            OSCMessage liftSecondaryVoltage = new OSCMessage();
            OSCMessage liftTertiaryVoltage = new OSCMessage();

            liftMasterVoltage.setAddress("/Robot/Motors/liftMaster/Voltage");
            liftMasterVoltage.addArgument(liftMaster.getBusVoltage());

            liftPrimaryVoltage.setAddress("/Robot/Motors/liftSlavePrimary/Voltage");
            liftPrimaryVoltage.addArgument(liftSlavePrimary.getBusVoltage());

            liftSecondaryVoltage.setAddress("/Robot/Motors/liftSlaveSecondary/Voltage");
            liftSecondaryVoltage.addArgument(liftSlaveSecondary.getBusVoltage());

            liftTertiaryVoltage.setAddress("/Robot/Motors/liftSlaveTertiary/Voltage");
            liftTertiaryVoltage.addArgument(liftSlaveTertiary.getBusVoltage());
//
//            oscWiredSender.send(liftMasterVoltage);
//            oscWirelessSender.send(liftMasterVoltage);
//            oscWiredSender.send(liftPrimaryVoltage);
//            oscWirelessSender.send(liftPrimaryVoltage);
//            oscWiredSender.send(liftSecondaryVoltage);
//            oscWirelessSender.send(liftSecondaryVoltage);
//            oscWiredSender.send(liftTertiaryVoltage);
//            oscWirelessSender.send(liftTertiaryVoltage);

            // Send current values for the drive motors.
            OSCMessage leftMasterVoltage = new OSCMessage();
            OSCMessage rightMasterVoltage = new OSCMessage();

            leftMasterVoltage.setAddress("/Robot/Motors/leftMaster/Voltage");
            leftMasterVoltage.addArgument(leftMaster.getBusVoltage());

            rightMasterVoltage.setAddress("/Robot/Motors/rightMaster/Voltage");
            rightMasterVoltage.addArgument(rightMaster.getBusVoltage());

            oscWiredSender.send(leftMasterVoltage);
            oscWirelessSender.send(leftMasterVoltage);
            oscWiredSender.send(rightMasterVoltage);
            oscWirelessSender.send(rightMasterVoltage);

            // Send the lift encoder velocity.
            OSCMessage liftEncoderVelocity = new OSCMessage();

            liftEncoderVelocity.setAddress("/Robot/Motors/liftMaster/EncoderVelocity");
            liftEncoderVelocity.addArgument((double) liftMaster.getSelectedSensorVelocity());

//            oscWiredSender.send(liftEncoderVelocity);
//            oscWirelessSender.send(liftEncoderVelocity);

            // Send the drive encoder velocity.
            OSCMessage leftMasterVelocity = new OSCMessage();
            OSCMessage rightMasterVelocity = new OSCMessage();

            leftMasterVelocity.setAddress("/Robot/Motors/leftMaster/EncoderVelocity");
            leftMasterVelocity.addArgument(leftMaster.getEncoder().getVelocity());

            rightMasterVelocity.setAddress("/Robot/Motors/rightMaster/EncoderVelocity");
            rightMasterVelocity.addArgument(rightMaster.getEncoder().getVelocity());

            oscWiredSender.send(leftMasterVelocity);
            oscWirelessSender.send(leftMasterVelocity);
            oscWiredSender.send(rightMasterVelocity);
            oscWirelessSender.send(rightMasterVelocity);

            // Send the gyro data, packed with the master drive motors.
            OSCMessage leftMasterGyro = new OSCMessage();
            OSCMessage rightMasterGyro = new OSCMessage();

            leftMasterGyro.setAddress("/Robot/Motors/leftMaster/GyroZ");
            leftMasterGyro.addArgument((double) navX.getVelocityZ());

            rightMasterGyro.setAddress("/Robot/Motors/rightMaster/GyroZ");
            rightMasterGyro.addArgument((double) navX.getVelocityZ());

            oscWiredSender.send(leftMasterGyro);
            oscWirelessSender.send(leftMasterGyro);
            oscWiredSender.send(rightMasterGyro);
            oscWirelessSender.send(rightMasterGyro);

            // Send the lift encoder position.
            OSCMessage liftEncoderPosition = new OSCMessage();

            liftEncoderPosition.setAddress("/Robot/Motors/liftMaster/EncoderPosition");
            liftEncoderPosition.addArgument((double) liftMaster.getSelectedSensorPosition());
//
//            oscWiredSender.send(liftEncoderPosition);
//            oscWirelessSender.send(liftEncoderPosition);
//
//            // Send the message.
//            // TODO: Bundle these in the future.
//            oscWirelessSender.send(leftMotorValueMessage);
//            oscWiredSender.send(leftMotorValueMessage);
//            oscWirelessSender.send(rightMotorValueMessage);
//            oscWiredSender.send(rightMotorValueMessage);
//            oscWirelessSender.send(leftMasterCurrentMessage);
//            oscWiredSender.send(leftMasterCurrentMessage);
//            oscWirelessSender.send(rightMasterCurrentMessage);
//            oscWiredSender.send(rightMasterCurrentMessage);
//            oscWirelessSender.send(navXGyroMessage);
//            oscWiredSender.send(navXGyroMessage);
//            oscWirelessSender.send(ControllerButtonsMessage);
//            oscWiredSender.send(ControllerButtonsMessage);

            // System.out.println("Currents: " + liftMaster.getOutputCurrent() + " " + liftSlaveSecondary.getOutputCurrent() + " " + liftSlaveTertiary.getOutputCurrent() + " " + liftSlavePrimary.getOutputCurrent());
            // System.out.println("Position: " + liftMaster.getSelectedSensorPosition() + " Velocity: " + liftMaster.getSelectedSensorVelocity());

            // Send the values for the Limelight
            OSCMessage limelightMessageX = new OSCMessage();
            OSCMessage limelightMessageY = new OSCMessage();
            OSCMessage limelightMessageA = new OSCMessage();
            OSCMessage limelightMessageV = new OSCMessage();
            limelightMessageX.setAddress("/Robot/Limelight/X");
            limelightMessageY.setAddress("/Robot/Limelight/Y");
            limelightMessageA.setAddress("/Robot/Limelight/A");
            limelightMessageV.setAddress("/Robot/Limelight/V");
            limelightMessageX.addArgument(limelightX);
            limelightMessageY.addArgument(limelightY);
            limelightMessageA.addArgument(limelightArea);
            limelightMessageV.addArgument(limelightTarget ? 0 : 1);
            oscWirelessSender.send(limelightMessageX);
            oscWirelessSender.send(limelightMessageY);
            oscWirelessSender.send(limelightMessageA);
            oscWirelessSender.send(limelightMessageV);

        } catch (Exception Ex) {
            System.out.println("Exception in OSC sending! " + Ex.getMessage());
        }

    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {

    }
}
