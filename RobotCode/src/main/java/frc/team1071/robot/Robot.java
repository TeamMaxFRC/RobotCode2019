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
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.InetAddress;
import java.util.BitSet;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private CurvatureDrive DriveTrain;
    private FourBar FourBarLift;

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // Is the stream deck in use?
    private static final boolean isStreamDeck = true;
    private static final boolean isPracticeRobot = true;

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

    // Encoder constants for the four bar, based off the practice robot specifics.
    static double fourBarEncoderOffset = isPracticeRobot ? -0.30078125 : 0.06298828125;
    static double fourBarGatheringPositionBall = isPracticeRobot ? 0.363525390625 : 0.121826171875;

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

            FourBarLift = new FourBar(fourBarMotor, 1272);

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
            if (!isPracticeRobot)
            {
                gathererMotor.setInverted(true);
            }

            gathererMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

            // Initialize the NavX.
            navX = new AHRS(SPI.Port.kMXP);

            DriveTrain = new CurvatureDrive(leftMaster, leftSlavePrimary, leftSlaveSecondary, rightMaster, rightSlavePrimary, rightSlaveSecondary, navX);

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
        limelightTarget = tv.getDouble(0.0) >= 1.0;
        // System.out.println(fourBarMotor.getPosition());
        // System.out.println(fourBarMotor.getSelectedSensorPosition());
        //System.out.println(liftMaster.getSelectedSensorPosition());

        FourBarLift.RobotPeriodic();

        // Always send out error data.
        SendOscErrorData();

        // Always send out sensor data.
        SendOscSensorData();
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
     * This function writes the provided line to the dashboard console.
     *
     * @param Line The string being written to the dashboard console.
     */
    private void writeConsole(String Line) {

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
        liftMaster.setSelectedSensorPosition(0,0,10);
        liftMaster.set(ControlMode.MotionMagic, 0);

        // Reset the solenoid position.
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);

        FourBarLift.TeleopInit();

        // Report that the console is functional.
        writeConsole("Robot has enabled!");
    }

    /**
     * Calls the necessary helpers to send all the relevant OSC data.
     */
    private void SendOscData() {

    }

    /**
     * Pools together the current data for each subsystem and sends it to the dashboard.
     */
    private void SendOscCurrentData() {

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("CurrentBundle");

        // Append the current total for the drive motors.
        OSCMessage driveCurrent = new OSCMessage();
        driveCurrent.setAddress("/DriveCurrent");

        double totalDriveCurrent = 0;
        totalDriveCurrent += leftMaster.getOutputCurrent() + leftSlavePrimary.getOutputCurrent() + leftSlaveSecondary.getOutputCurrent();
        totalDriveCurrent += rightMaster.getOutputCurrent() + rightSlavePrimary.getOutputCurrent() + rightSlaveSecondary.getOutputCurrent();

        driveCurrent.addArgument(totalDriveCurrent);

        // Append the current total for the lift motors.
        OSCMessage liftCurrent = new OSCMessage();
        liftCurrent.setAddress("/LiftCurrent");

        double totalLiftCurrent = 0;
        totalLiftCurrent += liftMaster.getOutputCurrent() + liftSlavePrimary.getOutputCurrent();
        totalLiftCurrent += liftSlaveSecondary.getOutputCurrent() + liftSlaveTertiary.getOutputCurrent();

        liftCurrent.addArgument(totalLiftCurrent);

        // Append the current for the four bar motor.
        OSCMessage fourBarCurrent = new OSCMessage();
        fourBarCurrent.setAddress("/FourBarCurrent");
        fourBarCurrent.addArgument(fourBarMotor.getOutputCurrent());

        // Append the current for the gatherer motor.
        OSCMessage gathererCurrent = new OSCMessage();
        gathererCurrent.setAddress("/GathererCurrent");
        gathererCurrent.addArgument(gathererMotor.getOutputCurrent());

        // Append the current for the compressor.
        OSCMessage compressorCurrent = new OSCMessage();
        compressorCurrent.setAddress("/CompressorCurrent");
        compressorCurrent.addArgument(compressor.getCompressorCurrent());

        // Add these packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(driveCurrent);
        bundle.addPacket(liftCurrent);
        bundle.addPacket(fourBarCurrent);
        bundle.addPacket(gathererCurrent);
        bundle.addPacket(compressorCurrent);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the current data! " + ex.getMessage());
        }

    }

    // Send controller data
    private void SendOscControllerData() {

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier  for the bundle
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("ControllerDataBundle");

        int convertedBoolean;

// Send driver controller data.
        OSCMessage aButton = new OSCMessage();
        aButton.setAddress("/Controller/AButton");
        convertedBoolean = driverJoystick.getRawButton(1) ? 1 : 0;
        aButton.addArgument(convertedBoolean);

        OSCMessage bButton = new OSCMessage();
        bButton.setAddress("/Controller/BButton");
        convertedBoolean = driverJoystick.getRawButton(2) ? 1 : 0;
        bButton.addArgument(convertedBoolean);

        OSCMessage xButton = new OSCMessage();
        xButton.setAddress("/Controller/XButton");
        convertedBoolean = driverJoystick.getRawButton(3) ? 1 : 0;
        xButton.addArgument(convertedBoolean);

        OSCMessage yButton = new OSCMessage();
        yButton.setAddress("/Controller/YButton");
        convertedBoolean = driverJoystick.getRawButton(4) ? 1 : 0;
        yButton.addArgument(convertedBoolean);

        OSCMessage leftBumper = new OSCMessage();
        leftBumper.setAddress("/Controller/LeftBumper");
        convertedBoolean = driverJoystick.getRawButton(5) ? 1 : 0;
        leftBumper.addArgument(convertedBoolean);

        OSCMessage rightBumper = new OSCMessage();
        rightBumper.setAddress("/Controller/RightBumper");
        convertedBoolean = driverJoystick.getRawButton(6) ? 1 : 0;
        rightBumper.addArgument(convertedBoolean);

        OSCMessage viewButton = new OSCMessage();
        viewButton.setAddress("/Controller/viewButton");
        convertedBoolean = driverJoystick.getRawButton(7) ? 1 : 0;
        viewButton.addArgument(convertedBoolean);

        OSCMessage menuButton = new OSCMessage();
        menuButton.setAddress("/Controller/menuButton");
        convertedBoolean = driverJoystick.getRawButton(8) ? 1 : 0;
        menuButton.addArgument(convertedBoolean);

        OSCMessage leftStickButton = new OSCMessage();
        leftStickButton.setAddress("/Controller/leftStickButton");
        convertedBoolean = driverJoystick.getRawButton(9) ? 1 : 0;
        leftStickButton.addArgument(convertedBoolean);

        OSCMessage rightStickButton = new OSCMessage();
        rightStickButton.setAddress("/Controller/rightStickButton");
        convertedBoolean = driverJoystick.getRawButton(10) ? 1 : 0;
        rightStickButton.addArgument(convertedBoolean);

        OSCMessage leftStickAxisX = new OSCMessage();
        leftStickAxisX.setAddress("/Controller/LeftStickAxisX");
        leftStickAxisX.addArgument(driverJoystick.getRawAxis(0));

        OSCMessage leftStickAxisY = new OSCMessage();
        leftStickAxisY.setAddress("/Controller/LeftStickAxisY");
        leftStickAxisY.addArgument(driverJoystick.getRawAxis(1));

        OSCMessage rightStickAxisX = new OSCMessage();
        rightStickAxisX.setAddress("/Controller/RightStickAxisX");
        rightStickAxisX.addArgument(driverJoystick.getRawAxis(4));

        OSCMessage rightStickAxisY = new OSCMessage();
        rightStickAxisY.setAddress("/Controller/RightStickAxisY");
        rightStickAxisY.addArgument(driverJoystick.getRawAxis(5));

        OSCMessage leftTrigger = new OSCMessage();
        leftTrigger.setAddress("/Controller/LeftTrigger");
        leftTrigger.addArgument(driverJoystick.getRawAxis(2));

        OSCMessage rightTrigger = new OSCMessage();
        rightTrigger.setAddress("/Controller/RightTrigger");
        rightTrigger.addArgument(driverJoystick.getRawAxis(3));

        // Send operator controller data
        OSCMessage aButton2 = new OSCMessage();
        aButton2.setAddress("/Controller/AButton2");
        convertedBoolean = operatorJoystick.getRawButton(1) ? 1 : 0;
        aButton2.addArgument(convertedBoolean);

        OSCMessage bButton2 = new OSCMessage();
        bButton2.setAddress("/Controller/BButton2");
        convertedBoolean = operatorJoystick.getRawButton(2) ? 1 : 0;
        bButton2.addArgument(convertedBoolean);

        OSCMessage xButton2 = new OSCMessage();
        xButton2.setAddress("/Controller/XButton2");
        convertedBoolean = operatorJoystick.getRawButton(3) ? 1 : 0;
        xButton2.addArgument(convertedBoolean);

        OSCMessage yButton2 = new OSCMessage();
        yButton2.setAddress("/Controller/YButton2");
        convertedBoolean = operatorJoystick.getRawButton(4) ? 1 : 0;
        yButton2.addArgument(convertedBoolean);

        OSCMessage leftBumper2 = new OSCMessage();
        leftBumper2.setAddress("/Controller/LeftBumper2");
        convertedBoolean = operatorJoystick.getRawButton(5) ? 1 : 0;
        leftBumper2.addArgument(convertedBoolean);

        OSCMessage rightBumper2 = new OSCMessage();
        rightBumper2.setAddress("/Controller/RightBumper2");
        convertedBoolean = operatorJoystick.getRawButton(6) ? 1 : 0;
        rightBumper2.addArgument(convertedBoolean);

        OSCMessage viewButton2 = new OSCMessage();
        viewButton2.setAddress("/Controller/viewButton2");
        convertedBoolean = operatorJoystick.getRawButton(7) ? 1 : 0;
        viewButton2.addArgument(convertedBoolean);

        OSCMessage menuButton2 = new OSCMessage();
        menuButton2.setAddress("/Controller/menuButton2");
        convertedBoolean = operatorJoystick.getRawButton(8) ? 1 : 0;
        menuButton2.addArgument(convertedBoolean);

        OSCMessage leftStickButton2 = new OSCMessage();
        leftStickButton2.setAddress("/Controller/leftStickButton2");
        convertedBoolean = operatorJoystick.getRawButton(9) ? 1 : 0;
        leftStickButton2.addArgument(convertedBoolean);

        OSCMessage rightStickButton2 = new OSCMessage();
        rightStickButton2.setAddress("/Controller/rightStickButton2");
        convertedBoolean = operatorJoystick.getRawButton(10) ? 1 : 0;
        rightStickButton2.addArgument(convertedBoolean);

        OSCMessage leftStickAxisX2 = new OSCMessage();
        leftStickAxisX2.setAddress("/Controller/LeftStickAxisX2");
        leftStickAxisX2.addArgument(operatorJoystick.getRawAxis(0));

        OSCMessage leftStickAxisY2 = new OSCMessage();
        leftStickAxisY2.setAddress("/Controller/LeftStickAxisY2");
        leftStickAxisY2.addArgument(operatorJoystick.getRawAxis(1));

        OSCMessage rightStickAxisX2 = new OSCMessage();
        rightStickAxisX2.setAddress("/Controller/RightStickAxisX2");
        rightStickAxisX2.addArgument(operatorJoystick.getRawAxis(4));

        OSCMessage rightStickAxisY2 = new OSCMessage();
        rightStickAxisY2.setAddress("/Controller/RightStickAxisY2");
        rightStickAxisY2.addArgument(driverJoystick.getRawAxis(5));

        OSCMessage leftTrigger2 = new OSCMessage();
        leftTrigger2.setAddress("/Controller/LeftTrigger2");
        leftTrigger2.addArgument(operatorJoystick.getRawAxis(2));

        OSCMessage rightTrigger2 = new OSCMessage();
        rightTrigger2.setAddress("/Controller/RightTrigger2");
        rightTrigger2.addArgument(operatorJoystick.getRawAxis(3));

        // Add these packets to the bundle
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(aButton);
        bundle.addPacket(bButton);
        bundle.addPacket(xButton);
        bundle.addPacket(yButton);
        bundle.addPacket(viewButton);
        bundle.addPacket(menuButton);
        bundle.addPacket(leftBumper);
        bundle.addPacket(rightBumper);
        bundle.addPacket(leftStickButton);
        bundle.addPacket(rightStickButton);
        bundle.addPacket(leftStickAxisX);
        bundle.addPacket(leftStickAxisY);
        bundle.addPacket(rightStickAxisX);
        bundle.addPacket(rightStickAxisY);
        bundle.addPacket(rightTrigger);
        bundle.addPacket(leftTrigger);
        bundle.addPacket(aButton2);
        bundle.addPacket(bButton2);
        bundle.addPacket(xButton2);
        bundle.addPacket(yButton2);
        bundle.addPacket(viewButton2);
        bundle.addPacket(menuButton2);
        bundle.addPacket(leftBumper2);
        bundle.addPacket(rightBumper2);
        bundle.addPacket(leftStickButton2);
        bundle.addPacket(rightStickButton2);
        bundle.addPacket(leftStickAxisX2);
        bundle.addPacket(leftStickAxisY2);
        bundle.addPacket(rightStickAxisX2);
        bundle.addPacket(rightStickAxisY2);
        bundle.addPacket(rightTrigger2);
        bundle.addPacket(leftTrigger2);

        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the controller data!" + ex.getMessage());
        }
    }

    /**
     * TODO: Comment.
     */
    private void SendOscSensorData(){

        // Create an OSC bundle for encoder velocities.
        OSCBundle bundle = new OSCBundle();

        // Create integer to convert hatchGrabber bool to an int
        double convertedBoolean;

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("SensorInputBundle");

        // Send the lift encoder velocity and position.
        OSCMessage liftEncoderVelocity = new OSCMessage();
        liftEncoderVelocity.setAddress("/LiftEncoderVelocity");
        liftEncoderVelocity.addArgument((double) liftMaster.getSelectedSensorVelocity());;

        OSCMessage liftEncoderPosition = new OSCMessage();
        liftEncoderPosition.setAddress("/LiftEncoderPosition");
        liftEncoderPosition.addArgument((double) liftMaster.getSelectedSensorPosition());

        // Send the drive encoder velocity.
        OSCMessage leftEncoderVelocity = new OSCMessage();
        leftEncoderVelocity.setAddress("/LeftEncoderVelocity");
        leftEncoderVelocity.addArgument(DriveTrain.GetLeftEncoderVelocity());

        OSCMessage rightEncoderVelocity = new OSCMessage();
        rightEncoderVelocity.setAddress("/RightEncoderVelocity");
        rightEncoderVelocity.addArgument(DriveTrain.GetRightEncoderVelocity());

        OSCMessage magneticGatherPosition = new OSCMessage();
        magneticGatherPosition.setAddress("/MagneticGatherEncoder");
        convertedBoolean = gathererMotor.getSensorCollection().isFwdLimitSwitchClosed() ? 1 : 0;
        magneticGatherPosition.addArgument(convertedBoolean);

        // Four bar encoder position.
        OSCMessage fourBarEncoderPosition = new OSCMessage();
        fourBarEncoderPosition.setAddress("/FourBarRelativeEncoderPosition");
        fourBarEncoderPosition.addArgument((double) fourBarMotor.getSensorCollection().getQuadraturePosition());

        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(liftEncoderPosition);
        bundle.addPacket(rightEncoderVelocity);
        bundle.addPacket(leftEncoderVelocity);
        bundle.addPacket(liftEncoderVelocity);
        bundle.addPacket(magneticGatherPosition);
        bundle.addPacket(fourBarEncoderPosition);

        // Send the sensor data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the error data! " + ex.getMessage());
        }

    }

    /**
     * Pools together the error data and sends it to the dashboard.
     */
    private void SendOscErrorData() {

        // Variable for converting Spark MAX faults.
        short faultShort;
        byte[] faultBytes = new byte[2];
        BitSet faultBits = new BitSet();

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("ErrorBundle");

        // See which errors there are and send to error widget.
        OSCMessage leftMasterFaults = new OSCMessage();
        leftMasterFaults.setAddress("/LeftMasterFaults");

        faultShort = leftMaster.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            leftMasterFaults.addArgument(i);
        }

        if (leftMasterFaults.getArguments().isEmpty()) {
            leftMasterFaults.addArgument(-1);
        }

        OSCMessage rightMasterFaults = new OSCMessage();
        rightMasterFaults.setAddress("/RightMasterFaults");

        faultShort = rightMaster.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            rightMasterFaults.addArgument(i);
        }

        if (rightMasterFaults.getArguments().isEmpty()) {
            rightMasterFaults.addArgument(-1);
        }

        OSCMessage leftSlavePrimaryFaults = new OSCMessage();
        leftSlavePrimaryFaults.setAddress("/LeftSlavePrimaryFaults");

        faultShort = leftSlavePrimary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            leftSlavePrimaryFaults.addArgument(i);
        }

        if (leftSlavePrimaryFaults.getArguments().isEmpty()) {
            leftSlavePrimaryFaults.addArgument(-1);
        }

        OSCMessage rightSlavePrimaryFaults = new OSCMessage();
        rightSlavePrimaryFaults.setAddress("/RightSlavePrimaryFaults");

        faultShort = rightSlavePrimary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            rightSlavePrimaryFaults.addArgument(i);
        }

        if (rightSlavePrimaryFaults.getArguments().isEmpty()) {
            rightSlavePrimaryFaults.addArgument(-1);
        }

        OSCMessage leftSlaveSecondaryFaults = new OSCMessage();
        leftSlaveSecondaryFaults.setAddress("/LeftSlaveSecondaryFaults");

        faultShort = leftSlaveSecondary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            leftSlaveSecondaryFaults.addArgument(i);
        }

        if (leftSlaveSecondaryFaults.getArguments().isEmpty()) {
            leftSlaveSecondaryFaults.addArgument(-1);
        }

        OSCMessage rightSlaveSecondaryFaults = new OSCMessage();
        rightSlaveSecondaryFaults.setAddress("/RightSlaveSecondaryFaults");

        faultShort = rightSlaveSecondary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            rightSlaveSecondaryFaults.addArgument(i);
        }

        if (rightSlaveSecondaryFaults.getArguments().isEmpty()) {
            rightSlaveSecondaryFaults.addArgument(-1);
        }

        // Add packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(leftMasterFaults);
        bundle.addPacket(rightMasterFaults);
        bundle.addPacket(leftSlavePrimaryFaults);
        bundle.addPacket(rightSlavePrimaryFaults);
        bundle.addPacket(leftSlaveSecondaryFaults);
        bundle.addPacket(rightSlaveSecondaryFaults);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the error data! " + ex.getMessage());
        }

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        //--------------------------------------------------------------------------------------------------------------
        // Drive Controls
        //--------------------------------------------------------------------------------------------------------------

        FourBarLift.RunFourBar();

        // Determine the proper motor values based on the joystick data.
        double driverVertical = 0;
        double driverTwist = 0;
        boolean driverQuickTurn = false;
        Update_Limelight_Tracking();
        if (driverJoystick.getRawButton(5)) {
            if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipeline").getDouble(0) != 1) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipline").setNumber(1);
            } else {
                if (limelightTarget) {
                    driverVertical = m_LimelightDriveCommand;
                    driverTwist = m_LimelightSteerCommand;
                    driverQuickTurn = true;
                }
            }
        } else {
            if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0) != 0) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipline").setNumber(0);
            }
            driverVertical = QuickMaths.normalizeJoystickWithDeadband(-driverJoystick.getRawAxis(1), 0.05);
            driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getRawAxis(4), 0.05);
            //System.out.println(driverTwist);
            driverQuickTurn = driverJoystick.getRawButton(6);
        }
        //System.out.println("test:" + driverTwist);
        DriveTrain.Run(driverVertical, driverTwist, false, false, driverJoystick.getRawAxis(3));
        //--------------------------------------------------------------------------------------------------------------
        // Operator Controls
        //--------------------------------------------------------------------------------------------------------------
        try {

            // TODO: Properly comment, or clean up this code.
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(operatorJoystick.getRawButton(8) ? 1 : 0);

            //All set points must be in rotations. Measure using getPosition().
            // Four bar ball set positions.
            double fourBarLowScoreBall = fourBarGatheringPositionBall + 0.34;
            double fourBarMiddleScoreBall = fourBarGatheringPositionBall + 0.36;
            double fourBarHighScoreBall = fourBarGatheringPositionBall + 0.32;

            // Four bar hatch set positions.
            double fourBarGatheringPositionHatch = fourBarGatheringPositionBall + 0.02;
            double fourBarLowScoreHatch = fourBarGatheringPositionHatch + 0.02;
            double fourBarMiddleScoreHatch = fourBarGatheringPositionHatch + 0.02;
            double fourBarHighScoreHatch = fourBarGatheringPositionBall + 0.07;

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
            if (isStreamDeck) {

                // If the '4' button is pressed, then lift sets to top hatch position.
                if (operatorJoystick.getRawButtonPressed(4)) {
                    liftMaster.set(ControlMode.MotionMagic, liftHighScoreHatch);
                    // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarHighScoreHatch, 0, 0);
                }

                // If the '9' button is pressed, then lift sets to middle hatch position.
                if (operatorJoystick.getRawButtonPressed(9)) {
                    liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreHatch);
                    // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarMiddleScoreHatch, 0, 0);
                }

                // If the '14' button is pressed, then lift sets to low hatch position.
                if (operatorJoystick.getRawButtonPressed(14)) {
                    liftMaster.set(ControlMode.MotionMagic, liftLowScoreHatch);
                    // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarLowScoreHatch, 0, 0);
                }
                //When buttons are pressed on the stream deck, set lift to ball positions.

                //If the '5' button is pressed, then the lift sets to top ball position.
                if (operatorJoystick.getRawButtonPressed(5)) {
                    liftMaster.set(ControlMode.MotionMagic, liftHighScoreBall);
                    // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarHighScoreBall, 0, 0);
                }

                //If the '10' button is pressed, then the lift sets to the middle ball position.
                if (operatorJoystick.getRawButtonPressed(10)) {
                    liftMaster.set(ControlMode.MotionMagic, liftMiddleScoreBall);
                    // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarMiddleScoreBall, 0, 0);
                }

                //If the '15' button is pressed, then the lift sets to the low ball position.
                if (operatorJoystick.getRawButtonPressed(15)) {
                    liftMaster.set(ControlMode.MotionMagic, liftLowScoreBall);
                    // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarLowScoreBall, 0, 0);
                }

                //When the '13' button is pressed, the lift sets to gathering position.
                if (operatorJoystick.getRawButtonPressed(13)) {
                    liftMaster.set(ControlMode.MotionMagic, liftGatheringPosition);
                    // fourBarMotor.set(MCControlMode.MotionVoodooArbFF, fourBarGatheringPositionBall, 0, 0);
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

        } catch (
                Exception Ex) {
            System.out.println("Exception in operator controls! " + Ex.getMessage());
        }

        //----------------------------------------------------------------------------------------------------------
        // Other Periodic Operations
        //----------------------------------------------------------------------------------------------------------

        // Send OSC data to the dashboard.
        SendOscData();

        try {

            // Create messages for the current motor values.
            OSCMessage leftMotorValueMessage = new OSCMessage();
            OSCMessage rightMotorValueMessage = new OSCMessage();

            // Set the current motor values in the OSC messages.
            OSCMessage ControllerButtonsMessage = new OSCMessage();

            // Create message for navX gyro values
            OSCMessage navXGyroMessage = new OSCMessage();

            // Send navX Gyro values
            // navXGyroMessage.setAddress("/Robot/NavX/Gyro");
            // navXGyroMessage.addArgument(navX.getFusedHeading());

//            // Send the current motor values
//            leftMotorValueMessage.setAddress("/Robot/Motors/Left/Value");
//            leftMotorValueMessage.addArgument(motorValues.leftDrive);
//
//            rightMotorValueMessage.setAddress("/Robot/Motors/Right/Value");
//            rightMotorValueMessage.addArgument(motorValues.rightDrive);

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

        } catch (
                Exception Ex) {
            System.out.println("Exception in OSC sending! " + Ex.getMessage());
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
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        if (!limelightTarget) {
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        // Start with proportional steering
        double steer_cmd = limelightX * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - limelightArea) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }
}
