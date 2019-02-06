package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    DriveHelper driveHelper = new DriveHelper();

    // Create the joystick for the driver and the operator.
    Joystick driverJoystick = new Joystick(0);
    Joystick operatorJoystick = new Joystick(1);

    // Create and initialize the power distribution board.
    PowerDistributionPanel PDP = new PowerDistributionPanel();

    // Create the drive motors.
    CANSparkMax leftMaster;
    CANSparkMax leftSlavePrimary;
    CANSparkMax leftSlaveSecondary;
    CANSparkMax rightMaster;
    CANSparkMax rightSlavePrimary;
    CANSparkMax rightSlaveSecondary;

    // Create the lift motors.
    TalonSRX liftMaster;
    TalonSRX liftSlavePrimary;
    TalonSRX liftSlaveSecondary;
    TalonSRX liftSlaveTertiary;

    // Create the NavX.
    AHRS navX;

    // Create the OSC sender on the robot.
    OSCPortOut oscWirelessSender;
    OSCPortOut oscWiredSender;

    // Create the Limelight.
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    double limelightX, limelightY, limelightArea;
    boolean limelightTarget;

    // Set constants for Limelight targeting
    double targetCenter;

    private String m_autoSelected;

    private static void configLiftMotorPower(TalonSRX t) {
        t.configPeakCurrentLimit(60);
        t.configPeakCurrentDuration(1200);
        t.configContinuousCurrentLimit(30);
        t.enableCurrentLimit(true);
        t.configVoltageCompSaturation(12);
        t.enableVoltageCompensation(true);
    }

    // This function is run when the robot is first started up.
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        // Initialize all the motor controllers.
        try {
            // Initialize the drive motor controllers.
            leftMaster = new CANSparkMax(0, kBrushless);
            leftSlavePrimary = new CANSparkMax(1, kBrushless);
            leftSlaveSecondary = new CANSparkMax(2, kBrushless);
            rightMaster = new CANSparkMax(13, kBrushless);
            rightSlavePrimary = new CANSparkMax(14, kBrushless);
            rightSlaveSecondary = new CANSparkMax(15, kBrushless);

            // Define the lift motors as master or slave.
            leftSlavePrimary.follow(leftMaster);
            leftSlaveSecondary.follow(leftMaster);
            rightSlavePrimary.follow(rightMaster);
            rightSlaveSecondary.follow(rightMaster);
            rightMaster.setInverted(true);
            rightSlavePrimary.setInverted(true);
            rightSlaveSecondary.setInverted(true);

            // Initialize the lift motor controllers.
            liftMaster = new TalonSRX(4);
            liftSlavePrimary = new TalonSRX(7);
            liftSlaveSecondary = new TalonSRX(9);
            liftSlaveTertiary = new TalonSRX(8);

            // Invert the motors.
            liftMaster.setInverted(true);
            liftSlavePrimary.setInverted(true);
            liftSlaveSecondary.setInverted(true);
            liftSlaveTertiary.setInverted(true);

            liftMaster.setSensorPhase(true);
            configLiftMotorPower(liftMaster);
            configLiftMotorPower(liftSlavePrimary);
            configLiftMotorPower(liftSlaveSecondary);
            configLiftMotorPower(liftSlaveTertiary);

            liftMaster.config_kF(0, 0.32058916);
            liftMaster.config_kP(0, 1.4);
            liftMaster.config_kD(0, 2.8);

            liftMaster.configMotionCruiseVelocity(2900);
            liftMaster.configMotionAcceleration(5200);

            // Define the lift motors as master or slave.
            liftSlavePrimary.follow(liftMaster);
            liftSlaveSecondary.follow(liftMaster);
            liftSlaveTertiary.follow(liftMaster);

            // Initialize the NavX.
            // Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP, or SerialPort.Port.kUSB
            navX = new AHRS(SPI.Port.kMXP);

        } catch (Exception Ex) {

        }


        // Try to open the OSC socket.
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
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    // This function is called periodically during autonomous.
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    // Create actions for the console.
    public void writeConsole(String Line) {

        OSCMessage ConsoleText = new OSCMessage();
        try {
            ConsoleText.setAddress("/Robot/Console/Text");
            ConsoleText.addArgument(Line);
        } catch (Exception Ex) {
            System.out.println("ConsoleText" + Ex.getMessage());
        }
        try {
            oscWirelessSender.send(ConsoleText);
            oscWiredSender.send(ConsoleText);
        } catch (Exception Ex) {

        }
    }

    @Override
    public void teleopInit() {

        liftMaster.setSelectedSensorPosition(0);

    }

    // This function is called periodically during operator control.
    @Override
    public void teleopPeriodic() {
        double driverVertical = QuickMaths.normalizeJoystickWithDeadband(-driverJoystick.getRawAxis(1), 0.05);
        double driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getRawAxis(4), 0.05);
        DriveMotorValues vals = driveHelper.calculateOutput(driverVertical, driverTwist, driverJoystick.getRawButton(6), false);
        try {
            if (driverJoystick.getRawButton(5)) {
                leftMaster.set(vals.leftDrive / 4);
                rightMaster.set(vals.rightDrive / 4);
            } else {
                leftMaster.set(vals.leftDrive);
                rightMaster.set(vals.rightDrive);
            }

            // Set the target lift speed from the operator's joystick.
            double targetLift = operatorJoystick.getRawAxis(1);

        } catch (Exception Ex) {

        }

        try {
            if (operatorJoystick.getRawButtonPressed(1)) {

            }
        } catch (Exception Ex) {

        }

        //Create messages for the errors.
        OSCMessage Error1 = new OSCMessage();

        try {

            Error1.setAddress("/Robot/Error/Test");

            if (driverJoystick.getRawButton(2)) {
                Error1.addArgument(1);
                System.out.println("Sending true...");
            } else {
                Error1.addArgument(0);
                // System.out.println("Sending false...");
            }
            oscWirelessSender.send(Error1);
            oscWiredSender.send(Error1);

        } catch (Exception Ex) {
            System.out.println("Exception in OSC error sending! " + Ex.getMessage());
        }

        try {

            //if (operatorJoystick.getRawButton(1)) {
//            liftMaster.set(ControlMode.PercentOutput, operatorJoystick.getX());
            /*} else if (operatorJoystick.getRawButton(4)) {
                liftMaster.set(ControlMode.PercentOutput, -0.2);
            } else {
                liftMaster.set(ControlMode.PercentOutput, 0);
            }*/

            if (operatorJoystick.getRawButtonPressed(3)) {
                liftMaster.set(ControlMode.MotionMagic, 24000);
            }

            if (operatorJoystick.getRawButtonPressed(1)) {
                liftMaster.set(ControlMode.MotionMagic, 3000);
            }

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
            navXGyroMessage.setAddress("/Robot/NavX/Gyro");
            navXGyroMessage.addArgument(navX.getFusedHeading());

            // Send the current motor values
            leftMotorValueMessage.setAddress("/Robot/Motors/Left/Value");
            leftMotorValueMessage.addArgument(vals.leftDrive);

            rightMotorValueMessage.setAddress("/Robot/Motors/Right/Value");
            rightMotorValueMessage.addArgument(vals.rightDrive);

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

            // Send the lift encoder velocity.
            OSCMessage liftEncoderVelocity = new OSCMessage();

            liftEncoderVelocity.setAddress("/Robot/Motors/liftMaster/EncoderVelocity");
            liftEncoderVelocity.addArgument((double) liftMaster.getSelectedSensorVelocity());

//            oscWiredSender.send(liftEncoderVelocity);
//            oscWirelessSender.send(liftEncoderVelocity);

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

            System.out.println("Currents: " + liftMaster.getOutputCurrent() + " " + liftSlaveSecondary.getOutputCurrent() + " " + liftSlaveTertiary.getOutputCurrent() + " " + liftSlavePrimary.getOutputCurrent());
            System.out.println("Position: " + liftMaster.getSelectedSensorPosition() + " Velocity: " + liftMaster.getSelectedSensorVelocity());

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
            limelightMessageV.addArgument(limelightTarget);
            oscWirelessSender.send(limelightMessageX);
            oscWirelessSender.send(limelightMessageY);
            oscWirelessSender.send(limelightMessageA);
            oscWirelessSender.send(limelightMessageV);

        } catch (Exception Ex) {
            System.out.println("Exception in OSC sending! " + Ex.getMessage());
        }

        try {
            //System.out.println("Test: " + navX.getRawGyroZ());
            // System.out.println("Motor: " + vals.leftDrive + " / " + vals.rightDrive);
        } catch (Exception Ex) {

        }

    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {

    }
}
