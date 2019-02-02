package frc.team1071.robot;

import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
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

    // Create the NavX.
    AHRS navX;

    // Create the OSC sender on the robot.
    OSCPortOut oscWirelessSender;
    OSCPortOut oscWiredSender;

    private String m_autoSelected;

    // This function is run when the robot is first started up.
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        // Initialize all the motor controllers.
        try {

            // Initialize the left drive motor controllers.
            leftMaster = new CANSparkMax(0, kBrushless);
            leftSlavePrimary = new CANSparkMax(1, kBrushless);
            leftSlaveSecondary = new CANSparkMax(2, kBrushless);
            rightMaster = new CANSparkMax(13, kBrushless);
            rightSlavePrimary = new CANSparkMax(14, kBrushless);
            rightSlaveSecondary = new CANSparkMax(15, kBrushless);

            // Define the motors as master or slave
            leftSlavePrimary.follow(leftMaster);
            leftSlaveSecondary.follow(leftMaster);
            rightSlavePrimary.follow(rightMaster);
            rightSlaveSecondary.follow(rightMaster);
            rightMaster.setInverted(true);
            rightSlavePrimary.setInverted(true);
            rightSlaveSecondary.setInverted(true);

            // Initialize the NavX.
            // Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP, or SerialPort.Port.kUSB
            navX = new AHRS(SPI.Port.kMXP);

        } catch (Exception Ex) {

        }


        // Try to open the OSC socket.
        try {
            oscWirelessSender = new OSCPortOut(InetAddress.getByName("10.10.71.9"), 5801);
            oscWiredSender = new OSCPortOut(InetAddress.getByName("10.10.71.5"), 5801);
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
        } catch (Exception Ex) {

        }

        try {
            if (operatorJoystick.getRawButton(1)) {

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

            // Send the message.
            // TODO: Bundle these in the future.
            oscWirelessSender.send(leftMotorValueMessage);
            oscWiredSender.send(leftMotorValueMessage);
            oscWirelessSender.send(rightMotorValueMessage);
            oscWiredSender.send(rightMotorValueMessage);
            oscWirelessSender.send(leftMasterCurrentMessage);
            oscWiredSender.send(leftMasterCurrentMessage);
            oscWirelessSender.send(rightMasterCurrentMessage);
            oscWiredSender.send(rightMasterCurrentMessage);
            oscWirelessSender.send(navXGyroMessage);
            oscWiredSender.send(navXGyroMessage);
            oscWirelessSender.send(ControllerButtonsMessage);
            oscWiredSender.send(ControllerButtonsMessage);

        } catch (
                Exception Ex) {
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
