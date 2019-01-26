package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
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

    PowerDistributionPanel PDP = new PowerDistributionPanel();

    // Create the drive motors.
    CANSparkMax leftMaster;
    CANSparkMax leftSlavePrimary;
    CANSparkMax leftSlaveSecondary;
    CANSparkMax rightMaster;
    CANSparkMax rightSlavePrimary;
    CANSparkMax rightSlaveSecondary;

    // Create the gathering motors.
    TalonSRX leftGatherer;
    TalonSRX rightGatherer;

    // Create the gatherer solenoid and its state variable.
    Solenoid gathererSolenoidOpen;
    Solenoid gathererSolenoidClose;
    Boolean gathererOpen = false;

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


        try {

            // Initialize the left drive motor controllers.
            leftMaster = new CANSparkMax(0, kBrushless);
            leftSlavePrimary = new CANSparkMax(1, kBrushless);
            leftSlaveSecondary = new CANSparkMax(2, kBrushless);

            // Initialize the right drive motor controllers.
            rightMaster = new CANSparkMax(13, kBrushless);
            rightSlavePrimary = new CANSparkMax(14, kBrushless);
            rightSlaveSecondary = new CANSparkMax(15, kBrushless);

            // Initialize the gathering motor controllers.
            leftGatherer = new TalonSRX(10);
            rightGatherer = new TalonSRX(11);

            // Define the motors as master or slave
            leftSlavePrimary.follow(leftMaster);
            leftSlaveSecondary.follow(leftMaster);
            rightSlavePrimary.follow(rightMaster);
            rightSlaveSecondary.follow(rightMaster);
            rightMaster.setInverted(true);
            rightSlavePrimary.setInverted(true);
            rightSlaveSecondary.setInverted(true);

            // Have the right gatherer follow the left gatherer, but also invert it.
            rightGatherer.follow(leftGatherer);
            rightGatherer.setInverted(true);

            // Initialize the gatherer solenoid.
            gathererSolenoidOpen = new Solenoid(0);
            gathererSolenoidClose = new Solenoid(7);
            gathererSolenoidOpen.set(gathererOpen);
            gathererSolenoidClose.set(!gathererOpen);

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

    // This function is called periodically during operator control.
    @Override
    public void teleopPeriodic() {
        double driverVertical = QuickMaths.normalizeJoystickWithDeadband(-driverJoystick.getRawAxis(1), 0.05);
        double driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getRawAxis(4), 0.05);
        DriveMotorValues vals = driveHelper.calculateOutput(driverVertical, driverTwist, driverJoystick.getRawButton(6), false);

        try {

            // Attach trigger presses to the gathering speed.
            if (driverJoystick.getRawAxis(2) > 0.1) {
                leftGatherer.set(ControlMode.PercentOutput, -driverJoystick.getRawAxis(2));
            } else if (driverJoystick.getRawAxis(3) > 0.1) {
                leftGatherer.set(ControlMode.PercentOutput, driverJoystick.getRawAxis(3));
            } else {
                leftGatherer.set(ControlMode.PercentOutput, 0);
            }

            // Set the motors to quarter speed.
            leftMaster.set(vals.leftDrive / 4);
            rightMaster.set(vals.rightDrive / 4);

            // When the "A" button is pressed, actuate the gatherer.
            if (driverJoystick.getRawButtonPressed(1)) {
                gathererOpen = !gathererOpen;
                gathererSolenoidOpen.set(gathererOpen);
                gathererSolenoidClose.set(!gathererOpen);
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

        // Create messages for the current motor values.
        OSCMessage leftMotorValueMessage = new OSCMessage();
        OSCMessage rightMotorValueMessage = new OSCMessage();
        OSCMessage leftMasterCurrentMessage = new OSCMessage();

        try {

            // Set the current motor values in the OSC messages.
            leftMotorValueMessage.setAddress("/Robot/Motors/Left/Value");
            leftMotorValueMessage.addArgument(vals.leftDrive);

            rightMotorValueMessage.setAddress("/Robot/Motors/Right/Value");
            rightMotorValueMessage.addArgument(vals.rightDrive);

            // Send Current Meter values
            // left Master
            leftMasterCurrentMessage.setAddress("/Robot/Motors/LeftMaster/Current");
            leftMasterCurrentMessage.addArgument(leftMaster.getOutputCurrent());
            // left Slave Primary
            leftMasterCurrentMessage.setAddress("/Robot/Motors/LeftSlavePrimary/Current");
            leftMasterCurrentMessage.addArgument(leftSlavePrimary.getOutputCurrent());
            // left Slave Secondary
            leftMasterCurrentMessage.setAddress("/Robot/Motors/LeftSlaveSecondary/Current");
            leftMasterCurrentMessage.addArgument(leftSlaveSecondary.getOutputCurrent());
            // right Master
            leftMasterCurrentMessage.setAddress("/Robot/Motors/RightMaster/Current");
            leftMasterCurrentMessage.addArgument(rightMaster.getOutputCurrent());
            // right Slave Primary
            leftMasterCurrentMessage.setAddress("/Robot/Motors/RightSlavePrimary/Current");
            leftMasterCurrentMessage.addArgument(rightSlavePrimary.getOutputCurrent());
            // right Slave Secondary
            leftMasterCurrentMessage.setAddress("/Robot/Motors/RightSlaveSecondary/Current");
            leftMasterCurrentMessage.addArgument(rightSlaveSecondary.getOutputCurrent());

            // Send the message.
            // TODO: Bundle these in the future.
            oscWirelessSender.send(leftMotorValueMessage);
            oscWiredSender.send(leftMotorValueMessage);
            oscWirelessSender.send(rightMotorValueMessage);
            oscWiredSender.send(rightMotorValueMessage);

        } catch (Exception Ex) {
            System.out.println("Exception in OSC sending! " + Ex.getMessage());
        }

        try {
            // System.out.println("Motor: " + vals.leftDrive + " / " + vals.rightDrive);
        } catch (Exception Ex) {

        }

    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {

    }
}
