package frc.team1071.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;
import com.illposed.osc.*;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static java.lang.Math.*;

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
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // Create the joystick for the driver and the operator.
    Joystick driverJoystick = new Joystick(0);
    Joystick operatorJoystick = new Joystick(1);

    // Create the drive motors and set configure them to their PDB number.
    CANSparkMax leftMaster = new CANSparkMax(0, kBrushless);
    CANSparkMax leftSlavePrimary = new CANSparkMax(1, kBrushless);
    CANSparkMax leftSlaveSecondary = new CANSparkMax(2, kBrushless);
    CANSparkMax rightMaster = new CANSparkMax(13, kBrushless);
    CANSparkMax rightSlavePrimary = new CANSparkMax(14, kBrushless);
    CANSparkMax rightSlaveSecondary = new CANSparkMax(15, kBrushless);

    // This function is run when the robot is first started up.
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        // Define the motors as master or slave
        leftSlavePrimary.follow(leftMaster);
        leftSlaveSecondary.follow(leftMaster);
        rightSlavePrimary.follow(rightMaster);
        rightSlaveSecondary.follow(rightMaster);
        rightMaster.setInverted(true);
        rightSlavePrimary.setInverted(true);
        rightSlaveSecondary.setInverted(true);
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
        double driverVertical = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getY(), 0.1);
        double driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getZ(), 0.1);
        DriveHelper driveHelper = new DriveHelper();
        DriveMotorValues vals = driveHelper.calculateOutput(driverVertical, driverTwist, driverJoystick.getTrigger(), false);
        leftMaster.set(vals.leftDrive);
        rightMaster.set(vals.rightDrive);
        System.out.println("Motor: " + vals.leftDrive + " / " + vals.rightDrive);
    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {

    }
}
