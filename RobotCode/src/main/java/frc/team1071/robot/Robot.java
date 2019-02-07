package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
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
    private TalonSRX fourBarMotor;

    // Helper class for calculating drive motor speeds.
    private DriveHelper driveHelper = new DriveHelper();

    // Create the joysticks for the driver and the operator.
    private Joystick driverJoystick = new Joystick(0);
    private Joystick operatorJoystick = new Joystick(1);

    // Create the drive motors.
    private CANSparkMax leftMaster;
    private CANSparkMax rightMaster;

    /**
     * Function that configures a lift motor's power.
     *
     * @param liftTalon The talon being power limited.
     */
    private static void configLiftMotorPower(TalonSRX liftTalon) {
        liftTalon.configPeakCurrentLimit(60);
        liftTalon.configPeakCurrentDuration(1200);
        liftTalon.configContinuousCurrentLimit(30);
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

        // Initialize all the motor controllers.
        try {

            //----------------------------------------------------------------------------------------------------------
            // Drive Motors
            //----------------------------------------------------------------------------------------------------------

            // Initialize the drive motors.
            leftMaster = new CANSparkMax(0, kBrushless);
            CANSparkMax leftSlavePrimary = new CANSparkMax(1, kBrushless);
            CANSparkMax leftSlaveSecondary = new CANSparkMax(2, kBrushless);

            rightMaster = new CANSparkMax(13, kBrushless);
            CANSparkMax rightSlavePrimary = new CANSparkMax(14, kBrushless);
            CANSparkMax rightSlaveSecondary = new CANSparkMax(15, kBrushless);

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
            fourBarMotor = new TalonSRX(5);

            // Set the encoder mode to absolute position.
            fourBarMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            fourBarMotor.setInverted(true);
            fourBarMotor.setSensorPhase(true);

            // Set the PID values for the four bar.
            fourBarMotor.config_kF(0, 2);
            fourBarMotor.config_kP(0, 1.5);// 3.5 / 2);
            fourBarMotor.config_kD(0, 4);// 7 / 2);

            // Establish the cruise velocity and max acceleration for the motion magic.
            fourBarMotor.configMotionCruiseVelocity(100);
            fourBarMotor.configMotionAcceleration(200);

            // Current limit the four bar motor.
            fourBarMotor.configPeakCurrentLimit(0);
            fourBarMotor.configPeakCurrentDuration(0);
            fourBarMotor.configContinuousCurrentLimit(5);
            fourBarMotor.enableCurrentLimit(true);

            //----------------------------------------------------------------------------------------------------------
            // Other Initialization
            //----------------------------------------------------------------------------------------------------------

            // Initialize the gatherer.
            gathererMotor = new TalonSRX(6);

            // Initialize the NavX.
            navX = new AHRS(SPI.Port.kMXP);

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

        // Print the Limelight area.
        //System.out.println(limelightArea);
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

        // TODO REMOVE
        //fourBarMotor.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, false);
        fourBarMotor.setSelectedSensorPosition(fourBarMotor.getSensorCollection().getPulseWidthPosition());

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        // Print the four bar arm's exact position.
        System.out.println("Four Bar Arm Position: " + fourBarMotor.getSelectedSensorPosition() + " Target: " + fourBarMotor.getActiveTrajectoryPosition() + " Duty Cycle: " + fourBarMotor.getMotorOutputPercent() + " Speed: " + fourBarMotor.getSelectedSensorVelocity());

        //--------------------------------------------------------------------------------------------------------------
        // Drive Controls
        //--------------------------------------------------------------------------------------------------------------

        // Determine the proper motor values based on the joystick data.
        double driverVertical = QuickMaths.normalizeJoystickWithDeadband(-driverJoystick.getRawAxis(1), 0.05);
        double driverTwist = QuickMaths.normalizeJoystickWithDeadband(driverJoystick.getRawAxis(4), 0.05);
        DriveMotorValues motorValues = driveHelper.calculateOutput(driverVertical, driverTwist, driverJoystick.getRawButton(6), false);

        try {
            leftMaster.set(motorValues.leftDrive / 4);
            rightMaster.set(motorValues.rightDrive / 4);
        } catch (Exception Ex) {
            System.out.println("Drive Motor Exception: " + Ex.getMessage());
        }

        //--------------------------------------------------------------------------------------------------------------
        // Operator Controls
        //--------------------------------------------------------------------------------------------------------------

        try {

            // Four bar set positions.
            int fourBarGatheringPosition = 3332;
            int fourBarHighScore = 4500;

            // Lift set positions.
            int liftGatheringPosition = 0;
            int liftHighScore = 24500;

            //when the right bumper is pressed, set the lift to ball positions
            if (operatorJoystick.getRawButton(6)) {

                // If the 'A' button is pressed, then set the mechanism to gathering height.
                if (operatorJoystick.getRawButtonPressed(1)) {
                    liftMaster.set(ControlMode.MotionMagic, liftGatheringPosition);
                    fourBarMotor.set(ControlMode.MotionMagic, fourBarGatheringPosition);
                }

                // If the 'B' button is pressed, then set the lift to about max height.
                if (operatorJoystick.getRawButtonPressed(2)) {
                    liftMaster.set(ControlMode.MotionMagic, liftHighScore);
                }

                // If the 'X' button is pressed, then set the four bar to gathering position.
                if (operatorJoystick.getRawButtonPressed(3)) {
                    System.out.println("Setting DOwn");
                    fourBarMotor.set(ControlMode.MotionMagic, fourBarGatheringPosition);
                }

                // If the 'Y' button is pressed, then set the four bar to the up position and lift to the highest position.
                if (operatorJoystick.getRawButtonPressed(4)) {
                    System.out.println("Setting Up");
                    fourBarMotor.set(ControlMode.MotionMagic, fourBarHighScore);
                    liftMaster.set(ControlMode.MotionMagic, liftHighScore);
                }

            }

            if (operatorJoystick.getRawAxis(2) > 0.1) {
                gathererMotor.set(ControlMode.PercentOutput, operatorJoystick.getRawAxis(2));
            } else if (operatorJoystick.getRawAxis(3) > 0.1) {
                gathererMotor.set(ControlMode.PercentOutput, -operatorJoystick.getRawAxis(3));
            } else {
                gathererMotor.set(ControlMode.PercentOutput, 0);
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
            limelightMessageV.addArgument(limelightTarget);
            oscWirelessSender.send(limelightMessageX);
            oscWirelessSender.send(limelightMessageY);
            oscWirelessSender.send(limelightMessageA);
            oscWirelessSender.send(limelightMessageV);

        } catch (Exception Ex) {
            System.out.println("Exception in OSC sending! " + Ex.getMessage());
        }

        try {
            // System.out.println("Test: " + navX.getRawGyroZ());
            // System.out.println("Motor: " + vals.leftDrive + " / " + vals.rightDrive);
        } catch (Exception Ex) {

        }

    }

    // This function is called periodically during test mode.
    @Override
    public void testPeriodic() {

    }
}
