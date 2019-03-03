package frc.team1071.robot;

import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;

import java.net.InetAddress;

public class CurvatureDrive {
    private static final double CappedSpeedFeetPerSecond = 12.0;
    private static final double CappedDegreesPerFeet = 45;
    private static final double BoostThresholdFeetPerSecond = 6.0;

    private static final double WheelDiameterFeet = 0.5;
    private static final double RevolutionPerMinutePerVolt = 470;
    //private static final double RevolutionPerMinutePerSecondPerVolt = 1300;
    private static final double GearRatio = 8.680555556;
    private static final double DegreesPerFeetPerSecondPerFeetPerSecondDifferential = 27.64219461;
    private static final double FeetPerSecondDifferentialPerDegreesPerSecond = 0.036176578;

    private static final double RPMtoFeetPerSecond = WheelDiameterFeet * Math.PI / 60 / GearRatio;
    private static final double FeetPerSecondPerVolt = RevolutionPerMinutePerVolt * RPMtoFeetPerSecond;
    //private static final double FeetPerSecondPerSecondPerVolt = RevolutionPerMinutePerSecondPerVolt * RPMtoFeetPerSecond;

    private CANSparkMax LeftMaster;
    private CANSparkMax LeftSlaveOne;
    private CANSparkMax LeftSlaveTwo;

    private CANSparkMax RightMaster;
    private CANSparkMax RightSlaveOne;
    private CANSparkMax RightSlaveTwo;

    private CANEncoder RightEncoder;
    private CANEncoder LeftEncoder;

    private AHRS NavX;

    private OSCPortOut oscWirelessSender;
    private OSCPortOut oscWiredSender;

    double InputThrottle = 0;
    double InputTurn = 0;
    boolean InputBrake = false;
    double InputBoost = 0;
    boolean InputQuickTurn = false;

    double LeftSpeedFeedForward = 0;
    double RightSpeedFeedForward = 0;

    private void ConfigureGeneral(CANSparkMax Motor) {
        Motor.enableVoltageCompensation(11);
        Motor.setSmartCurrentLimit(55);
        Motor.setOpenLoopRampRate(1);
        Motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }


    private void ConfigureMaster(CANSparkMax Master) {
        ConfigureGeneral(Master);
    }

    private void ConfigureSlave(CANSparkMax Slave, CANSparkMax Master) {
        ConfigureGeneral(Slave);
        Slave.follow(Master);
    }

    private void ConfigureDriveMotors() {
        ConfigureMaster(this.RightMaster);
        ConfigureSlave(this.RightSlaveOne, this.RightMaster);
        ConfigureSlave(this.RightSlaveTwo, this.RightMaster);

        ConfigureMaster(this.LeftMaster);
        ConfigureSlave(this.LeftSlaveOne, this.LeftMaster);
        ConfigureSlave(this.LeftSlaveTwo, this.LeftMaster);

        // Invert the right side of the drive train.
        this.RightMaster.setInverted(true);
        this.RightSlaveOne.setInverted(true);
        this.RightSlaveTwo.setInverted(true);
    }

    public CurvatureDrive(CANSparkMax LeftMaster, CANSparkMax LeftSlaveOne, CANSparkMax LeftSlaveTwo, CANSparkMax RightMaster, CANSparkMax RightSlaveOne, CANSparkMax RightSlaveTwo, AHRS NavX) {
        this.LeftMaster = LeftMaster;
        this.LeftSlaveOne = LeftSlaveOne;
        this.LeftSlaveTwo = LeftSlaveTwo;
        this.RightMaster = RightMaster;
        this.RightSlaveOne = RightSlaveOne;
        this.RightSlaveTwo = RightSlaveTwo;
        this.RightEncoder = this.RightMaster.getEncoder();
        this.LeftEncoder = this.LeftMaster.getEncoder();
        this.NavX = NavX;

        ConfigureDriveMotors();

        try {
            oscWirelessSender = new OSCPortOut(InetAddress.getByName("10.10.71.9"), 5803);
            oscWiredSender = new OSCPortOut(InetAddress.getByName("10.10.71.5"), 5803);
        } catch (Exception Ex) {
            System.out.println("OSC Initialization Exception: " + Ex.getMessage());
        }
    }

    private void RunCurvatureMode(double Throttle, double Turn, boolean Brake, double Boost) {
        double multiplier = Math.copySign(1, Throttle);
        double BoostIncrease = Boost * (CappedSpeedFeetPerSecond - BoostThresholdFeetPerSecond);
        double TargetSpeed = Throttle * (BoostThresholdFeetPerSecond + BoostIncrease);// ( Boost ? CappedSpeedFeetPerSecond : BoostThresholdFeetPerSecond );
        double TargetCurvature = Turn * CappedDegreesPerFeet;

        double TargetDegreesPerSecond = TargetSpeed * TargetCurvature;

        double SpeedDifferential = TargetDegreesPerSecond * FeetPerSecondDifferentialPerDegreesPerSecond;

        double LeftSpeedTarget = TargetSpeed + ((SpeedDifferential / 2) * multiplier);
        double RightSpeedTarget = TargetSpeed - ((SpeedDifferential / 2) * multiplier);

        double LeftSpeedVoltage = LeftSpeedTarget / FeetPerSecondPerVolt;
        double RightSpeedVoltage = RightSpeedTarget / FeetPerSecondPerVolt;

        LeftSpeedFeedForward = LeftSpeedVoltage / 11.0;
        RightSpeedFeedForward = RightSpeedVoltage / 11.0;

        LeftMaster.set(LeftSpeedFeedForward);
        RightMaster.set(RightSpeedFeedForward);
    }

    private void RunQuickTurnMode(double Throttle, double Turn, boolean Brake, double Boost) {
        double multiplier = 1;//Math.copySign(1, Throttle);
        double BoostIncrease = Boost * (CappedSpeedFeetPerSecond - BoostThresholdFeetPerSecond);
        double TargetSpeed = Throttle * (BoostThresholdFeetPerSecond + BoostIncrease);// ( Boost ? CappedSpeedFeetPerSecond : BoostThresholdFeetPerSecond );
        double TargetCurvature = Turn * CappedDegreesPerFeet;

        double TargetDegreesPerSecond = (BoostThresholdFeetPerSecond + BoostIncrease) * TargetCurvature;

        double SpeedDifferential = TargetDegreesPerSecond * FeetPerSecondDifferentialPerDegreesPerSecond;

        double LeftSpeedTarget = TargetSpeed + ((SpeedDifferential / 2) * multiplier);
        double RightSpeedTarget = TargetSpeed - ((SpeedDifferential / 2) * multiplier);

        double LeftSpeedVoltage = LeftSpeedTarget / FeetPerSecondPerVolt;
        double RightSpeedVoltage = RightSpeedTarget / FeetPerSecondPerVolt;

        LeftSpeedFeedForward = LeftSpeedVoltage / 11.0;
        RightSpeedFeedForward = RightSpeedVoltage / 11.0;

        LeftMaster.set(LeftSpeedFeedForward);
        RightMaster.set(RightSpeedFeedForward);
    }

    public void Run(double Throttle, double Turn, boolean QuickTurn, boolean Brake, double Boost) {
        if (QuickTurn) {
            RunQuickTurnMode(Throttle, Turn, Brake, Boost);
        } else {
            RunCurvatureMode(Throttle, Turn, Brake, Boost);
        }

        InputThrottle = Throttle;
        InputTurn = Turn;
        InputBrake = Brake;
        InputBoost = Boost;
        InputQuickTurn = QuickTurn;
        SendDriveData();
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
        bundleIdentifier.addArgument("DriveTrainLog");

        // Append the robot timestamp for the data.
        OSCMessage timestamp = new OSCMessage();
        timestamp.setAddress("/timestamp");
        timestamp.addArgument(Timer.getFPGATimestamp());

        // Append the left encoder data.
        CANEncoder leftEncoder = LeftMaster.getEncoder();
        OSCMessage leftVelocity = new OSCMessage();
        leftVelocity.setAddress("/LeftVelocity");
        leftVelocity.addArgument(leftEncoder.getVelocity());

        // Append the right encoder data.
        CANEncoder rightEncoder = RightMaster.getEncoder();
        OSCMessage rightVelocity = new OSCMessage();
        rightVelocity.setAddress("/RightVelocity");
        rightVelocity.addArgument(rightEncoder.getVelocity());

        // Append the bus voltage.
        OSCMessage Voltage = new OSCMessage();
        Voltage.setAddress("/Voltage");
        Voltage.addArgument(LeftMaster.getBusVoltage());

        // Append the fused heading from the Nav X.
        OSCMessage fusedHeading = new OSCMessage();
        fusedHeading.setAddress("/FusedHeading");
        fusedHeading.addArgument((double) NavX.getFusedHeading());

        // Append the input throttle
        OSCMessage InputThrottleMsg = new OSCMessage();
        InputThrottleMsg.setAddress("/InputThrottle");
        InputThrottleMsg.addArgument((double) InputThrottle);

        // Append the input turn
        OSCMessage InputTurnMsg = new OSCMessage();
        InputTurnMsg.setAddress("/InputTurn");
        InputTurnMsg.addArgument((double) InputTurn);

        // Append the input brake
        OSCMessage InputBrakeMsg = new OSCMessage();
        InputBrakeMsg.setAddress("/InputBrake");
        InputBrakeMsg.addArgument((double)(InputBrake ? 0 : 1));

        // Append the input boost
        OSCMessage InputBoostMsg = new OSCMessage();
        InputBoostMsg.setAddress("/InputBoost");
        InputBoostMsg.addArgument((double) InputBoost);

        // Append the input quickturn
        OSCMessage InputQT = new OSCMessage();
        InputQT.setAddress("/InputQT");
        InputQT.addArgument((double) (InputQuickTurn ? 0 : 1));

        // Append the left FF
        OSCMessage LeftFF = new OSCMessage();
        LeftFF.setAddress("/LeftFF");
        LeftFF.addArgument((double) LeftSpeedFeedForward);

        // Append the right FF
        OSCMessage RightFF = new OSCMessage();
        RightFF.setAddress("/RightFF");
        RightFF.addArgument((double) RightSpeedFeedForward);

        // Add these packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(timestamp);
        bundle.addPacket(leftVelocity);
        bundle.addPacket(Voltage);
        bundle.addPacket(rightVelocity);
        bundle.addPacket(fusedHeading);
        bundle.addPacket(InputThrottleMsg);
        bundle.addPacket(InputTurnMsg);
        bundle.addPacket(InputBrakeMsg);
        bundle.addPacket(InputBoostMsg);
        bundle.addPacket(InputQT);
        bundle.addPacket(LeftFF);
        bundle.addPacket(RightFF);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the drive log data! " + ex.getMessage());
        }
    }

    double getLeftEncoderVelocity() {
        return LeftEncoder.getVelocity();
    }

    double getRightEncoderVelocity() {
        return RightEncoder.getVelocity();
    }

    /**
     * Sums the current pulled by all the drive motors.
     * @return The total current drawn by the drive train.
     */
    double getCurrent() {

        double totalCurrent = 0;

        totalCurrent += LeftMaster.getOutputCurrent();
        totalCurrent += RightMaster.getOutputCurrent();
        totalCurrent += LeftSlaveOne.getOutputCurrent();
        totalCurrent += RightSlaveOne.getOutputCurrent();
        totalCurrent += LeftSlaveTwo.getOutputCurrent();
        totalCurrent += RightSlaveTwo.getOutputCurrent();

        return totalCurrent;
    }

}
