package frc.team1071.robot;

import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;

import java.io.ObjectInputFilter;
import java.lang.annotation.Target;
import java.net.InetAddress;

public class CurvatureDrive
{
    private static final double CappedSpeedFeetPerSecond = 12.0;
    private static final double CappedDegreesPerFeet = 55;
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

    private void ConfigureGeneral(CANSparkMax Motor)
    {
        Motor.enableVoltageCompensation(11);
        Motor.setSmartCurrentLimit(55);
        Motor.setOpenLoopRampRate(1.5);
        Motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }


    private void ConfigureMaster(CANSparkMax Master)
    {
        ConfigureGeneral(Master);
    }

    private void ConfigureSlave(CANSparkMax Slave, CANSparkMax Master)
    {
        ConfigureGeneral(Slave);
        Slave.follow(Master);
    }

    private void ConfigureDriveMotors()
    {
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

    public CurvatureDrive(CANSparkMax LeftMaster, CANSparkMax LeftSlaveOne, CANSparkMax LeftSlaveTwo, CANSparkMax RightMaster, CANSparkMax RightSlaveOne, CANSparkMax RightSlaveTwo, AHRS NavX)
    {
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

    private void RunCurvatureMode(double Throttle, double Turn, boolean Brake, double Boost)
    {
        double multiplier = Math.copySign(1,Throttle);
        double BoostIncrease = Boost * (CappedSpeedFeetPerSecond - BoostThresholdFeetPerSecond);
        double TargetSpeed = Throttle * (BoostThresholdFeetPerSecond + BoostIncrease);// ( Boost ? CappedSpeedFeetPerSecond : BoostThresholdFeetPerSecond );
        double TargetCurvature = Turn * CappedDegreesPerFeet;

        System.out.println("BoostIncrease: " + BoostIncrease);

        double TargetDegreesPerSecond = TargetSpeed * TargetCurvature;

        double SpeedDifferential = TargetDegreesPerSecond * FeetPerSecondDifferentialPerDegreesPerSecond;

        double LeftSpeedTarget = TargetSpeed + ((SpeedDifferential / 2) * multiplier);
        double RightSpeedTarget = TargetSpeed - ((SpeedDifferential/ 2) * multiplier);

        double LeftSpeedVoltage = LeftSpeedTarget / FeetPerSecondPerVolt;
        double RightSpeedVoltage = RightSpeedTarget / FeetPerSecondPerVolt;

        double LeftSpeedFeedForward = LeftSpeedVoltage / 11.0;
        double RightSpeedFeedForward = RightSpeedVoltage / 11.0;

        LeftMaster.set(LeftSpeedFeedForward);
        RightMaster.set(RightSpeedFeedForward);
    }

    private void RunQuickTurnMode(double Throttle, double Turn, boolean Brake, double Boost)
    {

    }

    public void Run(double Throttle, double Turn, boolean QuickTurn, boolean Brake, double Boost)
    {
        if(QuickTurn)
        {
            RunQuickTurnMode(Throttle, Turn, Brake, Boost);
        }
        else
        {
            RunCurvatureMode(Throttle, Turn, Brake, Boost);
        }
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
        bundleIdentifier.addArgument("DriveTrain");

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


}
