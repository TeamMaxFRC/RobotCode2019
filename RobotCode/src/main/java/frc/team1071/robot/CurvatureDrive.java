package frc.team1071.robot;

import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Timer;

import java.net.InetAddress;

public class CurvatureDrive
{
    // Personality characteristics
    private static final double CappedSpeedFeetPerSecond = 12.0;
    private static final double CappedDegreesPerFeet = 55;
    private static final double BoostThresholdFeetPerSecond = 6.0;
    private static final double SlowDownFeetPerSecond = 4.0;
    private static final double BrakingThreshold = 2.0;
    private static final double AccelerationFeetPerSecondPerSecond = 4.0;

    // Motor Characteristics
    private static final double FreeRPM = 5676;
    private static final double StallTorqueNm = 2.6;

    // GearBox Characteristics
    private static final double GearRatio = 8.680555556;
    private static final double MotorsPerSide = 3;

    // Robot Characteristics
    private static final double RobotWeightLbs = 125;
    private static final double DriveTrainBaseWidthInches = 28;
    private static final double WheelDiameterInches = 6;

    // Universal constants
    private static final double InchesToMeters = 0.0254;
    private static final double FeetToMeters = 0.3048;
    private static final double LbToKg = 0.453592;
    private static final double MinutesToSeconds = 1/60.0;
    private static final double MotorMeasurementVoltage = 12;

    // Derived Values
    private static final double FreeWheelRPM = FreeRPM / GearRatio;
    private static final double WheelDiameterMeters = WheelDiameterInches * InchesToMeters;
    private static final double DriveTrainBaseWidthMeters = DriveTrainBaseWidthInches * InchesToMeters;
    private static final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;
    private static final double WheelRadiusMeters = WheelDiameterMeters / 2;
    private static final double FreeWheelMetersPerSecond = FreeWheelRPM * WheelCircumferenceMeters * MinutesToSeconds;
    private static final double CappedSpeedMetersPerSecond = CappedSpeedFeetPerSecond * FeetToMeters;
    private static final double BoostThresholdMetersPerSecond = BoostThresholdFeetPerSecond * FeetToMeters;
    private static final double RobotWeightKgs = RobotWeightLbs * LbToKg;
    private static final double MetersPerSecondPerVolt = FreeWheelMetersPerSecond / MotorMeasurementVoltage;
    private static final double NewtonsPerVolt = (StallTorqueNm / MotorMeasurementVoltage) / WheelRadiusMeters; 
    private static final double MetersPerSecondPerSecondPerVolt = NewtonsPerVolt * MotorsPerSide * 2 / RobotWeightKgs;
    private static final double CappedDegreesPerMeter = CappedDegreesPerFeet / FeetToMeters;
    private static final double AccelerationMetersPerSecondPerSecond = AccelerationFeetPerSecondPerSecond * FeetToMeters;

    private CANSparkMax LeftMaster;
    private CANSparkMax LeftSlaveOne;
    private CANSparkMax LeftSlaveTwo;

    private CANSparkMax RightMaster;
    private CANSparkMax RightSlaveOne;
    private CANSparkMax RightSlaveTwo;

    private CANPIDController LeftController;
    private CANPIDController RightController;

    private CANEncoder RightEncoder;
    private CANEncoder LeftEncoder;

    private AHRS NavX;

    private OSCPortOut oscWirelessSender;
    private OSCPortOut oscWiredSender;

    private double LastRunTime = 0;
    private double TargetSpeedMetersPerSecond = 0;
    private double TargetDegreesPerMeter = 0;
    private boolean Brake = false;
    private boolean QuickTurn = false;
    
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
        this.LeftController = this.LeftMaster.getPIDController();
        this.RightController = this.RightMaster.getPIDController();

        this.LeftController.setP(0);
        this.LeftController.setI(0);
        this.LeftController.setD(0);
        this.LeftController.setFF(0);
        this.LeftController.setDFilter(0.25);
        this.LeftController.setOutputRange(-1, 1);

        this.RightController.setP(0);
        this.RightController.setI(0);
        this.RightController.setD(0);
        this.RightController.setFF(0);
        this.RightController.setDFilter(0.25);
        this.LeftController.setOutputRange(-1, 1);

        ConfigureDriveMotors();

        try {
            oscWirelessSender = new OSCPortOut(InetAddress.getByName("10.10.71.9"), 5803);
            oscWiredSender = new OSCPortOut(InetAddress.getByName("10.10.71.5"), 5803);
        } catch (Exception Ex) {
            System.out.println("OSC Initialization Exception: " + Ex.getMessage());
        }
    }

    public void Set(double Throttle, double Turn, boolean Brake, double Boost, boolean QuickTurn, boolean CarMode)
    {
        Turn *= CarMode ? 1 : Math.copySign(1, Throttle);

        double BoostIncreaseMeters = Boost * (CappedSpeedMetersPerSecond - BoostThresholdMetersPerSecond);
        this.TargetSpeedMetersPerSecond = Throttle * (BoostThresholdMetersPerSecond + BoostIncreaseMeters);
        this.TargetDegreesPerMeter = Turn * CappedDegreesPerMeter;
        this.Brake = Brake;
        this.QuickTurn = QuickTurn;
    }

    public void Run()
    {
        double NewLeftMetersPerSecond = 0;
        double NewRightMetersPerSecond = 0;
        double TotalLeftFeedForward = 0;
        double TotalRightFeedForward = 0;

        double Circumference = 360.0 / Math.abs(this.TargetDegreesPerMeter);

        double Radius = Circumference / Math.PI / 2;
        double ShortRadius = Radius - (DriveTrainBaseWidthMeters / 2);
        double WideRadius = Radius + (DriveTrainBaseWidthMeters / 2);

        // Can cheat here and use the ratio of the radiuses to determine wheel speed
        double ShortWheelMetersPerSecond = (ShortRadius / Radius) * TargetSpeedMetersPerSecond;
        double WideWheelMetersPerSecond = (WideRadius / Radius) * TargetSpeedMetersPerSecond;

        if(TargetDegreesPerMeter == 0)
        {
            ShortWheelMetersPerSecond = TargetSpeedMetersPerSecond;
            WideWheelMetersPerSecond = TargetSpeedMetersPerSecond;
        }
        
        double TargetLeftMetersPerSecond = TargetDegreesPerMeter >= 0 ? WideWheelMetersPerSecond : ShortWheelMetersPerSecond;
        double TargetRightMetersPerSecond = TargetDegreesPerMeter >= 0 ? ShortWheelMetersPerSecond : WideWheelMetersPerSecond;

        double CurrentLeftMetersPerSecond = LeftEncoder.getVelocity() / GearRatio * WheelCircumferenceMeters * MinutesToSeconds;
        double CurrentRightMetersPerSecond = RightEncoder.getVelocity() / GearRatio * WheelCircumferenceMeters * MinutesToSeconds;

        // System.out.print(" TargetLeftFeetPerSecond: " + TargetLeftMetersPerSecond / FeetToMeters);

        double TimeNow = Timer.getFPGATimestamp();
        double TimeDelta = TimeNow - LastRunTime;
        LastRunTime = TimeNow;

        NewLeftMetersPerSecond = CurrentLeftMetersPerSecond;
        NewRightMetersPerSecond = CurrentRightMetersPerSecond;

        double LeftAccelerationMetersPerSecondPerSecond = 0;
        double RightAccelerationMetersPerSecondPerSecond = 0;

        // If it's been longer than a second since the last update... just use the last values
        // This covers things like being in disabled, booting, etc

        if(TimeDelta < 1.0)
        {
            double DeltaVelocityMetersPerSecond = AccelerationMetersPerSecondPerSecond * TimeDelta;
            double ShortDeltaVelocityMetersPerSecond = DeltaVelocityMetersPerSecond * (ShortRadius / Radius);
            double WideDeltaVelocityMetersPerSecond = DeltaVelocityMetersPerSecond * (WideRadius / Radius);

            double LeftDeltaVelocityMetersPerSecond = TargetDegreesPerMeter >= 0 ? WideDeltaVelocityMetersPerSecond : ShortDeltaVelocityMetersPerSecond;
            double RightDeltaVelocityMetersPerSecond = TargetDegreesPerMeter >= 0 ? ShortDeltaVelocityMetersPerSecond : WideDeltaVelocityMetersPerSecond;

            LeftAccelerationMetersPerSecondPerSecond = AccelerationMetersPerSecondPerSecond * (TargetDegreesPerMeter >= 0 ? WideRadius/Radius : ShortRadius / Radius); 
            RightAccelerationMetersPerSecondPerSecond = AccelerationMetersPerSecondPerSecond * (TargetDegreesPerMeter >= 0 ? ShortRadius/Radius : WideRadius / Radius);

            if(TargetDegreesPerMeter == 0)
            {
                LeftAccelerationMetersPerSecondPerSecond = AccelerationMetersPerSecondPerSecond;
                RightAccelerationMetersPerSecondPerSecond = AccelerationMetersPerSecondPerSecond;
                LeftDeltaVelocityMetersPerSecond = DeltaVelocityMetersPerSecond;
                RightDeltaVelocityMetersPerSecond = DeltaVelocityMetersPerSecond;
            }

            if(Math.abs(TargetLeftMetersPerSecond - CurrentLeftMetersPerSecond) < Math.abs(LeftDeltaVelocityMetersPerSecond * 5) || TargetLeftMetersPerSecond == 0)
            {
                NewLeftMetersPerSecond = TargetLeftMetersPerSecond;
                LeftAccelerationMetersPerSecondPerSecond = 0;
            }
            else
            {
                NewLeftMetersPerSecond = CurrentLeftMetersPerSecond + Math.copySign(LeftDeltaVelocityMetersPerSecond, TargetLeftMetersPerSecond - CurrentLeftMetersPerSecond);
            }

            if(Math.abs(TargetRightMetersPerSecond - CurrentRightMetersPerSecond) < RightDeltaVelocityMetersPerSecond * 5 || TargetRightMetersPerSecond == 0)
            {
                NewRightMetersPerSecond = TargetRightMetersPerSecond;
                RightAccelerationMetersPerSecondPerSecond = 0;
            }
            else
            {
                NewRightMetersPerSecond = CurrentRightMetersPerSecond + Math.copySign(RightDeltaVelocityMetersPerSecond, TargetRightMetersPerSecond - CurrentRightMetersPerSecond);
            }

            LeftAccelerationMetersPerSecondPerSecond = Math.copySign(LeftAccelerationMetersPerSecondPerSecond, TargetLeftMetersPerSecond - CurrentLeftMetersPerSecond);
            RightAccelerationMetersPerSecondPerSecond = Math.copySign(RightAccelerationMetersPerSecondPerSecond, TargetRightMetersPerSecond - CurrentRightMetersPerSecond);
        }

        double LeftSpeedVoltage = NewLeftMetersPerSecond / MetersPerSecondPerVolt;
        double RightSpeedVoltage = NewRightMetersPerSecond / MetersPerSecondPerVolt;

        double LeftSpeedFeedForward = LeftSpeedVoltage / 11.0;
        double RightSpeedFeedForward = RightSpeedVoltage / 11.0;

        double LeftAccelerationVoltage = LeftAccelerationMetersPerSecondPerSecond / MetersPerSecondPerSecondPerVolt;
        double RightAccelerationVoltage = RightAccelerationMetersPerSecondPerSecond / MetersPerSecondPerSecondPerVolt;

        // System.out.print(" Meters/s/s/V: " + NewtonsPerVolt);

        double LeftAccelerationFeedForward = LeftAccelerationVoltage / 11.0;
        double RightAccelerationFeedForward = RightAccelerationVoltage / 11.0;

        TotalLeftFeedForward = LeftSpeedFeedForward + LeftAccelerationFeedForward;
        TotalRightFeedForward = RightSpeedFeedForward + RightAccelerationFeedForward;

        SendDriveData();
        
        double LeftSpeedRotationsPerMinute = NewLeftMetersPerSecond / WheelCircumferenceMeters / MinutesToSeconds * GearRatio;
        double RightSpeedRotationsPerMinute = NewRightMetersPerSecond / WheelCircumferenceMeters / MinutesToSeconds * GearRatio;

        // System.out.print(" TargetRightm/s: " + TargetRightMetersPerSecond);
        // System.out.print(" ActualRight : " + CurrentLeftMetersPerSecond);
        // System.out.print(" TargetLeftm/s: " + TargetLeftMetersPerSecond);
        // System.out.print(" LeftAccelerationFeedForward: " + LeftAccelerationFeedForward);
        // System.out.print(" LeftSpeedFeedForward: " + LeftSpeedFeedForward);
        // System.out.print(" RightSpeedFt/s: " + (TargetRightMetersPerSecond / FeetToMeters));
        // System.out.print(" Target Deg/ft: " + (TargetDegreesPerMeter * FeetToMeters));
        // System.out.println(" NewLeftMetersPerSecond: " + NewLeftMetersPerSecond + " RightSpeedFeedForward: " + RightSpeedFeedForward);
        System.out.print(" RightFF: " + TotalRightFeedForward);
        System.out.println(" LeftFF: " + TotalLeftFeedForward);

        LeftMaster.set(TotalLeftFeedForward);
        RightMaster.set(TotalRightFeedForward);

        // LeftController.setReference(LeftSpeedRotationsPerMinute, ControlType.kVelocity, 0, TotalLeftFeedForward * 12);
        // RightController.setReference(RightSpeedRotationsPerMinute, ControlType.kVelocity, 0, TotalRightFeedForward * 12);
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

        OSCMessage TargetVelocity = new OSCMessage();
        TargetVelocity.setAddress("/TargetVelocity");
        TargetVelocity.addArgument(this.TargetSpeedMetersPerSecond / FeetToMeters);

        // Append the left encoder data.
        CANEncoder leftEncoder = LeftMaster.getEncoder();
        OSCMessage leftVelocity = new OSCMessage();
        leftVelocity.setAddress("/LeftVelocity");
        leftVelocity.addArgument(leftEncoder.getVelocity() / FeetToMeters);

        // Append the right encoder data.
        CANEncoder rightEncoder = RightMaster.getEncoder();
        OSCMessage rightVelocity = new OSCMessage();
        rightVelocity.setAddress("/RightVelocity");
        rightVelocity.addArgument(rightEncoder.getVelocity() / FeetToMeters);

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
        bundle.addPacket(TargetVelocity);
        bundle.addPacket(leftVelocity);
        bundle.addPacket(rightVelocity);
        bundle.addPacket(Voltage);
        bundle.addPacket(fusedHeading);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the drive log data! " + ex.getMessage());
        }

    }

    public double GetLeftEncoderVelocity()
    {
        return LeftEncoder.getVelocity();
    }

    public double GetRightEncoderVelocity()
    {
        return RightEncoder.getVelocity();
    }
}
