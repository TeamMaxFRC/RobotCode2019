package frc.team1071.robot;

import java.net.InetAddress;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;

import edu.wpi.first.wpilibj.Timer;
import frc.team254.InterpolatingDouble;
import frc.team254.InterpolatingTreeMap;

public class FourBar {

    private static final double FourBarSpread = 1506;
    private static final double SafetyLimitThresholdDegrees = 5.0;

    private TalonSRX FourBarTalon;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ArbFFLookup = new InterpolatingTreeMap<>();
    private double FourBarOffset;

    private OSCPortOut oscWirelessSender;
    private OSCPortOut oscWiredSender;

    private double TargetEncoderPosition;
    private boolean initialized;

    public double GetOffsetAbsolute() {
        return FourBarTalon.getSensorCollection().getPulseWidthPosition() - FourBarOffset;
    }

    public double GetOffsetRelative() {
        return FourBarTalon.getSensorCollection().getQuadraturePosition() - FourBarOffset;
    }

    public double GetOffsetRelativeRotations() {
        return GetOffsetRelative() / 4096.0;
    }

    public double GetDegrees() {
        return GetOffsetRelativeRotations() * 360.0;
    }

    public double GetTargetDegrees() {
        return (this.TargetEncoderPosition - FourBarOffset) / 4096.0 * 360.0;
    }

    public void SetPositionDegrees(double Degrees) {
        double Rotations = Degrees / 360;
        double EncoderTicks = Rotations * 4096;
        this.TargetEncoderPosition = EncoderTicks + FourBarOffset;
        initialized = true;

    }

    public double GetFeedForwardAmount() {
        return ArbFFLookup.getInterpolated(new InterpolatingDouble(GetOffsetRelativeRotations())).value;
    }

    public FourBar(TalonSRX FourBarTalon, double FourBarOffset) {

        this.FourBarTalon = FourBarTalon;
        this.FourBarOffset = FourBarOffset;
        this.TargetEncoderPosition = FourBarOffset + FourBarSpread;
        this.initialized = false;

        FourBarTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        FourBarTalon.setInverted(false);
        FourBarTalon.setSensorPhase(false);
        FourBarTalon.setSelectedSensorPosition(FourBarTalon.getSensorCollection().getPulseWidthPosition());

        ArbFFLookup.put(new InterpolatingDouble(0.00), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.01), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.02), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.03), new InterpolatingDouble(0.15));
        ArbFFLookup.put(new InterpolatingDouble(0.04), new InterpolatingDouble(0.17));
        ArbFFLookup.put(new InterpolatingDouble(0.05), new InterpolatingDouble(0.17));
        ArbFFLookup.put(new InterpolatingDouble(0.06), new InterpolatingDouble(0.18));
        ArbFFLookup.put(new InterpolatingDouble(0.07), new InterpolatingDouble(0.19));
        ArbFFLookup.put(new InterpolatingDouble(0.08), new InterpolatingDouble(0.11));
        ArbFFLookup.put(new InterpolatingDouble(0.09), new InterpolatingDouble(0.16));
        ArbFFLookup.put(new InterpolatingDouble(0.1), new InterpolatingDouble(0.17));
        ArbFFLookup.put(new InterpolatingDouble(0.11), new InterpolatingDouble(0.17));
        ArbFFLookup.put(new InterpolatingDouble(0.12), new InterpolatingDouble(0.16));
        ArbFFLookup.put(new InterpolatingDouble(0.13), new InterpolatingDouble(0.14));
        ArbFFLookup.put(new InterpolatingDouble(0.14), new InterpolatingDouble(0.14));
        ArbFFLookup.put(new InterpolatingDouble(0.15), new InterpolatingDouble(0.14));
        ArbFFLookup.put(new InterpolatingDouble(0.16), new InterpolatingDouble(0.215));
        ArbFFLookup.put(new InterpolatingDouble(0.17), new InterpolatingDouble(0.20));
        ArbFFLookup.put(new InterpolatingDouble(0.18), new InterpolatingDouble(0.20));
        ArbFFLookup.put(new InterpolatingDouble(0.19), new InterpolatingDouble(0.20));
        ArbFFLookup.put(new InterpolatingDouble(0.2), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.21), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.22), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.23), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.24), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.25), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.26), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.27), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.28), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.29), new InterpolatingDouble(0.12));
        ArbFFLookup.put(new InterpolatingDouble(0.3), new InterpolatingDouble(0.1));
        ArbFFLookup.put(new InterpolatingDouble(0.31), new InterpolatingDouble(0.05));
        ArbFFLookup.put(new InterpolatingDouble(0.32), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.33), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.34), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.35), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.36), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.37), new InterpolatingDouble(0.00));

        try {
            oscWirelessSender = new OSCPortOut(InetAddress.getByName("10.10.71.9"), 5803);
            oscWiredSender = new OSCPortOut(InetAddress.getByName("10.10.71.5"), 5803);
        } catch (Exception Ex) {
            System.out.println("OSC Initialization Exception: " + Ex.getMessage());
        }

        FourBarTalon.configMotionAcceleration(2000, 10);
        FourBarTalon.configMotionCruiseVelocity(1000, 10);
        FourBarTalon.config_kP(0, 1, 10);
        FourBarTalon.config_kI(0, 0, 10);
        FourBarTalon.config_kD(0, 1.5, 10);
        FourBarTalon.config_kF(0, 0.8, 10);

        FourBarTalon.configVoltageCompSaturation(11);
        FourBarTalon.enableVoltageCompensation(true);

        FourBarTalon.configContinuousCurrentLimit(25);
        FourBarTalon.configPeakCurrentDuration(0);
        FourBarTalon.configPeakCurrentLimit(0);
        FourBarTalon.enableCurrentLimit(true);

        FourBarTalon.configForwardSoftLimitThreshold((int) (this.FourBarOffset + FourBarSpread - (SafetyLimitThresholdDegrees / 360 * 4096)));
        FourBarTalon.configForwardSoftLimitEnable(true);
        FourBarTalon.configReverseSoftLimitThreshold((int) (this.FourBarOffset + (SafetyLimitThresholdDegrees / 360 * 4096)));
        FourBarTalon.configReverseSoftLimitEnable(true);

    }


    /**
     * Sends drive information to be logged by the dashboard.
     */
    public void SendFourBarData() {

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("FourBarLog");

        // Append the robot timestamp for the data.
        OSCMessage timestamp = new OSCMessage();
        timestamp.setAddress("/timestamp");
        timestamp.addArgument(Timer.getFPGATimestamp());

        OSCMessage AbsolutePosition = new OSCMessage();
        AbsolutePosition.setAddress("/AbsolutePosition");
        AbsolutePosition.addArgument((double) FourBarTalon.getSensorCollection().getPulseWidthPosition());

        OSCMessage QuadraturePosition = new OSCMessage();
        QuadraturePosition.setAddress("/QuadraturePosition");
        QuadraturePosition.addArgument((double) FourBarTalon.getSensorCollection().getQuadraturePosition());

        OSCMessage ActualDegrees = new OSCMessage();
        ActualDegrees.setAddress("/ActualDegrees");
        ActualDegrees.addArgument(GetDegrees());

        OSCMessage ActualRotations = new OSCMessage();
        ActualRotations.setAddress("/ActualRotations");
        ActualRotations.addArgument(GetOffsetRelativeRotations());

        OSCMessage TargetDegrees = new OSCMessage();
        TargetDegrees.setAddress("/TargetDegrees");
        TargetDegrees.addArgument(GetTargetDegrees());

        OSCMessage TargetNative = new OSCMessage();
        TargetNative.setAddress("/TargetNative");
        TargetNative.addArgument(TargetEncoderPosition);

        OSCMessage ActiveTrajectoryPosition = new OSCMessage();
        ActiveTrajectoryPosition.setAddress("/ActiveTrajectoryPosition");
        ActiveTrajectoryPosition.addArgument((double) FourBarTalon.getActiveTrajectoryPosition());

        OSCMessage ClosedLoopError = new OSCMessage();
        ClosedLoopError.setAddress("/ClosedLoopError");
        ClosedLoopError.addArgument((double) FourBarTalon.getClosedLoopError());

        OSCMessage ArbFF = new OSCMessage();
        ArbFF.setAddress("/ArbFF");
        ArbFF.addArgument(GetFeedForwardAmount());

        OSCMessage MotorOut = new OSCMessage();
        MotorOut.setAddress("/MotorOut");
        MotorOut.addArgument(FourBarTalon.getMotorOutputPercent());

        // Add these packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(timestamp);
        bundle.addPacket(AbsolutePosition);
        bundle.addPacket(QuadraturePosition);
        bundle.addPacket(ActualDegrees);
        bundle.addPacket(ActualRotations);
        bundle.addPacket(TargetDegrees);
        bundle.addPacket(TargetNative);
        bundle.addPacket(ActiveTrajectoryPosition);
        bundle.addPacket(ClosedLoopError);
        bundle.addPacket(ArbFF);
        bundle.addPacket(MotorOut);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the drive log data! " + ex.getMessage());
        }
    }

    public void TeleopInit() {
        initialized = true;
    }

    public void RobotPeriodic() {

    }

    public boolean isFaulted() {
        return FourBarTalon.getSensorCollection().getPulseWidthPosition() == 0;
    }

    public void RunFourBar() {

        SendFourBarData();

        if (initialized && !isFaulted()) {
            FourBarTalon.set(ControlMode.MotionMagic, TargetEncoderPosition, DemandType.ArbitraryFeedForward, GetFeedForwardAmount());
        } else {
            FourBarTalon.set(ControlMode.PercentOutput, 0);
        }
    }
}
