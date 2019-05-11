package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.team254.InterpolatingDouble;
import frc.team254.InterpolatingTreeMap;

import java.net.InetAddress;

class Lift {

    public enum LiftPosition {
        ActiveGatherHatch, GatheringBall, GatheringHatch, LowBall, LowHatch, MiddleBall, MiddleHatch, HighBall,
        HighHatch, AirBrake
    }

    private TalonSRX elevatorMaster, elevatorSlaveOne, elevatorSlaveTwo, elevatorSlaveThree;
    private TalonSRX fourBarMaster, fourBarSlave;
    private Solenoid airBrake;

    private static final double FourBarSpread = 1506;
    private static final double UpperSafetyLimitThresholdDegrees = 5.0;
    private static final double LowerSafetyLimitThresholdDegrees = 5.0;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ArbFFLookup = new InterpolatingTreeMap<>();
    private double FourBarOffset;

    private double targetElevatorPosition;
    private double targetFourBarPosition;
    private boolean updated;
    private boolean initialized = false;

    private LiftPosition currentPosition;
    private boolean isAirBrake = false;
    private boolean AirBrakeActivated = false;
    private int AirBrakeCounter = 0;

    private OSCPortOut oscWirelessSender;
    private OSCPortOut oscWiredSender;

    private int HasResetEncoder = 0;
    private int ResetCounter = 0;

    private double FBPh = 1.8;
    private double FBIh = 0.0;
    private double FBDh = 6.0;
    private double FBFh = 0.8;
    private double FBVh = 200;
    private double FBAh = 500;

    private double FBPs = 0.2;
    private double FBIs = 0.0;
    private double FBDs = 4.0;
    private double FBFs = 0.8;
    private double FBVs = 200;
    private double FBAs = 500;

    /**
     * TODO: Comment.
     *
     * @param elevatorMaster
     * @param elevatorSlaveOne
     * @param elevatorSlaveTwo
     * @param elevatorSlaveThree
     * @param fourBarMaster
     * @param FourBarOffset
     */
    Lift(TalonSRX elevatorMaster, TalonSRX elevatorSlaveOne, TalonSRX elevatorSlaveTwo, TalonSRX elevatorSlaveThree,
            TalonSRX fourBarMaster, Solenoid airBrake, TalonSRX fourBarSlave, double FourBarOffset,
            boolean isPracticeRobot) {

        // Set the elevator talons.
        this.elevatorMaster = elevatorMaster;
        this.elevatorSlaveOne = elevatorSlaveOne;
        this.elevatorSlaveTwo = elevatorSlaveTwo;
        this.elevatorSlaveThree = elevatorSlaveThree;

        // Set the four bar talon and additional variables.
        this.fourBarMaster = fourBarMaster;
        this.fourBarSlave = fourBarSlave;
        this.FourBarOffset = FourBarOffset;
        this.targetFourBarPosition = FourBarOffset + FourBarSpread;
        this.updated = false;

        // Set the solenoid.
        this.airBrake = airBrake;

        // Set the air brake to inactive.
        airBrake.set(false);

        // Configure the four bar talon.
        if (isPracticeRobot) {
            fourBarMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
            fourBarMaster.setInverted(true);
            fourBarMaster.setSensorPhase(true);
            fourBarSlave.setInverted(false);
        } else {
            fourBarMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
            fourBarMaster.setInverted(false);
            fourBarMaster.setSensorPhase(false);
            fourBarSlave.setInverted(true);
        }

        // The arbitrary feed forward look up table.
        ArbFFLookup.put(new InterpolatingDouble(0.00), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.01), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.02), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.03), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.04), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.05), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.06), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.07), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.08), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.09), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.1), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.11), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.12), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.13), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.14), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.15), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.16), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.17), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.18), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.19), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.2), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.21), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.22), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.23), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.24), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.25), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.26), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.27), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.28), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.29), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.3), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.31), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.32), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.33), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.34), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.35), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.36), new InterpolatingDouble(0.00));
        ArbFFLookup.put(new InterpolatingDouble(0.37), new InterpolatingDouble(0.00));

        // The four bar PID.
        fourBarMaster.configMotionAcceleration((int) FBAh, 10);
        fourBarMaster.configMotionCruiseVelocity((int) FBVh, 10);
        fourBarMaster.config_kP(0, FBPh, 10);
        fourBarMaster.config_kI(0, FBIh, 10);
        fourBarMaster.config_kD(0, FBDh, 10);
        fourBarMaster.config_kF(0, FBFh, 10);
        fourBarMaster.config_kP(1, FBPs, 10);
        fourBarMaster.config_kI(1, FBIs, 10);
        fourBarMaster.config_kD(1, FBDs, 10);
        fourBarMaster.config_kF(1, FBFs, 10);
        fourBarMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);

        // Configure the four bar voltage compensation.
        fourBarMaster.configVoltageCompSaturation(11);
        fourBarMaster.enableVoltageCompensation(true);
        fourBarSlave.configVoltageCompSaturation(11);
        fourBarSlave.enableVoltageCompensation(true);

        // Configure the four bar current limits.
        fourBarMaster.configContinuousCurrentLimit(25);
        fourBarMaster.configPeakCurrentDuration(0);
        fourBarMaster.configPeakCurrentLimit(0);
        fourBarMaster.enableCurrentLimit(true);
        fourBarSlave.configContinuousCurrentLimit(25);
        fourBarSlave.configPeakCurrentDuration(0);
        fourBarSlave.configPeakCurrentLimit(0);
        fourBarSlave.enableCurrentLimit(true);

        // Set the soft limits on the four bar motors.
        fourBarMaster.configForwardSoftLimitThreshold(
                (int) (this.FourBarOffset + FourBarSpread - (UpperSafetyLimitThresholdDegrees / 360 * 4096)));
        fourBarMaster.configForwardSoftLimitEnable(true);
        fourBarMaster.configReverseSoftLimitThreshold(
                (int) (this.FourBarOffset + (LowerSafetyLimitThresholdDegrees / 360 * 4096)));
        fourBarMaster.configReverseSoftLimitEnable(true);

        // Enable voltage compensation on the lift motors.
        elevatorMaster.enableVoltageCompensation(true);
        elevatorSlaveOne.enableVoltageCompensation(true);
        elevatorSlaveTwo.enableVoltageCompensation(true);
        elevatorSlaveThree.enableVoltageCompensation(true);

        // Configure lift motor power.
        configElevatorMotorPower(elevatorMaster);
        configElevatorMotorPower(elevatorSlaveOne);
        configElevatorMotorPower(elevatorSlaveTwo);
        configElevatorMotorPower(elevatorSlaveThree);

        // Have lift slaves follow the master.
        elevatorSlaveOne.follow(elevatorMaster);
        elevatorSlaveTwo.follow(elevatorMaster);
        elevatorSlaveThree.follow(elevatorMaster);

        // Have the four bar slave follow the master.
        fourBarSlave.follow(fourBarMaster);

        // Invert the lift motors.
        elevatorMaster.setInverted(true);
        elevatorSlaveOne.setInverted(true);
        elevatorSlaveTwo.setInverted(true);
        elevatorSlaveThree.setInverted(true);

        // Invert the lift encoder.
        elevatorMaster.setSensorPhase(true);

        try {
            oscWirelessSender = new OSCPortOut(InetAddress.getByName("10.10.71.9"), 5803);
            oscWiredSender = new OSCPortOut(InetAddress.getByName("10.10.71.5"), 5803);
        } catch (Exception Ex) {
            System.out.println("OSC Initialization Exception: " + Ex.getMessage());
        }

    }

    /**
     * Function that configures a elevator motor's power.
     *
     * @param elevatorTalon The talon being power limited.
     */
    private static void configElevatorMotorPower(TalonSRX elevatorTalon) {
        elevatorTalon.configPeakCurrentLimit(0);
        elevatorTalon.configPeakCurrentDuration(0);
        elevatorTalon.configContinuousCurrentLimit(15);
        elevatorTalon.enableCurrentLimit(true);
        elevatorTalon.configVoltageCompSaturation(12);
        elevatorTalon.enableVoltageCompensation(true);
    }

    /**
     * Sets the lift to the desired position.
     *
     * @param position The desired position of the lift, in enumeration format.
     */
    void setLiftPosition(LiftPosition position) {

        if (position == LiftPosition.AirBrake) {
            AirBrakeActivated = false;
            isAirBrake = true;
        } else {
            isAirBrake = false;
        }

        // Four bar positions in degrees.
        double fourBarBallGatheringPositionDegrees = 105;
        double fourBarHatchGatheringPositionDegrees = 30;

        double fourBarLowBallDegrees = 105;
        double fourBarMiddleBallDegrees = 105;
        double fourBarHighBallDegrees = 105;

        double fourBarLowHatchDegrees = 40;
        double fourBarMiddleHatchDegrees = 40;
        double fourBarHighHatchDegrees = 70;

        // Elevator set positions.
        int liftGatheringPositionBall = 7500;
        int liftLowScoreBall = 3500;
        int liftMiddleScoreBall = 16500;
        int liftHighScoreBall = 29000;

        int liftGatheringPositionHatch = 0;
        int liftLowScoreHatch = 1700;
        int liftMiddleScoreHatch = 16000;
        int liftHighScoreHatch = 26000;
        int liftActiveGatherHatch = 4000;

        switch (position) {

        case ActiveGatherHatch:
            setElevatorPosition(liftActiveGatherHatch);
            setFourBarPositionDegrees(fourBarHatchGatheringPositionDegrees);
            break;

        case HighBall:
            setElevatorPosition(liftHighScoreBall);
            setFourBarPositionDegrees(fourBarHighBallDegrees);
            break;

        case HighHatch:
            setElevatorPosition(liftHighScoreHatch);
            setFourBarPositionDegrees(fourBarHighHatchDegrees);
            break;

        case MiddleBall:
            setElevatorPosition(liftMiddleScoreBall);
            setFourBarPositionDegrees(fourBarMiddleBallDegrees);
            break;

        case MiddleHatch:
            setElevatorPosition(liftMiddleScoreHatch);
            setFourBarPositionDegrees(fourBarMiddleHatchDegrees);
            break;

        case LowBall:
            setElevatorPosition(liftLowScoreBall);
            setFourBarPositionDegrees(fourBarLowBallDegrees);
            break;

        case LowHatch:
            setElevatorPosition(liftLowScoreHatch);
            setFourBarPositionDegrees(fourBarLowHatchDegrees);
            break;

        case GatheringBall:
            setElevatorPosition(liftGatheringPositionBall);
            setFourBarPositionDegrees(fourBarBallGatheringPositionDegrees);
            break;

        case AirBrake:
            setElevatorPosition(liftGatheringPositionHatch);
            setFourBarPositionDegrees(180);
            break;

        case GatheringHatch:
        default:
            setElevatorPosition(liftGatheringPositionHatch);
            setFourBarPositionDegrees(fourBarHatchGatheringPositionDegrees);
            break;

        }

    }

    /**
     * TODO: Comment.
     *
     * @param Ticks
     */
    private void setElevatorPosition(double Ticks) {
        targetElevatorPosition = Ticks;
    }

    /**
     * TODO: Comment.
     *
     * @param Degrees
     */
    private void setFourBarPositionDegrees(double Degrees) {
        double Rotations = Degrees / 360;
        double EncoderTicks = Rotations * 4096;
        targetFourBarPosition = EncoderTicks + FourBarOffset;
        updated = true;
        initialized = true;
    }

    public void LiftInit() {
        targetFourBarPosition = 0;
        updated = false;
        initialized = false;
    }

    public void LiftPeriodic() {

        // Set the four bar PID slot based on it position.
        fourBarMaster.selectProfileSlot(targetFourBarPosition > getAbsoluteFourBarTicks() ? 0 : 1, 0);

        // Set the lift to the proper position.
        elevatorMaster.set(ControlMode.MotionMagic, targetElevatorPosition);
        HasResetEncoder = 0;
        double temp = Math.abs(fourBarMaster.getSelectedSensorPosition()
                - Math.abs(fourBarMaster.getSensorCollection().getPulseWidthPosition() % 4096));
        if ((temp > 100 && temp < 3996) || temp >= 4096) {
            ResetCounter++;
            if (ResetCounter > 30) {
                ResetCounter = 0;
                System.out.println(
                        "Resetting encoder: " + fourBarMaster.getSensorCollection().getPulseWidthPosition() % 4096
                                + " : " + fourBarMaster.getSelectedSensorPosition());
                HasResetEncoder = 1;
                fourBarMaster.setSelectedSensorPosition(
                        Math.abs(fourBarMaster.getSensorCollection().getPulseWidthPosition() % 4096));
            }
        } else {
            ResetCounter--;
            if (ResetCounter < 0) {
                ResetCounter = 0;
            }
        }
    }

    /**
     * TODO: Comment.
     */
    void runLift() {

        // Set the four bar to the proper position.
        if (updated && !isFourBarFaulted() && !AirBrakeActivated) {
            updated = false;
            fourBarMaster.set(ControlMode.MotionMagic, targetFourBarPosition, DemandType.ArbitraryFeedForward,
                    getFeedForwardAmount());
        } else if (isFourBarFaulted() || !initialized || AirBrakeActivated) {
            fourBarMaster.set(ControlMode.PercentOutput, 0);
            if (isFourBarFaulted()) {
                System.out.println("FourBarFaulted");
            }
        }

        if ((isAirBrake && getFourBarDegrees() >= 134) || AirBrakeActivated) {
            if (AirBrakeCounter >= 20) {
                airBrake.set(false);
            }
            if (isAirBrake) {
                if (AirBrakeCounter >= 20) {
                    AirBrakeCounter = 20;
                    AirBrakeActivated = true;
                }
                AirBrakeCounter++;
            } else if (AirBrakeCounter > 0) {
                AirBrakeCounter--;
            }

            if (AirBrakeCounter == 0) {
                AirBrakeActivated = false;
            }

        } else {
            airBrake.set(true);
        }

        SendLiftData();

    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getAbsoluteFourBarTicks() {
        return fourBarMaster.getSensorCollection().getPulseWidthPosition();
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getRelativeFourBarTicks() {
        return fourBarMaster.getSensorCollection().getQuadraturePosition();
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getFourBarDegrees() {
        return getOffsetRelativeRotations() * 360.0;
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getFourBarCurrent() {
        return fourBarMaster.getOutputCurrent() + fourBarSlave.getOutputCurrent();
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getElevatorCurrent() {

        double totalCurrent = 0;

        totalCurrent += elevatorMaster.getOutputCurrent();
        totalCurrent += elevatorSlaveOne.getOutputCurrent();
        totalCurrent += elevatorSlaveTwo.getOutputCurrent();
        totalCurrent += elevatorSlaveThree.getOutputCurrent();

        return totalCurrent;
    }

    private double getOffsetRelative() {
        return fourBarMaster.getSelectedSensorPosition() - FourBarOffset;
    }

    double getOffsetRelativeRotations() {
        return getOffsetRelative() / 4096.0;
    }

    /**
     * Getter for the current four bar target in degrees.
     *
     * @return The four bar target in degrees.
     */
    double getTargetFourBarDegrees() {
        return (targetFourBarPosition - FourBarOffset) / 4096.0 * 360.0;
    }

    double getTargetFourBarTicks() {
        return targetFourBarPosition;
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getFeedForwardAmount() {
        return ArbFFLookup.getInterpolated(new InterpolatingDouble(getOffsetRelativeRotations())).value;
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    boolean isFourBarFaulted() {
        return fourBarMaster.getSensorCollection().getPulseWidthPosition() == 0
                || fourBarMaster.getSelectedSensorPosition() < -100 || fourBarMaster.getSelectedSensorPosition() > 4200;
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getFourBarActiveTrajectoryPosition() {
        return fourBarMaster.getActiveTrajectoryPosition();
    }

    double getFourBarMotorOutputPercent() {
        return fourBarMaster.getMotorOutputPercent();
    }

    double getFourBarClosedLoopError() {
        return fourBarMaster.getClosedLoopError();
    }

    double getElevatorVelocity() {
        return elevatorMaster.getSelectedSensorVelocity();
    }

    double getElevatorPosition() {
        return elevatorMaster.getSelectedSensorPosition();
    }

    /**
     * Sends drive information to be logged by the dashboard.
     */
    public void SendLiftData() {

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

        // Append the left encoder data.
        OSCMessage liftPosition = new OSCMessage();
        liftPosition.setAddress("/LiftPosition");
        liftPosition.addArgument((double) fourBarMaster.getSelectedSensorPosition());

        // Append the left encoder data.
        OSCMessage liftRelative = new OSCMessage();
        liftRelative.setAddress("/LiftRelative");
        liftRelative.addArgument((double) (fourBarMaster.getSensorCollection().getPulseWidthPosition() % 4096));

        // Append the right encoder data.
        OSCMessage liftVelocity = new OSCMessage();
        liftVelocity.setAddress("/LiftVelocity");
        liftVelocity.addArgument((double) fourBarMaster.getSelectedSensorVelocity());

        OSCMessage targetFBPosition = new OSCMessage();
        targetFBPosition.setAddress("/TargetFBPosition");
        targetFBPosition.addArgument(targetFourBarPosition);

        OSCMessage resetEncoder = new OSCMessage();
        resetEncoder.setAddress("/ResetEncoder");
        resetEncoder.addArgument((double) HasResetEncoder);

        OSCMessage MasterTarget = new OSCMessage();
        MasterTarget.setAddress("/MasterTargetDuty");
        MasterTarget.addArgument(fourBarMaster.getMotorOutputPercent());

        OSCMessage SlaveTarget = new OSCMessage();
        SlaveTarget.setAddress("/SlaveTargetDuty");
        SlaveTarget.addArgument(fourBarSlave.getMotorOutputPercent());

        OSCMessage PMessage = new OSCMessage();
        PMessage.setAddress("/P");
        PMessage.addArgument(FBPh);

        OSCMessage IMessage = new OSCMessage();
        IMessage.setAddress("/I");
        IMessage.addArgument(FBIh);

        OSCMessage DMessage = new OSCMessage();
        DMessage.setAddress("/D");
        DMessage.addArgument(FBDh);

        OSCMessage FMessage = new OSCMessage();
        FMessage.setAddress("/F");
        FMessage.addArgument(FBFh);

        OSCMessage AMessage = new OSCMessage();
        AMessage.setAddress("/A");
        AMessage.addArgument(FBAh);

        OSCMessage VMessage = new OSCMessage();
        VMessage.setAddress("/V");
        VMessage.addArgument(FBVh);

        OSCMessage ClosedError = new OSCMessage();
        ClosedError.setAddress("/ClosedLoopError");
        ClosedError.addArgument((double) fourBarMaster.getClosedLoopError());

        OSCMessage TrajVelocity = new OSCMessage();
        TrajVelocity.setAddress("/ActiveTrajVel");
        TrajVelocity.addArgument((double) fourBarMaster.getActiveTrajectoryVelocity());

        OSCMessage ActiveTrajPos = new OSCMessage();
        ActiveTrajPos.setAddress("/ActiveTrajPos");
        ActiveTrajPos.addArgument((double) fourBarMaster.getActiveTrajectoryPosition());

        // Add these packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(timestamp);
        bundle.addPacket(liftPosition);
        bundle.addPacket(liftRelative);
        bundle.addPacket(liftVelocity);
        bundle.addPacket(targetFBPosition);
        bundle.addPacket(resetEncoder);
        bundle.addPacket(MasterTarget);
        bundle.addPacket(SlaveTarget);
        bundle.addPacket(PMessage);
        bundle.addPacket(IMessage);
        bundle.addPacket(DMessage);
        bundle.addPacket(FMessage);
        bundle.addPacket(VMessage);
        bundle.addPacket(AMessage);
        bundle.addPacket(ClosedError);
        bundle.addPacket(TrajVelocity);
        bundle.addPacket(ActiveTrajPos);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the drive log data! " + ex.getMessage());
        }
    }

}
