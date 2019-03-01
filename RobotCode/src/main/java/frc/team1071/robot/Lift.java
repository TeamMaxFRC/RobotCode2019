package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team254.InterpolatingDouble;
import frc.team254.InterpolatingTreeMap;

class Lift {

    public enum LiftPosition {
        ActiveGatherHatch,
        GatheringBall,
        GatheringHatch,
        LowBall,
        LowHatch,
        MiddleBall,
        MiddleHatch,
        HighBall,
        HighHatch
    }

    private TalonSRX elevatorMaster, elevatorSlaveOne, elevatorSlaveTwo, elevatorSlaveThree;
    private TalonSRX fourBar;

    private static final double FourBarSpread = 1506;
    private static final double UpperSafetyLimitThresholdDegrees = 5.0;
    private static final double LowerSafetyLimitThresholdDegrees = 5.0;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ArbFFLookup = new InterpolatingTreeMap<>();
    private double FourBarOffset;

    private double targetElevatorPosition;
    private double targetFourBarPosition;
    private boolean initialized;

    /**
     * TODO: Comment.
     *
     * @param elevatorMaster
     * @param elevatorSlaveOne
     * @param elevatorSlaveTwo
     * @param elevatorSlaveThree
     * @param FourBarTalon
     * @param FourBarOffset
     */
    Lift(TalonSRX elevatorMaster, TalonSRX elevatorSlaveOne, TalonSRX elevatorSlaveTwo, TalonSRX elevatorSlaveThree, TalonSRX FourBarTalon, double FourBarOffset) {

        // Set the elevator talons.
        this.elevatorMaster = elevatorMaster;
        this.elevatorSlaveOne = elevatorSlaveOne;
        this.elevatorSlaveTwo = elevatorSlaveTwo;
        this.elevatorSlaveThree = elevatorSlaveThree;

        // Set the four bar talon and additional variables.
        this.fourBar = FourBarTalon;
        this.FourBarOffset = FourBarOffset;
        this.targetFourBarPosition = FourBarOffset + FourBarSpread;
        this.initialized = false;

        // Configure the four bar talon.
        FourBarTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        FourBarTalon.setInverted(true);
        FourBarTalon.setSensorPhase(true);

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
        FourBarTalon.configMotionAcceleration(100, 10);
        FourBarTalon.configMotionCruiseVelocity(100, 10);
        FourBarTalon.config_kP(0, 2.0, 10);
        FourBarTalon.config_kI(0, 0, 10);
        FourBarTalon.config_kD(0, 6.0, 10);
        FourBarTalon.config_kF(0, 0.8, 10);

        FourBarTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);

        // Configure the four bar voltage compensation.
        FourBarTalon.configVoltageCompSaturation(11);
        FourBarTalon.enableVoltageCompensation(true);

        // Configure the four bar current limits.
        FourBarTalon.configContinuousCurrentLimit(10);
        FourBarTalon.configPeakCurrentDuration(1000);
        FourBarTalon.configPeakCurrentLimit(20);
        FourBarTalon.enableCurrentLimit(true);

        // Set the soft limits on the four bar motors.
        FourBarTalon.configForwardSoftLimitThreshold((int) (this.FourBarOffset + FourBarSpread - (UpperSafetyLimitThresholdDegrees / 360 * 4096)));
        FourBarTalon.configForwardSoftLimitEnable(true);
        FourBarTalon.configReverseSoftLimitThreshold((int) (this.FourBarOffset + (LowerSafetyLimitThresholdDegrees / 360 * 4096)));
        FourBarTalon.configReverseSoftLimitEnable(true);


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

        // Invert the lift motors.
        elevatorMaster.setInverted(true);
        elevatorSlaveOne.setInverted(true);
        elevatorSlaveTwo.setInverted(true);
        elevatorSlaveThree.setInverted(true);

        // Invert the lift encoder.
        elevatorMaster.setSensorPhase(true);

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

        // Four bar positions in degrees.
        double fourBarBallGatheringPositionDegrees = 110;
        double fourBarHatchGatheringPositionDegrees = 30;


        double fourBarLowBallDegrees = 110;
        double fourBarMiddleBallDegrees = 110;
        double fourBarHighBallDegrees = 110;

        double fourBarLowHatchDegrees = 50;
        double fourBarMiddleHatchDegrees = 40;
        double fourBarHighHatchDegrees = 50;

        // Elevator set positions.
        int liftGatheringPositionBall = 7500;
        int liftLowScoreBall = 0;
        int liftMiddleScoreBall = 10000;
        int liftHighScoreBall = 25500;

        int liftGatheringPositionHatch = 0;
        int liftLowScoreHatch = 0;
        int liftMiddleScoreHatch = 15000;
        int liftHighScoreHatch = 25500;
        int liftActiveGatherHatch = 2000;

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
        initialized = true;
    }

    /**
     * TODO: Comment.
     */
    void runLift() {

        // Set the four bar to the proper position.
        if (initialized && !isFourBarFaulted()) {
            fourBar.set(ControlMode.MotionMagic, targetFourBarPosition, DemandType.ArbitraryFeedForward, getFeedForwardAmount());
        } else {
            fourBar.set(ControlMode.PercentOutput, 0);
        }

        // Set the lift to the proper position.
        elevatorMaster.set(ControlMode.MotionMagic, targetElevatorPosition);
        if (fourBar.getSelectedSensorPosition() % 4096 != fourBar.getSelectedSensorPosition())
        {
            fourBar.setSelectedSensorPosition(fourBar.getSelectedSensorPosition() % 4096);
        }
        //FourBarTalon.setSelectedSensorPosition(FourBarTalon.getSensorCollection().getPulseWidthPosition() % 4096);

    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getAbsoluteFourBarTicks() {
        return fourBar.getSensorCollection().getPulseWidthPosition();
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getRelativeFourBarTicks() {
        return fourBar.getSensorCollection().getQuadraturePosition();
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
        return fourBar.getOutputCurrent();
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
        return fourBar.getSensorCollection().getQuadraturePosition() - FourBarOffset;
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
        return fourBar.getSensorCollection().getPulseWidthPosition() == 0;
    }

    /**
     * TODO: Comment.
     *
     * @return
     */
    double getFourBarActiveTrajectoryPosition() {
        return fourBar.getActiveTrajectoryPosition();
    }

    double getFourBarMotorOutputPercent() {
        return fourBar.getMotorOutputPercent();
    }

    double getFourBarClosedLoopError() {
        return fourBar.getClosedLoopError();
    }

    double getElevatorVelocity() {
        return elevatorMaster.getSelectedSensorVelocity();
    }

    double getElevatorPosition() {
        return elevatorMaster.getSelectedSensorPosition();
    }

}

