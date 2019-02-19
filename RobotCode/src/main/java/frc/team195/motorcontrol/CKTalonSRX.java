package frc.team195.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team195.reporters.ConsoleReporter;
import frc.team195.reporters.DiagnosticMessage;
import frc.team195.reporters.MessageLevel;
import frc.team195.utils.ThreadRateControl;
import frc.team254.InterpolatingDouble;
import frc.team254.InterpolatingTreeMap;

public class CKTalonSRX extends TalonSRX implements TuneableMotorController {
	private int currentSelectedSlot = 0;
	private double[] mCLRampRate = new double[4];
	private int[] mMMAccel = new int[4];
	private int[] mMMVel = new int[4];
	private double prevOutput = Double.MIN_VALUE;
	private final PDPBreaker motorBreaker;

	private final Configuration fastMasterConfig = new Configuration(5, 5, 20);
	private final Configuration normalMasterConfig = new Configuration(10, 10, 20);
	private final Configuration normalSlaveConfig = new Configuration(10, 100, 100);

	private double prevMotionVelocitySetpoint = 0;
	private double minSetpointOutput = 0;
	private double allowedClosedLoopError = 0;
	private MCControlMode currentControlMode = MCControlMode.Disabled;  //Force an update

	private Thread motionVoodooArbFFControlThread;
	private double motionVoodooArbFFDemand = 0;
	public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> motionVoodooArbFFLookup = new InterpolatingTreeMap<>();
	public double absoluteEncoderOffset = 0;

	private double speedNew = 0;

	public CKTalonSRX(int deviceId, boolean fastMaster, PDPBreaker breakerCurrent) {
		super(deviceId);
		motorBreaker = breakerCurrent;
		doDefaultConfig(fastMaster ? fastMasterConfig : normalMasterConfig);
		System.out.println("\n\n\n\nSomething\n\n\n\n");
	}

	public CKTalonSRX(int deviceId, TalonSRX masterTalon, PDPBreaker breakerCurrent) {
		super(deviceId);
		motorBreaker = breakerCurrent;
		doDefaultConfig(normalSlaveConfig);
		follow(masterTalon);
	}

	private void doDefaultConfig(Configuration config) {
		boolean setSucceeded;
		int retryCounter = 0;
		do {
			setSucceeded = clearStickyFaults(Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS) == ErrorCode.OK;
			setSucceeded &= setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configContinuousCurrentLimit(motorBreaker.value, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configPeakCurrentLimit(motorBreaker.value * 2, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configPeakCurrentDuration(getMSDurationForBreakerLimit(motorBreaker.value * 2, motorBreaker.value), Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			enableCurrentLimit(true);
			setSucceeded &= configVoltageCompSaturation(12) == ErrorCode.OK;
			setSucceeded &= configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs) == ErrorCode.OK;
			set(MCControlMode.PercentOut, 0, 0, 0);
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
		return configClosedloopRamp(secondsFromNeutralToFull, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int slotIdx, int timeoutMs) {
		setCurrentSlotCLRampRate(secondsFromNeutralToFull, slotIdx);
		return super.configClosedloopRamp(secondsFromNeutralToFull, timeoutMs);
	}

	@Override
	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) {
		return configMotionAcceleration(sensorUnitsPer100msPerSec, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int slotIdx, int timeoutMs) {
		setCurrentMMAccel(sensorUnitsPer100msPerSec, slotIdx);
		return super.configMotionAcceleration(sensorUnitsPer100msPerSec, timeoutMs);
	}

	@Override
	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs) {
		return configMotionCruiseVelocity(sensorUnitsPer100ms, currentSelectedSlot, timeoutMs);
	}

	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int slotIdx, int timeoutMs) {
		setCurrentMMVel(sensorUnitsPer100ms, slotIdx);
		return super.configMotionCruiseVelocity(sensorUnitsPer100ms, timeoutMs);
	}

	public synchronized void setAbsoluteEncoderOffset(double offset) {
		absoluteEncoderOffset = offset;
	}

	/**
	 * Make sure you call this method before first use of set() to ensure CL ramp rate and PID gains are selected properly when using CKTalonSRX
	 * @param slotIdx Gain profile slot
	 * @param pidIdx PID ID, 0 for main, 1 for aux
	 */
	@Override
	public void selectProfileSlot(int slotIdx, int pidIdx) {
		super.selectProfileSlot(slotIdx, pidIdx);
		setCurrentSlotValue(slotIdx);
		if (currentSelectedSlot < mCLRampRate.length && currentSelectedSlot < mMMAccel.length && currentSelectedSlot < mMMVel.length) {
			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = configClosedloopRamp(mCLRampRate[currentSelectedSlot], currentSelectedSlot, Constants.kCANTimeoutMs) == ErrorCode.OK;
				setSucceeded &= configMotionAcceleration(mMMAccel[currentSelectedSlot], currentSelectedSlot, Constants.kCANTimeoutMs) == ErrorCode.OK;
				setSucceeded &= configMotionCruiseVelocity(mMMVel[currentSelectedSlot], currentSelectedSlot, Constants.kCANTimeoutMs) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to change Talon ID" + getDeviceID() +  " profile slot!!!", MessageLevel.DEFCON1);
		}
	}

	private synchronized void setCurrentSlotValue(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	private synchronized void setCurrentSlotCLRampRate(double rampRate, int slot) {
		if (slot < mCLRampRate.length) {
			mCLRampRate[slot] = rampRate;
		}
	}

	private synchronized void setCurrentMMVel(int vel, int slot) {
		if (slot < mMMVel.length) {
			mMMVel[slot] = vel;
		}
	}

	private synchronized void setCurrentMMAccel(int accel, int slot) {
		if (slot < mMMAccel.length) {
			mMMAccel[slot] = accel;
		}
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("General Status Frame 1: " + getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 2: " + getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 3: " + getStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 4: " + getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 6: " + getStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 7: " + getStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 8: " + getStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 9: " + getStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 10: " + getStatusFramePeriod(StatusFrame.Status_10_Targets, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 11: " + getStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 12: " + getStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 13: " + getStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 14: " + getStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, Constants.kLongCANTimeoutMs) + "\r\n");
		return sb.toString();
	}

	@Deprecated
	public void set(ControlMode mode, double outputValue) {
		set(MCControlMode.valueOf(mode), outputValue, currentSelectedSlot, 0);
	}

	@Deprecated
	public void set(ControlMode mode, double demand0, double demand1) {
		set(MCControlMode.valueOf(mode), demand0, currentSelectedSlot, demand1);
	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode != currentControlMode) {
			currentControlMode = controlMode;

			if (controlMode == MCControlMode.MotionVoodooArbFF) {
				motionVoodooArbFFDemand = demand;
				startMotionVoodooArbFFControlThread();
			} else {
				demand = convertDemandToNativeUnits(controlMode, demand);
				set(controlMode.CTRE(), demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
			}
			prevOutput = demand + arbitraryFeedForward;
		}
	}

	private void startMotionVoodooArbFFControlThread() {
		if (motionVoodooArbFFControlThread != null) {
			if (!motionVoodooArbFFControlThread.isAlive()) {
				resetMotionVoodooArbFFLambda();
				motionVoodooArbFFControlThread.start();
			}
		} else {
			resetMotionVoodooArbFFLambda();
			motionVoodooArbFFControlThread.start();
		}
	}

	private void resetMotionVoodooArbFFLambda() {
		//Instantiate Motion Magic Thread
		motionVoodooArbFFControlThread = new Thread(() -> {
			ThreadRateControl trc = new ThreadRateControl();
			trc.start();
			while (currentControlMode == MCControlMode.MotionVoodooArbFF) {
				double demandStep = generateMotionVoodooArbFFValue(getPosition(), motionVoodooArbFFDemand, getVelocity(), trc.getDt());
				prevMotionVelocitySetpoint = demandStep;
				demandStep = Math.abs(demandStep) < minSetpointOutput ? 0 : demandStep;
				double arbFF = motionVoodooArbFFLookup.getInterpolated(new InterpolatingDouble(getPosition() + absoluteEncoderOffset)).value;
				System.out.println("Used for ARBFF: " + (getPosition() + absoluteEncoderOffset) + " Value: " + arbFF);
				set(ControlMode.Velocity, convertDemandToNativeUnits(MCControlMode.MotionVoodooArbFF, demandStep), DemandType.ArbitraryFeedForward, arbFF);
//				System.out.println("ArbFF: " + arbFF + ", OutputDC: " + getMCOutputPercent() + ", Set: " + motionVoodooArbFFDemand + ", Pos: " + (getPosition()) + ", Spd: " + demandStep);
				trc.doRateControl(10);
			}
		});
	}

	private double generateMotionVoodooArbFFValue(double position, double setpoint, double speed, double dt) {
		double motionVoodooArbFFVelocity = convertNativeUnitsToRPM(mMMVel[currentSelectedSlot]);
		double motionVoodooArbFFMaxAccel = convertNativeUnitsToRPM(mMMAccel[currentSelectedSlot]);

		double diffErr = setpoint - position;
		double diffSign = Math.copySign(1.0, diffErr);
		diffErr = Math.abs(diffErr);

		if (motionVoodooArbFFVelocity == 0 || motionVoodooArbFFMaxAccel == 0 || diffErr <= allowedClosedLoopError)
			return 0;

		double decelVelocity = Math.sqrt(diffErr * 2.0 / (motionVoodooArbFFMaxAccel * 60.0)) * (motionVoodooArbFFVelocity * 60.0);

		speedNew = prevMotionVelocitySetpoint + (motionVoodooArbFFMaxAccel * dt * diffSign);
		double absoluteSpeed = Math.abs(speedNew);
		speedNew = absoluteSpeed > motionVoodooArbFFVelocity ? Math.copySign(motionVoodooArbFFVelocity, speedNew) : speedNew;

		speedNew = absoluteSpeed <= decelVelocity ? speedNew : Math.copySign(decelVelocity, speedNew);
		speedNew = absoluteSpeed <= minSetpointOutput ? 0 : speedNew;
		return speedNew;
	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = config_kP(currentSelectedSlot, kP, Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= config_kI(currentSelectedSlot, kI, Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= config_kD(currentSelectedSlot, kD, Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= config_kF(currentSelectedSlot, kF, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set PID Gains Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setDFilter(double dFilter) {

	}

	@Override
	public void setIZone(double iZone) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = config_IntegralZone(currentSelectedSlot, (int)iZone, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set IZone Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCIAccum(double iAccum) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = setIntegralAccumulator(iAccum, 0, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set I Accum Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMaxIAccum(double maxIAccum) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = configMaxIntegralAccumulator(currentSelectedSlot, maxIAccum, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Max I Accum Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = configOpenloopRamp(rampRate, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set MC Closed Loop Ramp Rate Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCClosedLoopRampRate(double rampRate) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = configClosedloopRamp(rampRate, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set MC Closed Loop Ramp Rate Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMotionParameters(int cruiseVel, int cruiseAccel) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = configMotionCruiseVelocity(convertRPMToNativeUnits(cruiseVel), Constants.kCANTimeoutMs) == ErrorCode.OK;
			setSucceeded &= configMotionAcceleration(convertRPMToNativeUnits(cruiseAccel), Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Motion Params Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);
	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {
		setNeutralMode(neutralMode.CTRE());
	}

	@Override
	public void setEncoderPosition(double position) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = setSelectedSensorPosition(convertRotationsToNativeUnits(position), 0, Constants.kCANTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set encoder position Talon " + getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setControlMode(MCControlMode controlMode) {
		if (getMotionControlMode() != controlMode) {
			switch (controlMode) {
				case PercentOut:
				case Velocity:
				case Current:
					set(controlMode, 0, currentSelectedSlot, 0);
					break;
				case Position:
				case MotionMagic:
				case MotionVoodooArbFF:
					set(controlMode, getPosition(), currentSelectedSlot, 0);
					break;
				default:
					break;
			}
		}
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return currentControlMode;
	}

	@Override
	public void setSetpoint(double setpoint) {
		set(getMotionControlMode(), setpoint, currentSelectedSlot, 0);
	}

	@Override
	public double getActual() {
		switch (currentControlMode) {
			case PercentOut:
//				return getMotorOutputPercent();
			case Position:
			case MotionMagic:
			case MotionVoodooArbFF:
				return getPosition();
			case Velocity:
				return getVelocity();
			case Current:
				return getOutputCurrent();
			default:
				return 0;
		}
	}

	@Override
	public double getSetpoint() {
		return 0;
	}

	@Override
	public double getMCOutputCurrent() {
		return getOutputCurrent();
	}

	@Override
	public double getMCOutputPercent() {
		return getMotorOutputPercent();
	}

	@Override
	public int getMCID() {
		return getDeviceID();
	}

	@Override
	public boolean isEncoderPresent() {
		return getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
	}

	@Override
	public double getMCInputVoltage() {
		return getBusVoltage();
	}

	@Override
	public double getMCOutputVoltage() {
		return getMotorOutputVoltage();
	}

	@Override
	public double getPosition() {
		return convertNativeUnitsToRotations(getSelectedSensorPosition());
	}

	@Override
	public double getVelocity() {
		return convertNativeUnitsToRPM(getSelectedSensorVelocity());
	}

	@Override
	public double getSensorUnitsPerRotation() {
		return 4096;
	}

	@Override
	public double getVelocityRPMTimeConversionFactor() {
		return 600;
	}

	@Override
	public double getNativeUnitsOutputRange() {
		return 1023.0;
	}

	@Override
	public double getMCIAccum() {
		return getIntegralAccumulator();
	}

	@Override
	public DiagnosticMessage hasMotorControllerReset() {
		if (hasResetOccurred()) {

			ConsoleReporter.report("Talon ID " + getDeviceID() + " has reset!", MessageLevel.DEFCON1);

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = clearStickyFaults(Constants.kCANTimeoutMs) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Talon ID " + getDeviceID() + " Reset !!!!!!", MessageLevel.DEFCON1);

			return new DiagnosticMessage("Talon" + getDeviceID() + "ResetHasOccurred");
		}

		return DiagnosticMessage.NO_MSG;
	}

	private static class Configuration {
		int CONTROL_FRAME_PERIOD_MS;
		int STATUS_FRAME_GENERAL_1_MS;
		int STATUS_FRAME_FEEDBACK0_2_MS;

		Configuration(int CONTROL_FRAME_PERIOD_MS, int STATUS_FRAME_GENERAL_1_MS, int STATUS_FRAME_FEEDBACK0_2_MS) {
			this.CONTROL_FRAME_PERIOD_MS = CONTROL_FRAME_PERIOD_MS;
			this.STATUS_FRAME_GENERAL_1_MS = STATUS_FRAME_GENERAL_1_MS;
			this.STATUS_FRAME_FEEDBACK0_2_MS = STATUS_FRAME_FEEDBACK0_2_MS;
		}
	}
}
