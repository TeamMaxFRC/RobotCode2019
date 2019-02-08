package frc.team195.motorcontrol;

import com.revrobotics.*;
import frc.team195.reporters.ConsoleReporter;
import frc.team195.reporters.DiagnosticMessage;
import frc.team195.reporters.MessageLevel;

import java.util.Optional;

public class CKSparkMax extends CANSparkMax implements TuneableMotorController {
	private int currentSelectedSlot = 0;
	public final CANPIDController canPIDController;
	public final CANEncoder canEncoder;
	private double prevOutput = Double.MIN_VALUE;
	private double setpoint = 0;

//	private Configuration fastMasterConfig = new Configuration(5, 5, 20, 50);
//	private Configuration normalMasterConfig = new Configuration(10, 10, 20, 50);
//	private Configuration normalSlaveConfig = new Configuration(10, 100, 100, 100);

	private Configuration fastMasterConfig = new Configuration(5, 5, 10, 10);
	private Configuration normalMasterConfig = new Configuration(10, 10, 10, 10);
	private Configuration normalSlaveConfig = new Configuration(10, 100, 100, 100);

	private double voltageCompensation = 12;
	private final PDPBreaker motorBreaker;

	private MCControlMode currentControlMode = MCControlMode.PercentOut;

	public CKSparkMax(int deviceID, MotorType type, boolean fastMaster, PDPBreaker breakerCurrent) {
		super(deviceID, type);
		motorBreaker = breakerCurrent;
		canPIDController = getPIDController();
		canEncoder = getEncoder();
		canPIDController.setOutputRange(-1, 1);
		doDefaultConfig(fastMaster ? fastMasterConfig : normalMasterConfig);
		setMinimumSetpointOutput(25);
		setBrakeCoastMode(MCNeutralMode.Brake);
	}

	public CKSparkMax(int deviceID, MotorType type, CANSparkMax masterSpark, PDPBreaker breakerCurrent) {
		super(deviceID, type);
		motorBreaker = breakerCurrent;
		canPIDController = getPIDController();
		canEncoder = getEncoder();
		doDefaultConfig(normalSlaveConfig);
		follow(masterSpark);
		setBrakeCoastMode(MCNeutralMode.valueOf(masterSpark.getIdleMode()));
	}

	private void doDefaultConfig(Configuration config) {
		//Fix encoder transient 0s which cause issues with all kinds of motion code
		setCANTimeout(500);

//		setControlFramePeriod(config.CONTROL_FRAME_PERIOD_MS);
		boolean setSucceeded;
		int retryCounter = 0;
		do {
			setSucceeded = setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS_FRAME_0_MS) == CANError.kOK;
			setSucceeded &= setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS_FRAME_1_MS) == CANError.kOK;
			setSucceeded &= setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS_FRAME_2_MS) == CANError.kOK;
			setSucceeded &= setSmartCurrentLimit(motorBreaker.value * 2) == CANError.kOK;
			//Erase previously stored values
			set(MCControlMode.PercentOut, 0, 0, 0);
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to config Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Deprecated
	public void set(double DO_NOT_USE) {

	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			setPIDGainSlot(slotIdx);

		//TODO: Remove if not necessary after testing position units
		demand = convertDemandToNativeUnits(controlMode, demand);

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode != currentControlMode) {
			currentControlMode = controlMode;
			setpoint = demand;

			boolean setSucceeded;
			int retryCounter = 1;

			do {
				System.out.println(getPosition());
				setSucceeded = canPIDController.setReference(demand, controlMode.Rev(), slotIdx, arbitraryFeedForward * voltageCompensation) == CANError.kOK;
			} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to set output Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);

			prevOutput = demand + arbitraryFeedForward;
		}
	}

	public synchronized void setMinimumSetpointOutput(double minSetpointOutput) {
		canPIDController.setSmartMotionMinOutputVelocity(minSetpointOutput, currentSelectedSlot);
	}

	public synchronized void setAllowedClosedLoopError(double allowedClosedLoopError) {
		canPIDController.setSmartMotionAllowedClosedLoopError(allowedClosedLoopError, currentSelectedSlot);
	}

	public double getPosition() {
		return canEncoder.getPosition();
	}

	public double getVelocity() {
		return canEncoder.getVelocity();
	}

	@Override
	public double getSensorUnitsPerRotation() {
//		return 42;
		//TODO: Change this after testing what position units are in for Rev
		return 1;
	}

	@Override
	public double getSetpoint() {
		return setpoint;
	}

	@Override
	public double getVelocityRPMTimeConversionFactor() {
		return 1;
	}

	@Override
	public double getNativeUnitsOutputRange() {
		return 1.0;
	}


	/**
	 * Config voltage compensation for arbitrary feed forward
	 * @param voltage Nominal voltage setting for arbitrary feed forward compensation
	 */
	public synchronized void configVoltageCompSaturation(double voltage) {
		voltageCompensation = voltage;
	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = canPIDController.setP(kP, currentSelectedSlot) == CANError.kOK;
			setSucceeded &= canPIDController.setI(kI, currentSelectedSlot) == CANError.kOK;
			setSucceeded &= canPIDController.setD(kD, currentSelectedSlot) == CANError.kOK;
			setSucceeded &= canPIDController.setFF(kF, currentSelectedSlot) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set PID Gains Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setDFilter(double dFilter) {
		canPIDController.setDFilter(dFilter, currentSelectedSlot);
	}

	@Override
	public void setIZone(double iZone) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = canPIDController.setIZone(iZone, currentSelectedSlot) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set IZone Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCIAccum(double iAccum) {
		canPIDController.setIAccum(iAccum);
	}

	@Override
	public void setMaxIAccum(double maxIAccum) {
		canPIDController.setIMaxAccum(maxIAccum, currentSelectedSlot);
	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = setOpenLoopRampRate(rampRate) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Ramp rate Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void setMCClosedLoopRampRate(double rampRate) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = setClosedLoopRampRate(rampRate) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Ramp rate Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public synchronized void setMotionParameters(int cruiseVel, int cruiseAccel) {
		canPIDController.setSmartMotionMaxVelocity(cruiseVel, currentSelectedSlot);
		canPIDController.setSmartMotionMaxAccel(cruiseAccel, currentSelectedSlot);
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		setCurrentSelectedSlot(slotIdx);
	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = setIdleMode(neutralMode.Rev()) == CANError.kOK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set Idle Mode Spark Max " + getDeviceId() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public synchronized void setEncoderPosition(double position) {
		canEncoder.setPosition(position);
	}

	private synchronized void setCurrentSelectedSlot(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	@Override
	public synchronized void setControlMode(MCControlMode controlMode) {
		if (currentControlMode != controlMode)
		{
			currentControlMode = controlMode;
			switch (controlMode) {
				case Position:
				case MotionMagic:
					set(controlMode, canEncoder.getPosition(), currentSelectedSlot, 0);
					break;
				case PercentOut:
				case Velocity:
				case Voltage:
				default:
					set(controlMode, 0, currentSelectedSlot, 0);
					break;
			}
		}
	}

	@Override
	public void setSetpoint(double setpoint) {
		set(getMotionControlMode(), setpoint, currentSelectedSlot, 0);
	}

	@Override
	public double getActual() {
		switch (getControlType()) {
			case kDutyCycle:
				return getAppliedOutput();
			case kVelocity:
				return canEncoder.getVelocity();
			case kVoltage:
				return getAppliedOutput() * voltageCompensation;
			case kPosition:
			case kSmartMotion:
				return canEncoder.getPosition();
			default:
				return 0;
		}
	}

	@Override
	public double getMCIAccum() {
		return canPIDController.getIAccum();
	}

	@Override
	public double getMCOutputCurrent() {
		return getOutputCurrent();
	}

	@Override
	public double getMCOutputPercent() {
		return getAppliedOutput();
	}

	@Override
	public int getMCID() {
		return getDeviceId();
	}

	@Override
	public double getMCInputVoltage() {
		return getBusVoltage();
	}

	@Override
	public double getMCOutputVoltage() {
		return getMCOutputPercent()*getMCInputVoltage();
	}

	@Override
	public boolean isEncoderPresent() {
		//TODO: Test
		return !getFault(FaultID.kSensorFault);
	}

	@Override
	public DiagnosticMessage hasMotorControllerReset() {
		//TODO: Verify this works in SparkMax API
		if (getFault(FaultID.kHasReset)) {
			ConsoleReporter.report("Spark Max ID " + getDeviceId() + " has reset!", MessageLevel.DEFCON1);

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = clearFaults() == CANError.kOK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Spark Max ID " + getDeviceId() + " Reset !!!!!!", MessageLevel.DEFCON1);

			return new DiagnosticMessage("SparkMax" + getDeviceId() + "ResetHasOccurred");
		}
		return DiagnosticMessage.NO_MSG;
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return currentControlMode;
	}

	public ControlType getControlType() {
		Optional<Integer> o = getParameterInt(ConfigParameter.kCtrlType);
		if (!o.isPresent())
			return ControlType.kDutyCycle;
		return controlTypeFromInt(o.get());
	}

	private ControlType controlTypeFromInt(int ctrlMode) {
		switch (ctrlMode) {
			case 0:
				return ControlType.kDutyCycle;
			case 1:
				return ControlType.kVelocity;
			case 2:
				return ControlType.kVoltage;
			case 3:
				return ControlType.kPosition;
			case 4:
				return ControlType.kSmartMotion;
			default:
				return ControlType.kDutyCycle;
		}
	}

	private static class Configuration {
		int CONTROL_FRAME_PERIOD_MS;
		int STATUS_FRAME_0_MS;
		int STATUS_FRAME_1_MS;
		int STATUS_FRAME_2_MS;

		Configuration(int CONTROL_FRAME_PERIOD_MS, int STATUS_FRAME_0_MS, int STATUS_FRAME_1_MS, int STATUS_FRAME_2_MS) {
			this.CONTROL_FRAME_PERIOD_MS = CONTROL_FRAME_PERIOD_MS;
			this.STATUS_FRAME_0_MS = STATUS_FRAME_0_MS;
			this.STATUS_FRAME_1_MS = STATUS_FRAME_1_MS;
			this.STATUS_FRAME_2_MS = STATUS_FRAME_2_MS;
		}
	}
}
