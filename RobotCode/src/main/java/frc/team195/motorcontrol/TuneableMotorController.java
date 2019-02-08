package frc.team195.motorcontrol;

import frc.team195.reporters.DiagnosticMessage;

/**
 * Interface for all motor control to abstract controllers from different vendors
 * Units expected in standard form, i.e. Rotations, RPM, etc.
 * All production set commands expected to be differential except those used for tuning
 */
public interface TuneableMotorController {
	void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward);

	void setPIDF(double kP, double kI, double kD, double kF);

	void setDFilter(double dFilter);

	void setIZone(double iZone);

	void setMCIAccum(double iAccum);

	void setMaxIAccum(double maxIAccum);

	void setMCOpenLoopRampRate(double rampRate);

	void setMCClosedLoopRampRate(double rampRate);

	void setMotionParameters(int cruiseVel, int cruiseAccel);

	void setPIDGainSlot(int slotIdx);

	void setBrakeCoastMode(MCNeutralMode neutralMode);

	void setEncoderPosition(double position);

	/**
	 * Method to change control modes. Make sure this method only changes modes if the current mode is not the desired mode.
	 *
	 * @param controlMode The desired control mode
	 */
	void setControlMode(MCControlMode controlMode);

	void setSetpoint(double setpoint);

	double getActual();

	double getSetpoint();

	double getPosition();

	double getVelocity();

	double getSensorUnitsPerRotation();

	double getVelocityRPMTimeConversionFactor();

	double getNativeUnitsOutputRange();

	double getMCIAccum();

	double getMCOutputCurrent();

	double getMCInputVoltage();

	double getMCOutputVoltage();

	double getMCOutputPercent();

	int getMCID();

	boolean isEncoderPresent();

	DiagnosticMessage hasMotorControllerReset();

	MCControlMode getMotionControlMode();

	default double convertNativeUnitsToRotations(double nativeUnitsPos) {
		return nativeUnitsPos / getSensorUnitsPerRotation();
	}

	default int convertRotationsToNativeUnits(double rotations) {
		return (int) (rotations * getSensorUnitsPerRotation());
	}

	default double convertNativeUnitsToRPM(int nativeUnits) {
		return (nativeUnits / getSensorUnitsPerRotation() * getVelocityRPMTimeConversionFactor());
	}

	default int convertRPMToNativeUnits(double rpm) {
		return (int) (rpm * getSensorUnitsPerRotation() / getVelocityRPMTimeConversionFactor());
	}

	default double convertDemandToNativeUnits(MCControlMode controlMode, double demand) {
		double output = demand;
		switch (controlMode) {
			case Position:
			case MotionMagic:
				output = convertRotationsToNativeUnits(demand);
				break;
			case MotionVoodooArbFF:
			case Velocity:
				output = convertRPMToNativeUnits(demand);
				break;
			default:
				break;
		}
		return output;
	}

	default int getMSDurationForBreakerLimit(double peakCurrentInput, double breakerRating) {
		return getMSDurationForBreakerLimit(peakCurrentInput, breakerRating, Constants.kPDPDefaultSafetyFactor);
	}

	default int getMSDurationForBreakerLimit(double peakCurrentInput, double breakerRating, double safetyFactor) {
		return (int)((Constants.kPDPBreakerModelA*Math.pow(peakCurrentInput/breakerRating, Constants.kPDPBreakerModelB)+Constants.kPDPBreakerModelC) * 1000.0 / safetyFactor);
	}

}
