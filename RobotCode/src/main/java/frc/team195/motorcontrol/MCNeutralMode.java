package frc.team195.motorcontrol;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import java.util.HashMap;

public enum MCNeutralMode {
	Coast(0) {
		@Override
		public NeutralMode CTRE() {
			return NeutralMode.Coast;
		}
		@Override
		public IdleMode Rev() {
			return IdleMode.kCoast;
		}
	},
	Brake(1) {
		@Override
		public NeutralMode CTRE() {
			return NeutralMode.Brake;
		}
		@Override
		public IdleMode Rev() {
			return IdleMode.kBrake;
		}
	};

	public final int value;
	MCNeutralMode(int initValue)
	{
		this.value = initValue;
	}

	private static HashMap<Integer, MCNeutralMode> intLookupMap = new HashMap<>();
	private static HashMap<NeutralMode, MCNeutralMode> ctreLookupMap = new HashMap<>();
	private static HashMap<IdleMode, MCNeutralMode> revLookupMap = new HashMap<>();
	static {
		for (MCNeutralMode type : MCNeutralMode.values()) {
			intLookupMap.put(type.value, type);
		}

		ctreLookupMap.put(NeutralMode.Coast, Coast);
		ctreLookupMap.put(NeutralMode.Brake, Brake);

		revLookupMap.put(IdleMode.kCoast, Coast);
		revLookupMap.put(IdleMode.kBrake, Brake);
	}

	public static MCNeutralMode valueOf(Object value) {
		MCNeutralMode retval = null;

		if (value instanceof NeutralMode) {
			retval = ctreLookupMap.get(value);
		} else if (value instanceof IdleMode) {
			retval = revLookupMap.get(value);
		} else if (value instanceof Integer) {
			retval = intLookupMap.get(value);
		} else if (value instanceof Double) {
			retval = intLookupMap.get((int) ((double) value));
		}

		if (retval != null)
			return retval;
		return Coast;
	}


	public abstract NeutralMode CTRE();
	public abstract IdleMode Rev();
}
