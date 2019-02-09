package frc.team195.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import java.util.HashMap;

public enum MCControlMode {
    PercentOut(0) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.PercentOutput;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kDutyCycle;
        }
    },
    Position(1) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.Position;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kPosition;
        }
    },
    Velocity(2) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.Velocity;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kVelocity;
        }
    },
    Current(3) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.Current;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kDutyCycle;
        }
    },
    Voltage(4) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.Disabled;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kVoltage;
        }
    },
    MotionMagic(5) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.MotionMagic;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kSmartMotion;
        }
    },
    SmartVelocity(6) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.Velocity;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kVelocity;
        }
    },
    MotionVoodooArbFF(7) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.Velocity;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kVelocity;
        }
    },
    Disabled(15) {
        @Override
        public ControlMode CTRE() {
            return ControlMode.Disabled;
        }

        @Override
        public ControlType Rev() {
            return ControlType.kDutyCycle;
        }
    };

    // Keep static maps to do fast lookup on control types via int and other motor library types
    private static HashMap<Integer, MCControlMode> intLookupMap = new HashMap<>();
    private static HashMap<ControlMode, MCControlMode> ctreLookupMap = new HashMap<>();
    private static HashMap<ControlType, MCControlMode> revLookupMap = new HashMap<>();

    static {
        for (MCControlMode type : MCControlMode.values()) {
            intLookupMap.put(type.value, type);
        }

        ctreLookupMap.put(ControlMode.PercentOutput, PercentOut);
        ctreLookupMap.put(ControlMode.Position, Position);
        ctreLookupMap.put(ControlMode.Velocity, Velocity);
        ctreLookupMap.put(ControlMode.Current, Current);
        ctreLookupMap.put(ControlMode.MotionMagic, MotionMagic);

        revLookupMap.put(ControlType.kDutyCycle, PercentOut);
        revLookupMap.put(ControlType.kPosition, Position);
        revLookupMap.put(ControlType.kVelocity, Velocity);
        revLookupMap.put(ControlType.kVoltage, Voltage);
        revLookupMap.put(ControlType.kSmartMotion, MotionMagic);
//		revLookupMap.put(ControlType.kSmartVelocity, SmartVelocity);
    }

    public final int value;

    MCControlMode(int initValue) {
        this.value = initValue;
    }

    public static MCControlMode valueOf(Object value) {
        MCControlMode retval = null;

        if (value instanceof ControlMode) {
            retval = ctreLookupMap.get(value);
        } else if (value instanceof ControlType) {
            retval = revLookupMap.get(value);
        } else if (value instanceof Integer) {
            retval = intLookupMap.get(value);
        } else if (value instanceof Double) {
            retval = intLookupMap.get((int) ((double) value));
        }

        if (retval != null)
            return retval;
        return Disabled;
    }

    public abstract ControlMode CTRE();

    public abstract ControlType Rev();

    @Override
    public String toString() {
        return valueOf(value).name();
    }
}