package frc.team195.motorcontrol;

public enum PDPBreaker {
	B5A(5),
	B10A(10),
	B20A(20),
	B30A(30),
	B40A(40);

	public final int value;
	PDPBreaker(int initValue)
	{
		this.value = initValue;
	}
}
