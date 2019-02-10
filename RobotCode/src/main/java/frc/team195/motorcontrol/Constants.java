package frc.team195.motorcontrol;

public class Constants {

	public static final int kLongCANTimeoutMs = 100;
	public static final int kTalonRetryCount = 3;
	public static final int kCANTimeoutMs = 10;

	//Breaker model for trip time output in seconds y = a*(current_percent_over_rating)^b + c
	public static final double kPDPBreakerModelA = 282.2962;
	public static final double kPDPBreakerModelB = -6.6305;
	public static final double kPDPBreakerModelC = 0.5;
	public static final double kPDPDefaultSafetyFactor = 4.0;
	public static final int LOG_OSC_REPORTER_PORT = 5801;
	public static final String DASHBOARD_IP = "";
	public static final boolean REPORTING_ENABLED = true;
	public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;

	public static final boolean TUNING_PIDS = true;
}
