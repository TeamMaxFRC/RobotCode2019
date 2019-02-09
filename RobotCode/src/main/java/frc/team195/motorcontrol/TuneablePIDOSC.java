package frc.team195.motorcontrol;

import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortIn;
import com.illposed.osc.OSCPortOut;
import frc.team195.utils.ThreadRateControl;

import java.io.IOException;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TuneablePIDOSC {
	private static final int MIN_PID_SEND_RATE_MS = 50;

	private String name;
	private ArrayList<TuneableMotorController> mcArr;
	private boolean runThread;

	private OSCPortOut oscPortOut;
	private OSCPortIn oscPortIn;

	private TuneablePIDData tpd = new TuneablePIDData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	private InetAddress IPAddress = null;
	private InetAddress prevIPAddress = null;


	public TuneablePIDOSC(String name, int portNumber, boolean autoUpdate, TuneableMotorController... tuneableMotorControllers) throws SocketException {
		if (Constants.TUNING_PIDS) {
			this.name = name;
			mcArr = new ArrayList<>(Arrays.asList(tuneableMotorControllers));

			oscPortIn = new OSCPortIn(portNumber);
			OSCListener updateListener = (time, message) -> {
				try {
					List<Object> valArr = message.getArguments();

					if (valArr.size() == 10) {
						setIPAddress(message.getIPAddress());
						setTuneablePIDData(processUDPPacket(valArr));
						for (TuneableMotorController tmc : mcArr) {
							if (autoUpdate) {
								tmc.setPIDF(tpd.kP, tpd.kI, tpd.kD, tpd.kF);
								tmc.setSetpoint(tpd.setpoint);
								tmc.setMCClosedLoopRampRate(tpd.rampRate);
								tmc.setIZone(tpd.iZone);
								tmc.setMaxIAccum(tpd.maxIAccum);
								tmc.setMotionParameters(tpd.cruiseVelocity, tpd.cruiseAccel);
								System.out.println(tpd.setpoint);
							}
						}
					}

				} catch (Exception ignored) {

				}
			};

			OSCListener iAccumResetListener = (time, message) -> {
				try {
					List<Object> valArr = message.getArguments();

					if (valArr.size() == 1) {
						setIPAddress(message.getIPAddress());
						for (TuneableMotorController tmc : mcArr) {
							tmc.setMCIAccum((double) valArr.get(0));
						}
					}

				} catch (Exception ignored) {

				}
			};

			oscPortIn.addListener("/PIDUpdate", updateListener);
			oscPortIn.addListener("/IReset", iAccumResetListener);
			oscPortIn.startListening();

			runThread = true;
			Thread oscSenderThread = new Thread(() -> {
				ThreadRateControl threadRateControl = new ThreadRateControl();
				threadRateControl.start();
				while (runThread) {
					try {
						if (oscPortOut == null || (IPAddress != null && !IPAddress.equals(prevIPAddress))) {
							if (IPAddress != null) {
								if (oscPortOut != null)
									oscPortOut.close();
								oscPortOut = new OSCPortOut(IPAddress, portNumber);
								prevIPAddress = IPAddress;
							}
							else {
								Thread.sleep(100);
								continue;
							}
						}
					} catch (Exception e) {
						e.printStackTrace();
						continue;
					}

					try {
						oscPortOut.send(createSendData());
					} catch (IOException e) {
						e.printStackTrace();
					}
					threadRateControl.doRateControl(MIN_PID_SEND_RATE_MS);
				}
				oscPortOut.close();
			});
			oscSenderThread.start();
		}
	}

	private synchronized void setIPAddress(InetAddress ipAddr) {
		try {
			IPAddress = ipAddr;
		} catch (Exception ex) {
			IPAddress = null;
		}
	}

	private OSCMessage createSendData() {
		ArrayList<Object> args = new ArrayList<>();
		if (mcArr.size() > 0) {
			TuneableMotorController tmc = mcArr.get(0);
			args.add((double)tmc.getActual());
			args.add((double)tpd.setpoint);
			args.add((double)tmc.getMCIAccum());
		} else {
			args.add((double)0);
			args.add((double)tpd.setpoint);
			args.add((double)0);
		}
		args.add(name);

		return new OSCMessage("/PIDData", args);
	}

	private synchronized void setTuneablePIDData(TuneablePIDData tuneablePIDData) {
		tpd = tuneablePIDData;
	}

	private TuneablePIDData processUDPPacket(List<Object> data) {

		return new TuneablePIDData(
				(double)data.get(0),
				(double)data.get(1),
				(double)data.get(2),
				(double)data.get(3),
				(double)data.get(4),
				(double)data.get(5),
				(double)data.get(6),
				(double)data.get(7),
				(double)data.get(8),
				(double)data.get(9)
		);
	}

	private class TuneablePIDData {
		double kP;
		double kI;
		double kD;
		double kF;
		double setpoint;
		double rampRate;
		double iZone;
		double maxIAccum;
		int cruiseVelocity;
		int cruiseAccel;

		TuneablePIDData(double kP, double kI, double kD, double kF, double cruiseAccel, double cruiseVelocity, double rampRate, double iZone, double setpoint, double maxIAccum) {
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
			this.kF = kF;
			this.setpoint = setpoint;
			this.cruiseVelocity = (int)cruiseVelocity;
			this.cruiseAccel = (int)cruiseAccel;
			this.rampRate = rampRate;
			this.iZone = iZone;
			this.maxIAccum = maxIAccum;
		}
	}
}
