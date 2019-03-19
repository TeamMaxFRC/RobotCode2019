package frc.team1071.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCBundle;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

import java.net.InetAddress;
import java.util.BitSet;

class OscSender {

    // Create the OSC sender on the robot.
    private OSCPortOut oscWirelessSender;
    private OSCPortOut oscWiredSender;

    /**
     * OscSender constructor.
     */
    OscSender() {

        // Try to open the OSC sockets.
        try {
            oscWirelessSender = new OSCPortOut(InetAddress.getByName("10.10.71.9"), 5803);
            oscWiredSender = new OSCPortOut(InetAddress.getByName("10.10.71.5"), 5803);
        } catch (Exception Ex) {
            System.out.println("OSC Initialization Exception: " + Ex.getMessage());
        }

    }

    /**
     * Pools together the error data and sends it to the dashboard.
     */
    void sendOscErrorData(CANSparkMax leftMaster, CANSparkMax rightMaster, CANSparkMax leftSlavePrimary,
            CANSparkMax rightSlavePrimary, CANSparkMax leftSlaveSecondary, CANSparkMax rightSlaveSecondary) {

        // Variable for converting Spark MAX faults.
        short faultShort;
        byte[] faultBytes = new byte[2];
        BitSet faultBits;

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("ErrorBundle");

        // See which errors there are and send to error widget.
        OSCMessage leftMasterFaults = new OSCMessage();
        leftMasterFaults.setAddress("/LeftMasterFaults");

        faultShort = leftMaster.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            leftMasterFaults.addArgument(i);
        }

        if (leftMasterFaults.getArguments().isEmpty()) {
            leftMasterFaults.addArgument(-1);
        }

        OSCMessage rightMasterFaults = new OSCMessage();
        rightMasterFaults.setAddress("/RightMasterFaults");

        faultShort = rightMaster.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            rightMasterFaults.addArgument(i);
        }

        if (rightMasterFaults.getArguments().isEmpty()) {
            rightMasterFaults.addArgument(-1);
        }

        OSCMessage leftSlavePrimaryFaults = new OSCMessage();
        leftSlavePrimaryFaults.setAddress("/LeftSlavePrimaryFaults");

        faultShort = leftSlavePrimary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            leftSlavePrimaryFaults.addArgument(i);
        }

        if (leftSlavePrimaryFaults.getArguments().isEmpty()) {
            leftSlavePrimaryFaults.addArgument(-1);
        }

        OSCMessage rightSlavePrimaryFaults = new OSCMessage();
        rightSlavePrimaryFaults.setAddress("/RightSlavePrimaryFaults");

        faultShort = rightSlavePrimary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            rightSlavePrimaryFaults.addArgument(i);
        }

        if (rightSlavePrimaryFaults.getArguments().isEmpty()) {
            rightSlavePrimaryFaults.addArgument(-1);
        }

        OSCMessage leftSlaveSecondaryFaults = new OSCMessage();
        leftSlaveSecondaryFaults.setAddress("/LeftSlaveSecondaryFaults");

        faultShort = leftSlaveSecondary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            leftSlaveSecondaryFaults.addArgument(i);
        }

        if (leftSlaveSecondaryFaults.getArguments().isEmpty()) {
            leftSlaveSecondaryFaults.addArgument(-1);
        }

        OSCMessage rightSlaveSecondaryFaults = new OSCMessage();
        rightSlaveSecondaryFaults.setAddress("/RightSlaveSecondaryFaults");

        faultShort = rightSlaveSecondary.getFaults();

        faultBytes[0] = (byte) (faultShort & 0xFF);
        faultBytes[1] = (byte) (faultShort >> 8 & 0xFF);

        faultBits = BitSet.valueOf(faultBytes);

        for (int i = faultBits.nextSetBit(0); i >= 0; i = faultBits.nextSetBit(i + 1)) {
            rightSlaveSecondaryFaults.addArgument(i);
        }

        if (rightSlaveSecondaryFaults.getArguments().isEmpty()) {
            rightSlaveSecondaryFaults.addArgument(-1);
        }

        // Add packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(leftMasterFaults);
        bundle.addPacket(rightMasterFaults);
        bundle.addPacket(leftSlavePrimaryFaults);
        bundle.addPacket(rightSlavePrimaryFaults);
        bundle.addPacket(leftSlaveSecondaryFaults);
        bundle.addPacket(rightSlaveSecondaryFaults);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the error data! " + ex.getMessage());
        }

    }

    /**
     * Pools together the current data for each subsystem and sends it to the
     * dashboard.
     * 
     * @param driveTrain    Drive train class, which contains all the drive motors.
     * @param lift          Lift class, which contains the elevator and four bar
     *                      motors.
     * @param gathererMotor Gatherer motor instance.
     * @param compressor    Compressor instance.
     */
    void sendOscCurrentData(CurvatureDrive driveTrain, Lift lift, Intake intake, Compressor compressor) {

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("CurrentBundle");

        // Append the current total for the drive motors.
        OSCMessage driveCurrent = new OSCMessage();
        driveCurrent.setAddress("/DriveCurrent");
        driveCurrent.addArgument(driveTrain.getCurrent());

        // TODO: Rename elevator current.
        // Append the current total for the lift motors.
        OSCMessage liftCurrent = new OSCMessage();
        liftCurrent.setAddress("/LiftCurrent");
        liftCurrent.addArgument(lift.getElevatorCurrent());

        // Append the current for the four bar motor.
        OSCMessage fourBarCurrent = new OSCMessage();
        fourBarCurrent.setAddress("/FourBarCurrent");
        fourBarCurrent.addArgument(lift.getFourBarCurrent());

        // Append the current for the gatherer motor.
        OSCMessage gathererCurrent = new OSCMessage();
        gathererCurrent.setAddress("/GathererCurrent");
        gathererCurrent.addArgument(intake.getCurrent());

        // Append the current for the compressor.
        OSCMessage compressorCurrent = new OSCMessage();
        compressorCurrent.setAddress("/CompressorCurrent");
        compressorCurrent.addArgument(compressor.getCompressorCurrent());

        // Add these packets to the bundle.
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(driveCurrent);
        bundle.addPacket(liftCurrent);
        bundle.addPacket(fourBarCurrent);
        bundle.addPacket(gathererCurrent);
        bundle.addPacket(compressorCurrent);

        // Send the drive log data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the current data! " + ex.getMessage());
        }
    }

    void sendOscSensorData(CurvatureDrive driveTrain, Lift lift, Intake intake) {

        // Create an OSC bundle for encoder velocities.
        OSCBundle bundle = new OSCBundle();

        // Create integer to convert hatchGrabber bool to an int
        double convertedBoolean;

        // Append an identifier for the bundle.
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("SensorInputBundle");

        // Send the lift encoder velocity and position.
        OSCMessage liftEncoderVelocity = new OSCMessage();
        liftEncoderVelocity.setAddress("/LiftEncoderVelocity");
        liftEncoderVelocity.addArgument((double) lift.getElevatorVelocity());
        ;

        OSCMessage liftEncoderPosition = new OSCMessage();
        liftEncoderPosition.setAddress("/LiftEncoderPosition");
        liftEncoderPosition.addArgument((double) lift.getElevatorPosition());

        // Send the drive encoder velocity.
        OSCMessage leftEncoderVelocity = new OSCMessage();
        leftEncoderVelocity.setAddress("/LeftEncoderVelocity");
        leftEncoderVelocity.addArgument(driveTrain.getLeftEncoderVelocity());

        OSCMessage rightEncoderVelocity = new OSCMessage();
        rightEncoderVelocity.setAddress("/RightEncoderVelocity");
        rightEncoderVelocity.addArgument(driveTrain.getRightEncoderVelocity());

        OSCMessage magneticGatherPosition = new OSCMessage();
        magneticGatherPosition.setAddress("/MagneticGatherEncoder");
        convertedBoolean = intake.getHatchLimitSwitch() ? 1 : 0;
        magneticGatherPosition.addArgument(convertedBoolean);

        // Four bar relative encoder position.
        OSCMessage fourBarEncoderRelativePosition = new OSCMessage();
        fourBarEncoderRelativePosition.setAddress("/FourBarEncoderRelativePosition");
        fourBarEncoderRelativePosition.addArgument(lift.getFourBarDegrees());

        // Four bar absolute encoder position.
        OSCMessage fourBarEncoderAbsolutePosition = new OSCMessage();
        fourBarEncoderAbsolutePosition.setAddress("/FourBarEncoderAbsolutePosition");
        fourBarEncoderAbsolutePosition.addArgument(lift.getAbsoluteFourBarTicks());

        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(liftEncoderPosition);
        bundle.addPacket(rightEncoderVelocity);
        bundle.addPacket(leftEncoderVelocity);
        bundle.addPacket(liftEncoderVelocity);
        bundle.addPacket(magneticGatherPosition);
        bundle.addPacket(fourBarEncoderRelativePosition);
        bundle.addPacket(fourBarEncoderAbsolutePosition);

        // Send the sensor data.
        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the error data! " + ex.getMessage());
        }

    }

    /**
     * This function writes the provided line to the dashboard console.
     *
     * @param Line The string being written to the dashboard console.
     */
    void writeConsole(String Line) {

        try {
            // Create the console message.
            OSCMessage ConsoleMessage = new OSCMessage();

            // Set the address and arguments for the message.
            ConsoleMessage.setAddress("/Robot/Console/Text");
            ConsoleMessage.addArgument(Line);

            // Send the console message.
            oscWirelessSender.send(ConsoleMessage);
            oscWiredSender.send(ConsoleMessage);

        } catch (Exception Ex) {
            System.out.println("Console Sending Exception: " + Ex.getMessage());
        }
    }

    // Send controller data
    void SendOscControllerData(Joystick driverJoystick, Joystick operatorJoystick) {

        // Create an OSC bundle.
        OSCBundle bundle = new OSCBundle();

        // Append an identifier for the bundle
        OSCMessage bundleIdentifier = new OSCMessage();
        bundleIdentifier.setAddress("/BundleIdentifier");
        bundleIdentifier.addArgument("ControllerDataBundle");

        int convertedBoolean;

        // Send driver controller data.
        OSCMessage aButton = new OSCMessage();
        aButton.setAddress("/Controller/AButton");
        convertedBoolean = driverJoystick.getRawButton(1) ? 1 : 0;
        aButton.addArgument(convertedBoolean);

        OSCMessage bButton = new OSCMessage();
        bButton.setAddress("/Controller/BButton");
        convertedBoolean = driverJoystick.getRawButton(2) ? 1 : 0;
        bButton.addArgument(convertedBoolean);

        OSCMessage xButton = new OSCMessage();
        xButton.setAddress("/Controller/XButton");
        convertedBoolean = driverJoystick.getRawButton(3) ? 1 : 0;
        xButton.addArgument(convertedBoolean);

        OSCMessage yButton = new OSCMessage();
        yButton.setAddress("/Controller/YButton");
        convertedBoolean = driverJoystick.getRawButton(4) ? 1 : 0;
        yButton.addArgument(convertedBoolean);

        OSCMessage leftBumper = new OSCMessage();
        leftBumper.setAddress("/Controller/LeftBumper");
        convertedBoolean = driverJoystick.getRawButton(5) ? 1 : 0;
        leftBumper.addArgument(convertedBoolean);

        OSCMessage rightBumper = new OSCMessage();
        rightBumper.setAddress("/Controller/RightBumper");
        convertedBoolean = driverJoystick.getRawButton(6) ? 1 : 0;
        rightBumper.addArgument(convertedBoolean);

        OSCMessage viewButton = new OSCMessage();
        viewButton.setAddress("/Controller/viewButton");
        convertedBoolean = driverJoystick.getRawButton(7) ? 1 : 0;
        viewButton.addArgument(convertedBoolean);

        OSCMessage menuButton = new OSCMessage();
        menuButton.setAddress("/Controller/menuButton");
        convertedBoolean = driverJoystick.getRawButton(8) ? 1 : 0;
        menuButton.addArgument(convertedBoolean);

        OSCMessage leftStickButton = new OSCMessage();
        leftStickButton.setAddress("/Controller/leftStickButton");
        convertedBoolean = driverJoystick.getRawButton(9) ? 1 : 0;
        leftStickButton.addArgument(convertedBoolean);

        OSCMessage rightStickButton = new OSCMessage();
        rightStickButton.setAddress("/Controller/rightStickButton");
        convertedBoolean = driverJoystick.getRawButton(10) ? 1 : 0;
        rightStickButton.addArgument(convertedBoolean);

        OSCMessage leftStickAxisX = new OSCMessage();
        leftStickAxisX.setAddress("/Controller/LeftStickAxisX");
        leftStickAxisX.addArgument(driverJoystick.getRawAxis(0));

        OSCMessage leftStickAxisY = new OSCMessage();
        leftStickAxisY.setAddress("/Controller/LeftStickAxisY");
        leftStickAxisY.addArgument(driverJoystick.getRawAxis(1));

        OSCMessage rightStickAxisX = new OSCMessage();
        rightStickAxisX.setAddress("/Controller/RightStickAxisX");
        rightStickAxisX.addArgument(driverJoystick.getRawAxis(4));

        OSCMessage rightStickAxisY = new OSCMessage();
        rightStickAxisY.setAddress("/Controller/RightStickAxisY");
        rightStickAxisY.addArgument(driverJoystick.getRawAxis(5));

        OSCMessage leftTrigger = new OSCMessage();
        leftTrigger.setAddress("/Controller/LeftTrigger");
        leftTrigger.addArgument(driverJoystick.getRawAxis(2));

        OSCMessage rightTrigger = new OSCMessage();
        rightTrigger.setAddress("/Controller/RightTrigger");
        rightTrigger.addArgument(driverJoystick.getRawAxis(3));

        // Send operator controller data
        OSCMessage aButton2 = new OSCMessage();
        aButton2.setAddress("/Controller/AButton2");
        convertedBoolean = operatorJoystick.getRawButton(1) ? 1 : 0;
        aButton2.addArgument(convertedBoolean);

        OSCMessage bButton2 = new OSCMessage();
        bButton2.setAddress("/Controller/BButton2");
        convertedBoolean = operatorJoystick.getRawButton(2) ? 1 : 0;
        bButton2.addArgument(convertedBoolean);

        OSCMessage xButton2 = new OSCMessage();
        xButton2.setAddress("/Controller/XButton2");
        convertedBoolean = operatorJoystick.getRawButton(3) ? 1 : 0;
        xButton2.addArgument(convertedBoolean);

        OSCMessage yButton2 = new OSCMessage();
        yButton2.setAddress("/Controller/YButton2");
        convertedBoolean = operatorJoystick.getRawButton(4) ? 1 : 0;
        yButton2.addArgument(convertedBoolean);

        OSCMessage leftBumper2 = new OSCMessage();
        leftBumper2.setAddress("/Controller/LeftBumper2");
        convertedBoolean = operatorJoystick.getRawButton(5) ? 1 : 0;
        leftBumper2.addArgument(convertedBoolean);

        OSCMessage rightBumper2 = new OSCMessage();
        rightBumper2.setAddress("/Controller/RightBumper2");
        convertedBoolean = operatorJoystick.getRawButton(6) ? 1 : 0;
        rightBumper2.addArgument(convertedBoolean);

        OSCMessage viewButton2 = new OSCMessage();
        viewButton2.setAddress("/Controller/viewButton2");
        convertedBoolean = operatorJoystick.getRawButton(7) ? 1 : 0;
        viewButton2.addArgument(convertedBoolean);

        OSCMessage menuButton2 = new OSCMessage();
        menuButton2.setAddress("/Controller/menuButton2");
        convertedBoolean = operatorJoystick.getRawButton(8) ? 1 : 0;
        menuButton2.addArgument(convertedBoolean);

        OSCMessage leftStickButton2 = new OSCMessage();
        leftStickButton2.setAddress("/Controller/leftStickButton2");
        convertedBoolean = operatorJoystick.getRawButton(9) ? 1 : 0;
        leftStickButton2.addArgument(convertedBoolean);

        OSCMessage rightStickButton2 = new OSCMessage();
        rightStickButton2.setAddress("/Controller/rightStickButton2");
        convertedBoolean = operatorJoystick.getRawButton(10) ? 1 : 0;
        rightStickButton2.addArgument(convertedBoolean);

        OSCMessage leftStickAxisX2 = new OSCMessage();
        leftStickAxisX2.setAddress("/Controller/LeftStickAxisX2");
        leftStickAxisX2.addArgument(operatorJoystick.getRawAxis(0));

        OSCMessage leftStickAxisY2 = new OSCMessage();
        leftStickAxisY2.setAddress("/Controller/LeftStickAxisY2");
        leftStickAxisY2.addArgument(operatorJoystick.getRawAxis(1));

        OSCMessage rightStickAxisX2 = new OSCMessage();
        rightStickAxisX2.setAddress("/Controller/RightStickAxisX2");
        rightStickAxisX2.addArgument(operatorJoystick.getRawAxis(4));

        OSCMessage rightStickAxisY2 = new OSCMessage();
        rightStickAxisY2.setAddress("/Controller/RightStickAxisY2");
        rightStickAxisY2.addArgument(driverJoystick.getRawAxis(5));

        OSCMessage leftTrigger2 = new OSCMessage();
        leftTrigger2.setAddress("/Controller/LeftTrigger2");
        leftTrigger2.addArgument(operatorJoystick.getRawAxis(2));

        OSCMessage rightTrigger2 = new OSCMessage();
        rightTrigger2.setAddress("/Controller/RightTrigger2");
        rightTrigger2.addArgument(operatorJoystick.getRawAxis(3));

        // Add these packets to the bundle
        bundle.addPacket(bundleIdentifier);
        bundle.addPacket(aButton);
        bundle.addPacket(bButton);
        bundle.addPacket(xButton);
        bundle.addPacket(yButton);
        bundle.addPacket(viewButton);
        bundle.addPacket(menuButton);
        bundle.addPacket(leftBumper);
        bundle.addPacket(rightBumper);
        bundle.addPacket(leftStickButton);
        bundle.addPacket(rightStickButton);
        bundle.addPacket(leftStickAxisX);
        bundle.addPacket(leftStickAxisY);
        bundle.addPacket(rightStickAxisX);
        bundle.addPacket(rightStickAxisY);
        bundle.addPacket(rightTrigger);
        bundle.addPacket(leftTrigger);
        bundle.addPacket(aButton2);
        bundle.addPacket(bButton2);
        bundle.addPacket(xButton2);
        bundle.addPacket(yButton2);
        bundle.addPacket(viewButton2);
        bundle.addPacket(menuButton2);
        bundle.addPacket(leftBumper2);
        bundle.addPacket(rightBumper2);
        bundle.addPacket(leftStickButton2);
        bundle.addPacket(rightStickButton2);
        bundle.addPacket(leftStickAxisX2);
        bundle.addPacket(leftStickAxisY2);
        bundle.addPacket(rightStickAxisX2);
        bundle.addPacket(rightStickAxisY2);
        bundle.addPacket(rightTrigger2);
        bundle.addPacket(leftTrigger2);

        try {
            oscWiredSender.send(bundle);
            oscWirelessSender.send(bundle);
        } catch (Exception ex) {
            System.out.println("Error sending the controller data!" + ex.getMessage());
        }
    }

    // TODO: Bundle these packets.
    void sendOscLimelightData(double limelightX, double limelightY, double limelightArea, boolean limelightTarget) {
        // Send the values for the Limelight
        OSCMessage limelightMessageX = new OSCMessage();
        OSCMessage limelightMessageY = new OSCMessage();
        OSCMessage limelightMessageA = new OSCMessage();
        OSCMessage limelightMessageV = new OSCMessage();

        limelightMessageX.setAddress("/Robot/Limelight/X");
        limelightMessageY.setAddress("/Robot/Limelight/Y");
        limelightMessageA.setAddress("/Robot/Limelight/A");
        limelightMessageV.setAddress("/Robot/Limelight/V");

        limelightMessageX.addArgument(limelightX);
        limelightMessageY.addArgument(limelightY);
        limelightMessageA.addArgument(limelightArea);
        limelightMessageV.addArgument(limelightTarget ? 0 : 1);

        try {
            oscWirelessSender.send(limelightMessageX);
            oscWiredSender.send(limelightMessageX);
            oscWirelessSender.send(limelightMessageY);
            oscWiredSender.send(limelightMessageY);
            oscWirelessSender.send(limelightMessageA);
            oscWiredSender.send(limelightMessageA);
            oscWirelessSender.send(limelightMessageV);
            oscWiredSender.send(limelightMessageV);
        } catch (Exception ex) {
            System.out.println("Exception send Limelight OSC data: " + ex.toString());
        }
    }
}
