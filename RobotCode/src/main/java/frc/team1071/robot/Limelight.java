package frc.team1071.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("GRIP/linesReport");

    Limelight() {

    }

    void getLines() {
        double[] defaultValue = new double[0];
        //double[] lines = table.get
    }
}
