package frc.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableDiagnostics {
    private static final NetworkTable _table = NetworkTableInstance.getDefault().getTable("Diagnostics");
    private static int _counter = 0;

    private interface DiagnosticSendFunction {
        void Send();
    }

    public interface BooleanResult {
        boolean GetBoolean();
    }

    public interface NumberResult {
        Number GetNumber();
    }

    private static List<DiagnosticSendFunction> _registeredFunctions = new ArrayList<DiagnosticSendFunction>();

    public static void SubsystemBoolean(String subsystemName, String name, BooleanResult function){
        NetworkTableEntry entry = _table.getEntry("Subsystem/" + subsystemName + "/" + name);
        DiagnosticSendFunction dSendFunc = () -> {
            entry.setBoolean(function.GetBoolean());
        };

        _registeredFunctions.add(dSendFunc);
    }

    public static void SubsystemNumber(String subsystemName, String name, NumberResult function){
        NetworkTableEntry entry = _table.getEntry("Subsystem/" + subsystemName + "/" + name);
        DiagnosticSendFunction dSendFunc = () -> {
            entry.setNumber(function.GetNumber());
        };

        _registeredFunctions.add(dSendFunc);
    }

    public static void SendData() {
        _counter++;
        if (_counter < 24) {
            return;
        }

        _counter = 0;
        for (DiagnosticSendFunction dsf : _registeredFunctions) {
            dsf.Send();
        }
    }
}