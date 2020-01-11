package frc.ravenhardware;

public class RavenEncoder {
    private RavenTalon _ravenTalon;

    private int _cyclesPerRevolution;
    private double _wheelDiameterInches;
    private double _wheelCircumferenceInches;

    private boolean _inverted = false;

    public RavenEncoder(RavenTalon ravenTalon, int cyclesPerRevolution, double wheelDiameterInches, boolean inverted) {
        _ravenTalon = ravenTalon;
        _cyclesPerRevolution = cyclesPerRevolution;
        _wheelDiameterInches = wheelDiameterInches;
        _inverted = inverted;

        this._wheelCircumferenceInches = Math.PI * _wheelDiameterInches;
    }

    public double getNetRevolutions() {
        double netRevolutions = (double) _ravenTalon.getEncoderPosition() / _cyclesPerRevolution;

        if (_inverted) {
            netRevolutions *= -1;
        }

        return netRevolutions;
    }

    public double getNetInchesTraveled() {
        double netRevolutions = getNetRevolutions();
        double netInchesTraveled = netRevolutions * _wheelCircumferenceInches;

        return netInchesTraveled / 2;
    }

    public int getCycles() {
        int cycles = _ravenTalon.getEncoderPosition();

        if (_inverted) {
            cycles *= -1;
        }

        return cycles;
    }

    public void resetEncoder() {
        _ravenTalon.resetEncoderPosition();
    }
}