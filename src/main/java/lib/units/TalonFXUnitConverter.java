package lib.units;

public class TalonFXUnitConverter {
    public static enum MechanismType {
        ARM,
        ELEVATOR,
        FLYWHEEL
    }

    private final double positionConversionFactor;
    private final double velocityConversionFactor;
    private final MechanismType type;

    public TalonFXUnitConverter(double positionConversionFactor, double velocityConversionFactor, MechanismType type) {
        this.positionConversionFactor = positionConversionFactor;
        this.velocityConversionFactor = velocityConversionFactor;
        this.type = type;
    }

    
}
