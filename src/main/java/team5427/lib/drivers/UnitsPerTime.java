package team5427.lib.drivers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class UnitsPerTime {
    private Rotation2d rotations;
    private double lastConvertedValue;
    private double rotatingDiameterMeters;
    private double timeSeconds;

    public UnitsPerTime(Rotation2d rotations, double timeSeconds) {
        this.timeSeconds = timeSeconds;
        this.rotations = rotations;
    }

    public static UnitsPerTime fromRotation2dPerMinutes(Rotation2d rotations, double timeMinutes) {
        return new UnitsPerTime(rotations, timeMinutes * 60.0);
    }

    public static UnitsPerTime fromRotation2dPerHour(Rotation2d rotations, double timeHours) {
        return new UnitsPerTime(rotations, timeHours * 60.0 * 60.0);
    }

    public static UnitsPerTime fromMetersPerMinute(double rotatingMeters, double meters, double timeMinutes) {
        return UnitsPerTime.fromMetersPerSecond(rotatingMeters, meters, timeMinutes / 60.0);
    }

    public static UnitsPerTime fromMetersPerSecond(double rotatingDiameterMeters, double metersTraveled,
            double timeSeconds) {

        Rotation2d rotations = new Rotation2d(metersTraveled / rotatingDiameterMeters * 2.0);
        return new UnitsPerTime(rotations, timeSeconds);
    }

    public UnitsPerTime getRotations() {
        lastConvertedValue = rotations.getRotations();
        return this;
    }

    public UnitsPerTime getRadians() {
        lastConvertedValue = rotations.getRadians();
        return this;
    }

    public UnitsPerTime getDegrees() {
        lastConvertedValue = rotations.getDegrees();
        return this;
    }

    public UnitsPerTime getMeters() {
        lastConvertedValue = rotations.getRadians() * rotatingDiameterMeters / 2.0;
        return this;
    }

    public UnitsPerTime getCentiMeters() {

        lastConvertedValue = getMeters().lastConvertedValue * 100.0;
        return this;
    }

    public UnitsPerTime getKiloMeters() {
        lastConvertedValue = getMeters().lastConvertedValue / 1000.0;
        return this;
    }

    public UnitsPerTime getInches() {
        lastConvertedValue = Units.metersToInches(getMeters().lastConvertedValue);
        return this;
    }

    public UnitsPerTime getFeet() {
        lastConvertedValue = Units.metersToFeet(getMeters().lastConvertedValue);
        return this;
    }

    public double getPerMilliSeconds() {
        return getPerSeconds() / 1000.0;
    }

    public double getPerSeconds() {
        return lastConvertedValue / timeSeconds;
    }

    public double getPerMinutes() {
        return getPerSeconds() * 60.0;
    }

    public double getPerHours() {
        return getPerHours() * 60.0;
    }
}
