package lib.kinematics;

import lib.drivers.ComplexGearRatio;

public class SwerveUtil {

    public static final int kFrontLeftModuleIdx = 0;
    public static final int kFrontRightModuleIdx = 1;
    public static final int kRearLeftModuleIdx = 2;
    public static final int kRearRightModuleIdx = 3;

    public static final int[] kSteerMotorIds = new int[4];
    public static final int[] kDriveMotorIds = new int[4];
    public static final int[] kCancoderIds = new int[4];
    public static final double[] kModuleOffsets = new double[4];

    public static final ComplexGearRatio kSDSSteerGearRatio = new ComplexGearRatio(1.0 / (150.0 / 7.0));
    public static final ComplexGearRatio kSDSL1GearRatio = new ComplexGearRatio((14.0 / 50.0), (25.0 / 19.0), (15.0 / 45.0));
    public static final ComplexGearRatio kSDSL2GearRatio = new ComplexGearRatio((14.0 / 50.0), (27.0 / 17.0), (15.0 / 45.0));
    public static final ComplexGearRatio kSDSL3GearRatio = new ComplexGearRatio((14.0 / 50.0), (28.0 / 16.0), (15.0 / 45.0));
    
}
