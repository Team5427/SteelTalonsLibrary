package lib.drivers;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SteelTalonsLogger {

    private static final String kLoggerTab = "5427Logger";
    private static ShuffleboardTab loggerTab = Shuffleboard.getTab(kLoggerTab);
    private static HashMap<String, GenericEntry> widgetList = new HashMap<>();

    public static boolean post(String key, Object obj) {
        if (!widgetList.containsKey(key)) {
            widgetList.put(key, loggerTab.add(key, obj).getEntry());
            return widgetList.get(key).setValue(obj);
        } else {
            return widgetList.get(key).setValue(obj);
        }
    }

    public static void postComplex(String key, Sendable sendable) {
        try {
            loggerTab.add(key, sendable);
        } catch (Exception e) {
            return;
        }
    }
    
}
