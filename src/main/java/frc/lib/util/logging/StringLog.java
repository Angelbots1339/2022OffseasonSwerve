// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.function.Supplier;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.util.logging.Logger.LogPriority;

/** Add your docs here. */
public class StringLog {
    private StringLogEntry logEntry;
    private NetworkTableEntry shufflebordLogEntry;
    private Supplier<String> supplier;
    private final LogPriority logPriority;
    private boolean continuous = false;
    private String key;

    public StringLog(String key, LogPriority LogPriority, String prefix) {
        this.key = key;
        this.logPriority = LogPriority;
        switch (LogPriority) {
            case ONBOARD_ONLY:
            logEntry = new StringLogEntry(DataLogManager.getLog(), prefix + "/" + key);
                break;
            case SHUFFLEBOARD:
            shufflebordLogEntry = Shuffleboard.getTab(prefix).add(key, 0).getEntry();
                break;
            case NONE:
                break;
            default:
                break;
        }
    }

    public StringLog(String key, LogPriority logPriority, SubsystemLogger subsystem){
        this(key, logPriority, subsystem.getName());
    }

    public StringLog(String key, LogPriority logPriority, String prefix,  Supplier<String> continuousLog) {
        this(key, logPriority, prefix);
        supplier = continuousLog;
        continuous = true;
    }

    public StringLog(String key, LogPriority logPriority, SubsystemLogger subsystem,  Supplier<String> continuousLog) {
        this(key, logPriority, subsystem.getName(), continuousLog);
    }

    public void log(String value) {
        switch (logPriority) {
            case ONBOARD_ONLY:
            logEntry.append(value);
                break;
            case SHUFFLEBOARD:
            shufflebordLogEntry.setString(value);
                break;
            default:
                break;
        }
        
    }

   
    public void log() {
        if(continuous){
            switch (logPriority) {
                case ONBOARD_ONLY:
                logEntry.append(supplier.get());
                    break;
                case SHUFFLEBOARD:
                shufflebordLogEntry.setString(supplier.get());
                    break;
                default:
                    break; 
            }
        }
    }
     public String getKey() {
         return key;
     }
}
