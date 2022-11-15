// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.function.DoubleSupplier;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.util.logging.Logger.LogPriority;

/** Add your docs here. */
public class DoubleLog {
    private DoubleLogEntry logEntry;
    private NetworkTableEntry shufflebordLogEntry;
    private DoubleSupplier supplier;
    private final LogPriority logType;
    private boolean continuous = false;

    private String key;

    public DoubleLog(String key, LogPriority logPriority, String prefix) {
        this.key = key;
        this.logType = logPriority;
        switch (logPriority) {
            case ONBOARD_ONLY:
            logEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "/" + key);
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

    public DoubleLog(String key, LogPriority logPriority, SubsystemLogger subsystem){
        this(key, logPriority, subsystem.getName());
    }

    public DoubleLog(String key, LogPriority logPriority, String prefix,  DoubleSupplier continuesLog) {
        this(key, logPriority, prefix);
        supplier = continuesLog;
        continuous = true;
    }

    public DoubleLog(String key, LogPriority logPriority, SubsystemLogger subsystem,  DoubleSupplier continuousLog) {
        this(key, logPriority, subsystem.getName(), continuousLog);
    }

    public void log(double value) {
        switch (logType) {
            case ONBOARD_ONLY:
            logEntry.append(value);
                break;
            case SHUFFLEBOARD:
            shufflebordLogEntry.setDouble(value);
                break;
            default:
                break;
        }
        
    }

   
    public void log() {
        if(continuous){
            switch (logType) {
                case ONBOARD_ONLY:
                logEntry.append(supplier.getAsDouble());
                    break;
                case SHUFFLEBOARD:
                shufflebordLogEntry.setDouble(supplier.getAsDouble());
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

