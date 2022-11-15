// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class Logger {

    private static Logger logger;
    private List<SubsystemLogger> subsystems = new ArrayList<SubsystemLogger>();
    public static Logger getInstance() {
        if(logger == null) {
            logger = new Logger();
        }
        return logger;
    }

    public void addSubsystem(SubsystemLogger subsystemLogger) {
        subsystems.add(subsystemLogger);
    }

    public void log() {
        for (SubsystemLogger subsystemLogger : subsystems) {
            subsystemLogger.log();
        }
    }
   
    public enum LogPriority {
        SHUFFLEBOARD,
        ONBOARD_ONLY,
        NONE
    }
    
}
