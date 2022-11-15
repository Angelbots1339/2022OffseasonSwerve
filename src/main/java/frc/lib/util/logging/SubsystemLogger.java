// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collector;
import java.util.stream.Collectors;


import frc.lib.util.logging.Logger.LogPriority;

/** Add your docs here. */
public class SubsystemLogger {
    private List<DoubleLog> doubleLogs = new ArrayList<DoubleLog>();
    private List<StringLog> stringLogs = new ArrayList<StringLog>();
    private LogPriority motorPriority;
    private LogPriority sensorPriority;
    private LogPriority calculatedValuesPriority;

    private String name;

  
    public SubsystemLogger(String name, LogPriority motorPriority, LogPriority sensorPriority, LogPriority calculatedValuesPriority) {
        this.motorPriority = motorPriority;
        this.sensorPriority = sensorPriority;
        this.calculatedValuesPriority = calculatedValuesPriority;
        Logger.getInstance().addSubsystem(this);
    }

    public void log() {
        for (DoubleLog doubleLog : doubleLogs) {
            doubleLog.log();
        }
        for (StringLog stringLog : stringLogs) {
            stringLog.log();
        }
    }

    public void add(String key, Supplier<String> strignSupplier, LogType logType) {
        if(getDoubleLog(key) != null){throw new IllegalArgumentException("Loged value with the same key " + key + "already exsits");}
        if(getStringLog(key) != null){throw new IllegalArgumentException("Loged value with the same key " + key + "already exsits");}

        switch (logType){
            case CALCULATED:
                stringLogs.add(new StringLog(key, calculatedValuesPriority, this, strignSupplier));
                break;
            case SENSOR:
                stringLogs.add(new StringLog(key, sensorPriority, this, strignSupplier));
                break;
            case MOTOR:
                stringLogs.add(new StringLog(key, motorPriority, this, strignSupplier));
            default:
                break;
        }
    }
    public void add(String key, DoubleSupplier doubleSupplier, LogType logType) {
        if(getDoubleLog(key) != null){throw new IllegalArgumentException("Loged value with the same key " + key + "already exsits");}
        if(getStringLog(key) != null){throw new IllegalArgumentException("Loged value with the same key " + key + "already exsits");}
        
        switch (logType){
            case CALCULATED:
                doubleLogs.add(new DoubleLog(key, calculatedValuesPriority, this, doubleSupplier));
                break;
            case SENSOR:
                doubleLogs.add(new DoubleLog(key, sensorPriority, this, doubleSupplier));
                break;
            case MOTOR:
                doubleLogs.add(new DoubleLog(key, motorPriority, this, doubleSupplier));
            default:
                break;
        }
    }


    public void upadate(String key, double value, LogType logType) {
        if(getStringLog(key) != null){throw new IllegalArgumentException("Loged value with the key " + key + "already exsits in string log");}
        if(getDoubleLog(key) == null){
            switch (logType){
                case CALCULATED:
                    doubleLogs.add(new DoubleLog(key, calculatedValuesPriority, this));
                    break;
                case SENSOR:
                    doubleLogs.add(new DoubleLog(key, sensorPriority, this));
                    break;
                case MOTOR:
                    doubleLogs.add(new DoubleLog(key, motorPriority, this));
                default:
                    break;
            }
        } 
        getDoubleLog(key).log(value);
        
    }
    public void upadate(String key, String value, LogType logType) {
        if(getDoubleLog(key) != null){throw new IllegalArgumentException("Loged value with the key " + key + "already exsits in double log");}
        if(getStringLog(key) == null){
            switch (logType){
                case CALCULATED:
                    stringLogs.add(new StringLog(key, calculatedValuesPriority, this));
                    break;
                case SENSOR:
                    stringLogs.add(new StringLog(key, sensorPriority, this));
                    break;
                case MOTOR:
                    stringLogs.add(new StringLog(key, motorPriority, this));
                default:
                    break;
            }
        } 
        getStringLog(key).log(value);
        
    }


    public String getName() {
        return name;
    }

    private DoubleLog getDoubleLog(String key){
        return doubleLogs.stream()
            .filter(log -> log.getKey().equals(key))
            .collect(toSingleLog());
    }
    private StringLog getStringLog(String key){
        return stringLogs.stream()
            .filter(log -> log.getKey().equals(key))
            .collect(toSingleLog());
    }
    
    private static <T> Collector<T, ?, T> toSingleLog() {
        return Collectors.collectingAndThen(
                Collectors.toList(),
                list -> {
                    if (list.size() > 1) {
                        throw new IllegalStateException();
                    }
                    else if (list.size() == 1) {
                        return list.get(0);
                    }
                    return null;
                }
        );
    }

    public enum LogType {
        MOTOR,
        SENSOR,
        CALCULATED
    }
    
}

