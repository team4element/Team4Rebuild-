// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumSet;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class LiveDoubleBinding {
    DoubleSubscriber valueSubscriber;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shuffleboardTable = inst.getTable("Shuffleboard");

    public LiveDoubleBinding(String tabName, String key, Double defaultValue, Consumer<NetworkTableEvent> listener) {
        Shuffleboard.getTab(tabName).add(key, defaultValue).withWidget(BuiltInWidgets.kTextView);
        valueSubscriber = shuffleboardTable.getDoubleTopic(tabName + "/" + key).subscribe(defaultValue);

        if (listener != null) {
            inst.addListener(
                    valueSubscriber,
                    EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    listener);
        }
    }

    // Create another constructor without the listener
    public LiveDoubleBinding(String tabName, String key, Double defaultValue) {
        Shuffleboard.getTab(tabName).add(key, defaultValue).withWidget(BuiltInWidgets.kTextView);
        valueSubscriber = shuffleboardTable.getDoubleTopic(key).subscribe(defaultValue);
    }

    /** 
     * @return DoubleSubscriber
     */
    public DoubleSubscriber getSubscriber() {
        return valueSubscriber;
    }

    public Supplier<Double> getSupplier() {
        return () -> valueSubscriber.get();
    }

    public double getDouble() {
        return valueSubscriber.get();
    }

    public double getDouble(double defaultValue) {
        return valueSubscriber.get(defaultValue);
    }
}
