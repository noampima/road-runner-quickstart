package org.firstinspires.ftc.teamcode.RoadRunner.auto.Actions;

import org.firstinspires.ftc.teamcode.RoadRunner.util.Stopwatch;

public class ActionHelper {
    static boolean activateSystem(Stopwatch timer, Runnable systemFunction, long delay) {
        if (timer.hasTimePassed(delay)) {
            systemFunction.run();
            timer.reset();
            return true; // Activation successful
        } else {
            return false; // Activation failed
        }
    }
}
