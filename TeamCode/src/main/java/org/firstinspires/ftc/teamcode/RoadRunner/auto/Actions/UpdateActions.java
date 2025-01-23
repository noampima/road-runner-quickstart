package org.firstinspires.ftc.teamcode.RoadRunner.auto.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RoadRunner.RobotHardware;

import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.RoadRunner.util.ClawSide;

public class UpdateActions {


    private Elevator elevator;

    private Outtake outtake;
    private Claw claw;
    RobotHardware robot = RobotHardware.getInstance();

    int counter = 0;
    public UpdateActions(Elevator elevator, Claw claw, Outtake outtake) {
        this.elevator = elevator;
        this.claw = claw;
        this.outtake = outtake;
    }
    public class UpdateSystems implements Action {

        public UpdateSystems() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elevator.setPidControl();
            claw.update();
            outtake.update();
            robot.drive.updatePoseEstimate();

            return true;
        }
    }

    public Action updateSystems()
    {
        return new UpdateSystems();
    }
}
