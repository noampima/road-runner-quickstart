package org.firstinspires.ftc.teamcode.RoadRunner.auto.Actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.RobotHardware;
import org.firstinspires.ftc.teamcode.RoadRunner.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.RoadRunner.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.RoadRunner.util.ClawSide;

public class SubsystemActions {


    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;


    private DepositActions depositActions;

    private UpdateActions updateActions;

    public SequentialAction intake5CloseAction, intake43OpenAction, transfer, depositAction, readyForDepositAction, deposit43Action;
    public ParallelAction placePreloadAndIntakeAction;
    int tempHeight = 1250;




    public SubsystemActions(DepositActions depositActions, UpdateActions updateActions) {


        this.depositActions = depositActions;

        this.updateActions = updateActions;






        depositAction = new SequentialAction(
                depositActions.placePixel(),
                new SleepAction(.5),
                depositActions.moveElevator(tempHeight + 400),
                depositActions.retractDeposit()
        );


        readyForDepositAction = new SequentialAction(
                new SleepAction(.75),
                depositActions.readyForDeposit(tempHeight));


        deposit43Action = new SequentialAction(
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),
                new SleepAction(0.6),
                depositActions.placePixel(),
                new SleepAction(0.25),
                depositActions.moveElevator(tempHeight + 300),
                depositActions.retractDeposit());

    }


}
