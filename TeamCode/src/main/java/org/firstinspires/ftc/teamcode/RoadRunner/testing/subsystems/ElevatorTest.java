package org.firstinspires.ftc.teamcode.RoadRunner.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.RobotHardware;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BetterGamepad;

@Config
@TeleOp(name = "Elevator Test", group = "A")
public class ElevatorTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Elevator elevator;
    BetterGamepad gamepadEx;
    public static double target = 0;

    @Override
    public void runOpMode() {
        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        elevator = new Elevator(gamepad1, true, true);
        elevator.setAuto(false);

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();

            if(gamepad1.right_stick_y != 0)
            {
                elevator.setUsePID(false);
            }
            else
            {
                elevator.setUsePID(true);
            }

            elevator.setTarget(target);

            elevator.update();

            telemetry.addData("right motor power", elevator.getControllerR().update());
            telemetry.addData("left motor power", elevator.getControllerL().update());
            telemetry.update();
        }
    }

}