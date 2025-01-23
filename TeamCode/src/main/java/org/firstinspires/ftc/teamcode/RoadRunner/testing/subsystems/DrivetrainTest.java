package org.firstinspires.ftc.teamcode.RoadRunner.testing.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.RobotHardware;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BetterGamepad;

@Config
@TeleOp(name = "Drivetrain Test", group = "A")
public class DrivetrainTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain;

    BetterGamepad gamepadEx;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        drivetrain = new Drivetrain(gamepad1, true, true);

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();


            drivetrain.update();
            telemetry.update();
        }
    }

}
