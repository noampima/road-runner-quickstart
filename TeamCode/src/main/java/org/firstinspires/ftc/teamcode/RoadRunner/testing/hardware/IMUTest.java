package org.firstinspires.ftc.teamcode.RoadRunner.testing.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.RobotHardware;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BetterGamepad;

@Config
@TeleOp(name = "IMU Test", group = "tests")
@Disabled
public class IMUTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("IMU", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
    }

}
