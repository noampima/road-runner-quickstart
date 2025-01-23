package org.firstinspires.ftc.teamcode.RoadRunner.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.RobotHardware;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Claw;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RoadRunner.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.RoadRunner.util.ClawSide;

@Config
@TeleOp(name = "Outtake Test", group = "A")
public class OuttakeTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    BetterGamepad gamepadEx;
    Outtake outtake;
    Claw claw;

    public static Outtake.Angle angle = Outtake.Angle.INTAKE;
    public static boolean DEBUG = true;


    @Override
    public void run() {
        outtake.update();

        if(!DEBUG)
        {
            if(gamepad1.right_bumper)
            {
                claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
                outtake.setAngle(Outtake.Angle.OUTTAKE);
            }
            else if(gamepad1.left_bumper)
            {
                outtake.setAngle(Outtake.Angle.INTAKE);
                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            }
        }
        else
        {
            outtake.setAngle(angle);
        }

        telemetry.update();
    }

    @Override
    public void initialize() {
        gamepadEx = new BetterGamepad(gamepad1);
        robot.init(hardwareMap, telemetry);

        outtake = new Outtake();
        claw = new Claw();
    }

}