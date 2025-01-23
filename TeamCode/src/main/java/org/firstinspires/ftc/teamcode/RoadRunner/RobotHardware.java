package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.Angle;
import org.firstinspires.ftc.teamcode.RoadRunner.util.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.RoadRunner.util.wrappers.BetterSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class RobotHardware {

    //drivetrain
    public HardwareMap hardwareMap;
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;
    // elevator

    // intake
    public BetterServo intakeClawLeftServo;
    public BetterServo intakeClawRightServo;
    public BetterServo intakeAngleServo;
    public BetterServo intakeHandPivotServo;
    public RevColorSensorV3 colorRight;
    public RevColorSensorV3 colorLeft;

    // outake
    public BetterServo outtakeClawLeftServo;
    public BetterServo outtakeClawRightServo;
    public BetterServo outtakeClawPivotServo;
    public BetterServo outtakeHandLeftServo;
    public BetterServo outtakeHandRightServo;
    public BetterServo outtakeSpinServo;


    public BetterServo planeServo;

    public MecanumDrive drive;

    // TODO: ADD x3 Distance Sensors, webcam

    // Telemetry storage
    public Telemetry telemetry;
    private double voltage = 0.0;
    private ElapsedTime voltageTimer;


    // singleton go brrrr
    private static RobotHardware instance = null;
    public boolean enabled;

    public IMU imu;

    private ArrayList<BetterSubsystem> subsystems;

    private double imuAngle, imuOffset = 0;


    boolean has2Pixels = false, closeRight = false, closeLeft = false;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpModeBlue.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry, Pose2d pose , IMU imu)  {
        this.hardwareMap = hardwareMap;

        drive = new MecanumDrive(hardwareMap, pose);
        voltageTimer = new ElapsedTime();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.subsystems = new ArrayList<>();

        this.imu = drive.imu.get();

        // DRIVETRAIN
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "mBL");
        this.dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "mFL");
        this.dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "mBR");
        this.dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "mFR");
        this.dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // INTAKE
        this.intakeAngleServo = new BetterServo(hardwareMap.get(Servo.class, "sIA"));
        intakeAngleServo.setDirection(Servo.Direction.REVERSE);
        this.intakeClawLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sICL"));
        this.intakeClawRightServo = new BetterServo(hardwareMap.get(Servo.class, "sICR"));
        this.intakeClawRightServo.setDirection(Servo.Direction.REVERSE);
        // COLOR/DS SENSORS
        this.colorRight = hardwareMap.get(RevColorSensorV3.class, "cR");
        this.colorLeft = hardwareMap.get(RevColorSensorV3.class, "cL");

        // HAND
        this.intakeHandPivotServo = new BetterServo(hardwareMap.get(Servo.class, "sIHPR"));

        // OUTTAKE
        // CLAW
        this.outtakeClawLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sCL"));
        this.outtakeClawRightServo = new BetterServo(hardwareMap.get(Servo.class, "sCR"));
        this.outtakeClawPivotServo = new BetterServo(hardwareMap.get(Servo.class, "sC"));
        this.outtakeClawPivotServo.setDirection(Servo.Direction.REVERSE);
        this.outtakeClawRightServo.setDirection(Servo.Direction.REVERSE);
        // HAND
        this.outtakeHandLeftServo = new BetterServo(hardwareMap.get(Servo.class, "sHL"));
        this.outtakeHandRightServo = new BetterServo(hardwareMap.get(Servo.class, "sHR"));
        this.outtakeHandRightServo.setDirection(Servo.Direction.REVERSE);
        this.outtakeSpinServo = new BetterServo(hardwareMap.get(Servo.class, "sSO"));

        this.planeServo = new BetterServo(hardwareMap.get(Servo.class, "sP"));

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry)
    {
        init(hardwareMap, telemetry, new Pose2d(0,0,0));
    }


    public void loopVoltage(HardwareMap hardwareMap)
    {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
    }

    public double getVoltage() {
        return voltage;
    }

    public void read() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }


    public void periodic() {
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void reset() {
        for (BetterSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void addSubsystem(BetterSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public void setExternalHeading(double value) {
    }

    public double getAngle() {
        return Angle.norm(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + imuOffset);
    }

    public void setImuOffset(double offset)
    {
        this.imuOffset = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + offset;
    }

    public double getImuOffset()
    {
        return imuOffset;
    }

    public boolean has2Pixels() {
        // check for colors and closed
        return has2Pixels;
    }

    public void setHas2Pixels(boolean has2Pixels) {
        this.has2Pixels = has2Pixels;
    }

    public boolean isCloseRight() {
        // check for colors and closed
        return closeRight;
    }

    public void closeRight(boolean closeRight) {
        this.closeRight = closeRight;
    }

    public boolean isCloseLeft() {
        // check for colors and closed
        return closeLeft;
    }

    public void closeLeft(boolean closeLeft) {
        this.closeLeft = closeLeft;
    }
}
