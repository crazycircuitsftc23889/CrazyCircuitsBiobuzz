package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.annotation.TargetApi;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="BlueTestTeleOp")
public class BlueTeleop extends OpMode {
    private MecanumDrive drive;
    private NormalizedColorSensor colorSensor;
    private Limelight3A limelight;
    public DcMotor intakeMotor = null;
    public DcMotorEx shooterMotor = null;
    public DcMotor transferMotor = null;
    public Servo intakeServo1 = null;
    public Servo intakeServo2 = null;
    public Servo compressionHood1 = null;
    public Servo compressionHood2 = null;
    public Servo turret = null;
    private double normRed;
    private double normBlue;
    private double normGreen;
    private PIDController pidController;

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
        colorSensor.setGain(12);
        limelight.pipelineSwitch(9);
        limelight.start();
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        intakeServo1 = hardwareMap.get(Servo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(Servo.class, "intakeServo2");
        compressionHood1 = hardwareMap.get(Servo.class, "compressionHood1");
        compressionHood2 = hardwareMap.get(Servo.class, "compressionHood2");
        turret = hardwareMap.get(Servo.class, "turret");

        //setting servo power
        intakeServo1.setPosition(0.0);
        intakeServo2.setPosition(0.0);
        compressionHood1.setPosition(0.0);
        compressionHood2.setPosition(0.0);
        turret.setPosition(0.5);


    }



    @Override
    public void loop() {
        drive.updatePoseEstimate();
        Pose2d currentPose = drive.localizer.getPose();

        //Moving Robot Around
        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(
                        -gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x)
        );

        LLResult result = limelight.getLatestResult();
        double tX = result.getTx();
        double tY = result.getTy();
        double tA = result.getTa();

        double currentVelocity = shooterMotor.getVelocity();
        double kF = 0.00042;
        double MAX_POWER = 0.9;
        double targetVelocity = 0;
        double feedforward = targetVelocity * kF;
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setCurrentAlert(8.0, CurrentUnit.AMPS);

        double pidCorrection = pidController.calculate(currentVelocity, targetVelocity);

        // Step 3: Combine them
        double totalPower = feedforward + pidCorrection;

        // Step 4: Clamp to safe range
        totalPower = Range.clip(totalPower, 0, MAX_POWER);


        //intake control
        if (gamepad2.right_trigger > 0) {
            intakeMotor.setPower(-1);
        } else if (gamepad2.left_trigger > 0) {
            intakeMotor.setPower(1); //reverse intake
        } else {
            intakeMotor.setPower(0);
        }

        //shooter control
        if (gamepad2.a) {
            shooterMotor.setPower(-1);
        } else {
            shooterMotor.setPower(0);
        }

        //transfer control
        if (gamepad2.right_bumper) {
            transferMotor.setPower(-1);
        } else if (gamepad2.left_bumper) {
            transferMotor.setPower(1); //reverse transfer
        } else {
            transferMotor.setPower(0);
        }
        shooterMotor.setPower(totalPower);

        //flower intake control
        if (gamepad2.b) {
            intakeServo1.setPosition(0.4);
            intakeServo2.setPosition(0.4);
        } else {
            intakeServo1.setPosition(0.0);
            intakeServo2.setPosition(0.0);
        }

        //adjustable compression hood control
        DetectedColor color = detectColors();
        if (color == DetectedColor.BLUE) {
            compressionHood1.setPosition(0.5);
            compressionHood2.setPosition(0.5);
        } else {
            compressionHood2.setPosition(0.0);
            compressionHood1.setPosition(0.0);
        }
        //Prototype later, adjust delay for hood extension

        //turret control
        //Code when we get gobilda pinpoint




        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        detectColors();

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        telemetry.addData("x", tX);
        telemetry.addData("y", tY);
        telemetry.addData("a", tA);
    }
    public DetectedColor detectColors() {
        if( normRed > 0.1 && normGreen < normRed && normBlue < normRed ) {
            return DetectedColor.RED;
        }
        else if( normGreen > 0.2 && normGreen > normRed && normBlue < normGreen) {
            return DetectedColor.YELLOW;
        }
        else if (normRed < normBlue && normGreen < normBlue && normBlue > 0.1) {
            return DetectedColor.BLUE;
        }
        return DetectedColor.UNKNOWN;
    }
}