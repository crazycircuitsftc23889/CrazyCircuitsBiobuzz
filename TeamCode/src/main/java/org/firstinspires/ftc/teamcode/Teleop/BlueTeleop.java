package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.annotation.TargetApi;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name="BlueTestTeleOp")
public class BlueTeleop extends OpMode {
    private MecanumDrive drive;
    private NormalizedColorSensor colorSensor;
    private Limelight3A limelight;

    private double normRed;
    private double normBlue;
    private double normGreen;

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
        else if( normGreen > 0.1 && normGreen > normRed && normBlue < normGreen) {
            return DetectedColor.YELLOW;
        }
        else if (normRed < normBlue && normGreen < normBlue && normBlue > 0.1) {
            return DetectedColor.BLUE;
        }
        return DetectedColor.UNKNOWN;
    }
}