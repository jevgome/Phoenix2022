package org.firstinspires.ftc.teamcode.auton.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.*;

@Disabled
@Autonomous(group = "auto")
public class RRTest extends LinearOpMode {
    DcMotorEx arm;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("clawServo");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35,-62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,0))
                .build();

        closeClaw();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(left);
        }
    }

    public void closeClaw() {
        claw.setPosition(0);

        telemetry.addData("Claw","closing");
        telemetry.update();
    }
    public void openClaw() {
        claw.setPosition(0.5);

        telemetry.addData("Claw","Opening");
        telemetry.update();
    }
    public void raiseArm(int pos, double velocity) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);

        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }
}