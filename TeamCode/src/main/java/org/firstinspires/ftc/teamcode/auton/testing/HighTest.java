package org.firstinspires.ftc.teamcode.auton.testing;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SleeveDetection;
import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.DriveConstants;
import org.opencv.core.Mat;
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

@Autonomous(group = "auto")
public class HighTest extends LinearOpMode {
    DcMotorEx arm;
    Servo claw;

    private SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("clawServo");

        boolean two = false;
        boolean three = false;

        TrajectoryVelocityConstraint slowT = SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowA = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
        double slowAV = DriveConstants.MAX_ANG_VEL/2;
        double slowAA = DriveConstants.MAX_ANG_ACCEL/2;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35,-62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3300,10000))
                .strafeLeft(24,slowT,
                        slowA)
                .forward(50,slowT, slowA)
                .strafeRight(12,slowT, slowA)
                .forward(9.5,slowT,slowA)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3200,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3,() -> openClaw())
                .waitSeconds(0.7)
                .back(8,slowT,slowA)
                .turn(Math.toRadians(-90),slowAV,slowAA)
                .forward(36,slowT,slowA)
                .UNSTABLE_addTemporalMarkerOffset(-2,() -> raiseArm(700,10000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(21,slowT,slowA)
                .turn(Math.toRadians(-42),slowAV,slowAA)
                .forward(3,slowT, slowA)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3,slowT,slowA)
                .turn(Math.toRadians(42),slowAV,slowAA)
                .back(20,slowT,slowA)
                .build();

        TrajectorySequence mid = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3300,10000))
                .strafeLeft(24,slowT,
                        slowA)
                .forward(50,slowT, slowA)
                .strafeRight(12,slowT, slowA)
                .forward(9.5,slowT,slowA)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3200,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3,() -> openClaw())
                .waitSeconds(0.7)
                .back(8,slowT,slowA)
                .turn(Math.toRadians(-90),slowAV,slowAA)
                .forward(36,slowT,slowA)
                .UNSTABLE_addTemporalMarkerOffset(-2,() -> raiseArm(700,10000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(21,slowT,slowA)
                .turn(Math.toRadians(-42),slowAV,slowAA)
                .forward(3,slowT, slowA)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3,slowT,slowA)
                .turn(Math.toRadians(42),slowAV,slowAA)
                .build();


        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3300,10000))
                .strafeLeft(24,slowT,
                        slowA)
                .forward(50,slowT, slowA)
                .strafeRight(12,slowT, slowA)
                .forward(9.5,slowT,slowA)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3200,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3,() -> openClaw())
                .waitSeconds(0.7)
                .back(8,slowT,slowA)
                .turn(Math.toRadians(-90),slowAV,slowAA)
                .forward(36,slowT,slowA)
                .UNSTABLE_addTemporalMarkerOffset(-2,() -> raiseArm(700,10000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7,() -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(21,slowT,slowA)
                .turn(Math.toRadians(-42),slowAV,slowAA)
                .forward(3,slowT, slowA)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3,slowT,slowA)
                .turn(Math.toRadians(42),slowAV,slowAA)
                .forward(26,slowT,slowA)
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        closeClaw();
        while(!isStarted()) {
            two = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER;
            three = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT;
            telemetry.addData("Position", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(two ? mid : three ? right : left);
            telemetry.addData("Park",sleeveDetection.getPosition());
            telemetry.update();

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