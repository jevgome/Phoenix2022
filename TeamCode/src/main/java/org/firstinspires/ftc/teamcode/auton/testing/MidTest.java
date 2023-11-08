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
public class MidTest extends LinearOpMode {
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35,-62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,0))
                .lineToConstantHeading(new Vector2d(35,-12))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> raiseArm(3320,10000))
                .turn(Math.toRadians(39))
                .forward(16)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(2520,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3320,10000))
                .back(10)
                .turn(Math.toRadians(-127))
                .forward(26.5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> raiseArm(600,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> raiseArm(3320,10000))
                .back(40)
                .turn(Math.toRadians(53))
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(2520,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3320,10000))
                .back(9)
                .turn(Math.toRadians(-53))

                .forward(49)
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> raiseArm(475,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-45))
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(2)
                .turn(Math.toRadians(45))
                .back(20)
                .build();

        TrajectorySequence mid = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,0))
                .lineToConstantHeading(new Vector2d(35,-12))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> raiseArm(3320,10000))
                .turn(Math.toRadians(39))
                .forward(16)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(2520,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3320,10000))
                .back(10)
                .turn(Math.toRadians(-127))
                .forward(26.5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> raiseArm(600,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> raiseArm(3320,10000))
                .back(40)
                .turn(Math.toRadians(53))
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(2520,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3320,10000))
                .back(9)
                .turn(Math.toRadians(-53))

                .forward(49)
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> raiseArm(475,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-45))
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(2)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,0))
                .lineToConstantHeading(new Vector2d(35,-12))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> raiseArm(3320,10000))
                .turn(Math.toRadians(39))
                .forward(16)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(2520,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3320,10000))
                .back(10)
                .turn(Math.toRadians(-127))
                .forward(26.5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> raiseArm(600,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> raiseArm(3320,10000))
                .back(40)
                .turn(Math.toRadians(53))
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(2520,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(3320,10000))
                .back(9)
                .turn(Math.toRadians(-53))

                .forward(49)
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> raiseArm(475,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-45))
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(2)
                .turn(Math.toRadians(45))
                .forward(22)
                .build();

        //22

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