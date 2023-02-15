package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;

@Autonomous(name = "RightSide", group = "Concept")
public class RightSide extends LinearOpMode{
    //movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per rotation for the GoBilda 5202 PLanetary Motor
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int targetPosition = 1;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double distanceInches = 1;
        double placeHolderDistance = 1;
        // servo position
        drive.clawServo.setPosition(0.5);
        sleep(2000);


        Pose2d startingPose = new Pose2d(35,-70,90);
        drive.setPoseEstimate(startingPose);

        // all of our trajectories
        // drive forward
        TrajectorySequence beginning = drive.trajectorySequenceBuilder(startingPose)
                .forward(48.5,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .UNSTABLE_addDisplacementMarkerOffset(-30, () -> {
                    targetPosition = 2800;
                })
                .addTemporalMarker(() -> {
                    drive.liftMotor.setPower(0.05);
                })
                .strafeLeft(30)
                .build();
        drive.followTrajectorySequenceAsync(beginning);

        while (!isStarted() && !isStopRequested()){

        }

        waitForStart();


        /*

        TrajectorySequence backToPole = drive.trajectorySequenceBuilder(startingPose)
                .forward(60)
                .UNSTABLE_addDisplacementMarkerOffset(-35, () -> {
                    drive.liftMotor.setPower(.8);
                })
                .addTemporalMarker(() -> {
                    drive.liftMotor.setPower(0.07);
                })
                .build();
        */

        // april tags
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        // april tag initialization
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0){
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.update();
            sleep(20);
        }

         */
        int numberLoop = 1;
        while (opModeIsActive()){
            distanceInches = drive.distSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance Inches", distanceInches);
            telemetry.update();

            if (distanceInches < 10 && numberLoop == 1) {
                sleep(50);
                drive.setMotorPower(0);
                placeHolderDistance = distanceInches;
                drive.breakFollowing();
                startingPose = new Pose2d(0, 0, 0);
                drive.setPoseEstimate(startingPose);
                TrajectorySequence distanceSensor = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            targetPosition = 2900;
                            drive.liftMotor.setPower(0.05);
                        })

                        .forward(placeHolderDistance - 3.5)
                        .addTemporalMarker(() -> {
                            drive.clawServo.setPosition(0.3);
                        })
                        .build();
                drive.followTrajectorySequenceAsync(distanceSensor);

                numberLoop++;
            }
            if (numberLoop == 2 && !drive.isBusy()){
                startingPose = new Pose2d(0, 0, Math.toRadians(90));
                drive.setPoseEstimate(startingPose);
                TrajectorySequence goToCone = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            drive.liftMotor.setPower(-0.2);
                        })
                        .lineToLinearHeading(new Pose2d(12, -10, Math.toRadians(90)))
                        .addTemporalMarker(() -> {
                            targetPosition = 350;
                            drive.liftMotor.setPower(.05);
                        })

                        /*.splineToLinearHeading(new Pose2d(25, 1.79, Math.toRadians(0.00)), Math.toRadians(11.46))
                        .waitSeconds(0.1)
                        .addTemporalMarker(() -> {
                            drive.clawServo.setPosition(0.5);
                        })
                        */

                        .build();
                drive.followTrajectorySequenceAsync(goToCone);
                numberLoop++;
            }
            drive.update();
            liftUpdate(drive);
        }
    }

    public void liftUpdate(SampleMecanumDrive drive) {
        int currentPosition = drive.liftMotor.getCurrentPosition();
        if (Math.abs(currentPosition - targetPosition) > 6) {
            currentPosition = drive.liftMotor.getCurrentPosition();
            double power = returnPower(targetPosition, drive.liftMotor.getCurrentPosition());
            drive.liftMotor.setPower(power);
            telemetry.addData("current position", currentPosition);
            telemetry.addData("targetPosition", targetPosition);
            telemetry.update();
        }
    }

    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
