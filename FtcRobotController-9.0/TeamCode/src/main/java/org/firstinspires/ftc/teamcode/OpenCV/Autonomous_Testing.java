package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Locale;

@Autonomous(name="Robot: Autonomous_Testing", group="Robot")
public class Autonomous_Testing extends LinearOpMode{

    static final double INCREMENT   = 0.02;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    DistanceSensor sensorDistance;
    TouchSensor touchSensor;
    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics; UNITS ARE PIXELS; NOTE: this calibration is for the C920 webcam at 800x448;You will need to do your own calibration for other configurations!
    double fx = 822.317; double fy = 822.317; double cx = 319.495; double cy = 242.502;
    // UNITS ARE METERS
    double tagsize = 0.166;
    //Declaring the Tag IDS we used in this program
    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;
    boolean tagFound = false;

    boolean aligned = false;

    @Override
    public void runOpMode()
    {
        servo = hardwareMap.get(Servo.class, "left_hand");
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
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
            public void onError(int errorCode) { }
        });
        telemetry.setMsTransmissionInterval(50);

        //The INIT-loop:This REPLACES waitForStart!
        while (!isStarted() && !isStopRequested())
        {
            tagInformation();
            if(tagFound)
            {
                telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
                tagToTelemetry(tagOfInterest);
                //Function to turn continuous till x-axis lies between -0.6 to 0
            }
            telemetry.update();
            sleep(20);
        }
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        /* Update the telemetry */
        /* Actually do something useful */
        if( tagOfInterest.id == left)
        {
            //left code
            telemetry.addLine("Left April Tag Going Towards it");
            telemetry.update();
            //go Towards the left tag
            goTowardsTag(left);
        }
        else if ( tagOfInterest.id == middle)
        {
            //middle code
            telemetry.addLine("Middle April Tag");
            telemetry.update();
            //go Towards the middle tag
            goTowardsTag(middle);
        }
        else if (tagOfInterest.id == right){
            //right code
            telemetry.addLine("Right April Tag");
            telemetry.update();
            //go Towards the right tag
            goTowardsTag(right);
        }
    }

    public void tagInformation(){
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == left || tag.id == middle || tag.id == right)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }
    public void goTowardsTag(int tag){
        if(tag == left){
            encoderDrive(DRIVE_SPEED,  -5.6,  -5.6, 5.0);
            encoderDrive(TURN_SPEED,   -1.4, 1.4, 4.0);
            encoderDrive(DRIVE_SPEED,-1.3,-1.3,3);
            sleep(2000);
            dropPixel();
            sleep(3000);

            encoderDrive(DRIVE_SPEED, -19, -19, 15);

            encoderDrive(DRIVE_SPEED, -1.4, 1.4, 3);
            encoderDrive(DRIVE_SPEED, -3, -3, 4);
        } else if (tag==middle) {
            encoderDrive(DRIVE_SPEED,  0.2,  -0.2, 5.0);
            encoderDrive(TURN_SPEED,   -8.2, -8.2, 7.0);
            sleep(2000);
            dropPixel();
            sleep(3000);
            encoderDrive(DRIVE_SPEED,-4.2,-4.2,2);
            encoderDrive(DRIVE_SPEED,-1.6,1.6,3);

            encoderDrive(DRIVE_SPEED, -21, -21, 20);

            encoderDrive(DRIVE_SPEED, -1.6, 1.6, 3);
            encoderDrive(DRIVE_SPEED, -3, -3, 10);
        } else if (tag==right) {
            encoderDrive(DRIVE_SPEED,  -5.6,  -5.6, 5.0);
            encoderDrive(TURN_SPEED,   1.4, -1.4, 4.0);
            encoderDrive(DRIVE_SPEED,-1.3,-1.3,3);
            sleep(2000);
            dropPixel();
            sleep(2000);
            encoderDrive(DRIVE_SPEED,1.3,1.3,3);
            encoderDrive(DRIVE_SPEED, -1.4, 1.4, 3);
            encoderDrive(DRIVE_SPEED,-5.6,-5.6,5);
            encoderDrive(DRIVE_SPEED, -1.4, 1.4, 3);
            encoderDrive(DRIVE_SPEED,-21,-21,20);

        }
    }
    public void dropPixel(){
            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            // Set the servo to the new position and pause;
            servo.setPosition(position);
            sleep(CYCLE_MS);
            idle();
    }
    //Use at the time of Parking
    public double getDistance(){
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        double distance = sensorDistance.getDistance(DistanceUnit.INCH);
        return distance;
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the OpMode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addLine("Robot is in motion");
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}


