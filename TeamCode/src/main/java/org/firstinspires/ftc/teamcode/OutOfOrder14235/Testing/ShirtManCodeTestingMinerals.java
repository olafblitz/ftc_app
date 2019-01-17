

package org.firstinspires.ftc.teamcode.OutOfOrder14235.Testing;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous

public class ShirtManCodeTestingMinerals extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Vuforia variables
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // DogeCV detector
    GoldAlignDetector detector;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor centerWheel;
    private DcMotor linearActuator;
    private Servo markerDropper;
    private DcMotor linExt;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;
boolean initLoop = false;
    //GoldAlignDetector
    //private GoldAlignDetector detector;


    @Override
    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Set up parameters for Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key
        parameters.vuforiaLicenseKey = "AXykNg//////AAABmdVQDVwq9kAEsspJU9r8u8VmSeZBFzHTHr6fsWSjYKVlHaAw6uE0fxEJ0zCNaIbGmpOSWf0NY/pFNh4N5uYYtL99ymMWhR2tfuIBXgo7T4m8ht7lStZtjHjmcmO0nQBzzGCm74gw+CDYvRbfDYtr95fNuoMIcyZUiv2TpUcsbebE+fT6HEfyGXyF1j4d6CEzWc1Qhdy+nCCC3kO/5oDt8usf3ryOzBgFW/l4l+YEqk1LVw1vrx4+DhiqQ87ohJDybGab6FvqxC2Hlryx0p7BdmwCtQqfaRD8s8icv7XUR09Xlij02Z5iRe/7+aJc44fxmu3xTB17y3r8Er0YmqVvU3EChH5p0+SiHl+z36p78c1J\n";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();





        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;
        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        // Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
        // Set up detector
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();

        parametersIMU.mode                = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        leftWheel = hardwareMap.get(DcMotor.class, "left_drive");
        rightWheel = hardwareMap.get(DcMotor.class, "right_drive");
        centerWheel = hardwareMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        linearActuator = hardwareMap.get(DcMotor.class, "wormGear");
        linExt = hardwareMap.get(DcMotor.class, "linExt");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.initialize(parametersIMU);

        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

    }

    @Override
    public void init_loop() {
        initLoop = true;
    }

    @Override
    public void loop(){
        initLoop = false;

        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
        
    }
    @Override
    public void start(){

        runtime.reset();
        rotate(90,.4);

    }
    @Override
    public void stop() {
        // Disable the detector
        vuforia.stop();

        detector.disable();
    }
    double DRIVE_POWER = 1.0;
    public void TurnLeft(double lPower, double rPower){
        leftWheel.setPower(lPower);
        rightWheel.setPower(rPower);

    }
    public void TurnRight(double lPower, double rPower){
        leftWheel.setPower(lPower);
        rightWheel.setPower(rPower);

    }

    public void DriveForward(double power){
        leftWheel.setPower(power);
        rightWheel.setPower(power);

    }
    public void DriveBackward(double power){
        leftWheel.setPower(-power);
        rightWheel.setPower(-power);

    }
    public void ShiftLeft(double power){
        centerWheel.setPower(power);
    }
    public void ShiftRight(double power){
        centerWheel.setPower(-power);
    }
    public void StopDriving(){
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        centerWheel.setPower(0);

    }
    public static void sleep(long sleepTime)
    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
                sleepTime = wakeupTime - System.currentTimeMillis();
            }
        }
    }   //sleep
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        leftWheel.setPower(leftPower);
        rightWheel.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (initLoop && getAngle() == 0) {msStuckDetectInitLoop =  5000 + (int)System.currentTimeMillis();}

            while (initLoop && getAngle() > degrees) {msStuckDetectInitLoop =  5000 + (int)System.currentTimeMillis();}
        }
        else    // left turn.
            while (initLoop && getAngle() < degrees) {msStuckDetectInitLoop =  5000 + (int)System.currentTimeMillis();}


        // turn the motors off.
        rightWheel.setPower(0);
        leftWheel.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
