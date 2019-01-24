package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class Meet3AutonomousDepotHardCoded extends LinearOpMode{
    HardwareRobot robot;
    private ElapsedTime runtime  = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXykNg//////AAABmdVQDVwq9kAEsspJU9r8u8VmSeZBFzHTHr6fsWSjYKVlHaAw6uE0fxEJ0zCNaIbGmpOSWf0NY/pFNh4N5uYYtL99ymMWhR2tfuIBXgo7T4m8ht7lStZtjHjmcmO0nQBzzGCm74gw+CDYvRbfDYtr95fNuoMIcyZUiv2TpUcsbebE+fT6HEfyGXyF1j4d6CEzWc1Qhdy+nCCC3kO/5oDt8usf3ryOzBgFW/l4l+YEqk1LVw1vrx4+DhiqQ87ohJDybGab6FvqxC2Hlryx0p7BdmwCtQqfaRD8s8icv7XUR09Xlij02Z5iRe/7+aJc44fxmu3xTB17y3r8Er0YmqVvU3EChH5p0+SiHl+z36p78c1J";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    Orientation             lastAngles = new Orientation();


    /*static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);*/
   // static final double     DRIVE_SPEED             = 0.6;
   // static final double     TURN_SPEED              = 0.5;
    double globalAngle, power = .30, correction;
    public void runOpMode() {
        robot = new HardwareRobot();
        robot.init(hardwareMap);
        initIMU();
        initVuforia();
        robot.markerDropper.setPosition(.3);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        robot.linearActuator.setPower(1);
        sleep(3500);
        StopDriving();
        /*ShiftRight(.2);
        sleep(500);
        StopDriving();
        DriveForward(.5);
        sleep(2000);
        StopDriving();
        TurnLeft(.5,-.5);
        sleep(2000);
        StopDriving();
        DriveForward(.5);
        sleep(2000);
        StopDriving();*/
//delatch, get off
       // encoderDrive(DRIVE_SPEED,  16,   5.0);  // S1: Forward 47 Inches with 5 Sec timeout

//delatch with encoder
        //move backa  but
        //move right
        //move back more
        //move back to center
        //look for mineral
        //hit mineral go to certain place
        //turn 45 detree
        //drive
        //place marker
        //turn
        //go to crater

        if (opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }

                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.update();

                                    ShiftRight(.5);
                                    sleep(800);
                                    StopDriving();

                                    DriveForward(.4);
                                    sleep(800);
                                    StopDriving();

                                    ShiftRight(.4);
                                    sleep(600);
                                    ShiftLeft(.4);
                                    sleep(700);
                                    StopDriving();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.update();

                                    ShiftRight(.5);
                                    sleep(800);
                                    StopDriving();
                                    DriveForward(-.4);
                                    sleep(800);
                                    StopDriving();

                                    ShiftRight(-.4);
                                    sleep(500);
                                    StopDriving();
                                    ShiftLeft(.4);
                                    sleep(700);
                                    StopDriving();
                                    DriveBackward(.6);
                                    sleep(500);
                                    StopDriving();

                                } else {

                                    telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.update();
                                    ShiftRight(.5);
                                    sleep(800);
                                    StopDriving();
                                    ShiftRight(.4);
                                    sleep(800);
                                    StopDriving();
                                }
                            }

                        }
                        telemetry.update();
                    }
                }
            }
            //
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        //gyro turn

    }

    public void waiting(long millis){
        long t = System.currentTimeMillis();
        while(opModeIsActive()&&System.currentTimeMillis()-t<millis){

        }
    }

    public void initIMU(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


        robot.imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.
}

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    double DRIVE_POWER = 1.0;
    public void TurnLeft(double lPower, double rPower){
        robot.leftWheel.setPower(lPower);
        robot.rightWheel.setPower(rPower);

    }
    public void StopMoving (){
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        robot.centerWheel.setPower(0);
    }
    public void TurnRight(double lPower, double rPower){
        robot.leftWheel.setPower(lPower);
        robot.rightWheel.setPower(rPower);

    }

    public void DriveForward(double power){
        robot.leftWheel.setPower(power);
        robot.rightWheel.setPower(power);

    }
    public void DriveBackward(double power){
        robot.leftWheel.setPower(-power);
        robot.rightWheel.setPower(-power);

    }
    public void ShiftLeft(double power){
        robot.centerWheel.setPower(power);
    }
    public void ShiftRight(double power){
        robot.centerWheel.setPower(-power);
    }
    public void StopDriving(){
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        robot.centerWheel.setPower(0);

    }
    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
        robot.leftWheel.setPower(leftPower);
        robot.rightWheel.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.rightWheel.setPower(0);
        robot.leftWheel.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    /*public void encoderDrive(double speed,
                             double inches,
                             double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.linearActuator.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.leftWheel.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.linearActuator.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.linearActuator.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.linearActuator.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.linearActuator.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }*/
}



