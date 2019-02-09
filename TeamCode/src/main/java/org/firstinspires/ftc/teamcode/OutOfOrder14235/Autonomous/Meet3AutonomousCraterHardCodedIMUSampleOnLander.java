package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;
@Disabled

@Autonomous
public class Meet3AutonomousCraterHardCodedIMUSampleOnLander extends LinearOpMode{
    HardwareRobot robot;
    private ElapsedTime runtime  = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXykNg//////AAABmdVQDVwq9kAEsspJU9r8u8VmSeZBFzHTHr6fsWSjYKVlHaAw6uE0fxEJ0zCNaIbGmpOSWf0NY/pFNh4N5uYYtL99ymMWhR2tfuIBXgo7T4m8ht7lStZtjHjmcmO0nQBzzGCm74gw+CDYvRbfDYtr95fNuoMIcyZUiv2TpUcsbebE+fT6HEfyGXyF1j4d6CEzWc1Qhdy+nCCC3kO/5oDt8usf3ryOzBgFW/l4l+YEqk1LVw1vrx4+DhiqQ87ohJDybGab6FvqxC2Hlryx0p7BdmwCtQqfaRD8s8icv7XUR09Xlij02Z5iRe/7+aJc44fxmu3xTB17y3r8Er0YmqVvU3EChH5p0+SiHl+z36p78c1J";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    BNO055IMU               imu;
    private Orientation angles;
    private Acceleration gravity;
    enum MineralPosition
    {
        LEFT, CENTER, RIGHT;
    }
    MineralPosition position;
    public void runOpMode() {
        robot = new HardwareRobot();
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imuBase");

        robot.linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        initVuforia();
        robot.markerDropper.setPosition(.3);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else telemetry.addData("Sorry!", "This device is not compatible with TFOD");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();






        telemetry.addData("Mode", "waiting for start DO NOT START UNTIL READY TO GO MSG");
        telemetry.update();
        telemetry.addLine("AUTONOMOUS READY TO GO");
        telemetry.update();
        waitForStart();

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        sendTelemetry();
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
                                     position = MineralPosition.LEFT;
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.update();
                                    break;


                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                     position = MineralPosition.RIGHT;

                                    telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.update();
                                    break;


                                } else  if (goldMineralX > silverMineral1X && goldMineralX < silverMineral2X || goldMineralX < silverMineral1X && goldMineralX > silverMineral2X ) {
                                     position = MineralPosition.CENTER;

                                    telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.update();
                                    break;

                                }
                                else{
                                    MineralPosition position = MineralPosition.LEFT;
                                    telemetry.addData("Gold Mineral Position", "NOT DETECTED: GUESSING LEFT");
                                    telemetry.update();
                                    break;

                                }
                            }

                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        robot.linearActuator.setTargetPosition(-11680);
        robot.linearActuator.setPower(.8);

        while (opModeIsActive() && robot.linearActuator.isBusy())
        {
            telemetry.addData("LINEAR ACTUATOR ENCODER", robot.linearActuator.getCurrentPosition() + "  busy=" + robot.linearActuator.isBusy());
            telemetry.update();
            idle();
        }

        robot.linearActuator.setPower(0.0);
        sleep(600);
        while(robot.distanceBottom.getDistance(DistanceUnit.CM) >= 8.5 ){

                robot.linearActuator.setPower(.3);

        }
        robot.linearActuator.setPower(0.0);
        sleep(400);
        telemetry.addData("Delatched!", 1);
        telemetry.update();
        ShiftRight(.5);
        sleep(300);
        StopDriving();
        DriveForward(.5);
            sleep(650);
            StopDriving();
        ShiftRight(.5);
            sleep(960);
            StopDriving();
        DriveBackward(.4);
            sleep(1150);
            StopDriving();
        telemetry.addData("Checkpoint Reached!!", 1);
        telemetry.update();

        if(position == MineralPosition.LEFT ){

            ShiftRight(.5);
            sleep(1500);
            StopDriving();
            DriveForward(.5);
            sleep(1100);
            StopDriving();

            ShiftRight(1);
            sleep(1380);
            StopDriving();
            ShiftRight(-1);
            sleep(1000);
            StopDriving();
            DriveForward(.5);
            sleep(6000);
            StopDriving();
            robot.markerDropper.setPosition(.8);
            sleep(100);



        }
        else if(position == MineralPosition.RIGHT){

            ShiftRight(.5);
            sleep(1750);
            StopDriving();
            DriveForward(-.8);
            sleep(950);
            StopDriving();
            ShiftRight(1);
            sleep(1670);
            StopDriving();
            ShiftRight(-1);
            sleep(1000);
            StopDriving();
            DriveForward(.5);
            sleep(8000);
            StopDriving();
            robot.markerDropper.setPosition(.8);
            sleep(100);





        }
        else if(position == MineralPosition.CENTER){
            ShiftRight(.7);
            sleep(3650);
            StopDriving();
            ShiftRight(-1);
            sleep(870);
            StopDriving();
            DriveForward(.5);
            sleep(7000);
            StopDriving();
            robot.markerDropper.setPosition(.8);
            sleep(100);



        }


    }

    public void waiting(long millis){
        long t = System.currentTimeMillis();
        while(opModeIsActive()&&System.currentTimeMillis()-t<millis){

        }
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
    void sendTelemetry()
    {
        telemetry.addData("Status", imu.getSystemStatus().toString());
        telemetry.addData("Calib", imu.getCalibrationStatus().toString());
        telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("Roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("Pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addData("Grav", gravity.toString());
        telemetry.update();
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



}



