package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class NewAutonomousRed extends LinearOpMode{
    HardwareRobot robot;
    private ElapsedTime runtime  = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXykNg//////AAABmdVQDVwq9kAEsspJU9r8u8VmSeZBFzHTHr6fsWSjYKVlHaAw6uE0fxEJ0zCNaIbGmpOSWf0NY/pFNh4N5uYYtL99ymMWhR2tfuIBXgo7T4m8ht7lStZtjHjmcmO0nQBzzGCm74gw+CDYvRbfDYtr95fNuoMIcyZUiv2TpUcsbebE+fT6HEfyGXyF1j4d6CEzWc1Qhdy+nCCC3kO/5oDt8usf3ryOzBgFW/l4l+YEqk1LVw1vrx4+DhiqQ87ohJDybGab6FvqxC2Hlryx0p7BdmwCtQqfaRD8s8icv7XUR09Xlij02Z5iRe/7+aJc44fxmu3xTB17y3r8Er0YmqVvU3EChH5p0+SiHl+z36p78c1J";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;



    double globalAngle, power = .30, correction;
    public void runOpMode() {
        robot = new HardwareRobot();
        robot.init(hardwareMap);
        initIMU();
        robot.linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        robot.linearActuator.setTargetPosition(-11730);
        robot.linearActuator.setPower(.6);
        while (opModeIsActive() && robot.linearActuator.isBusy())
        {
            telemetry.addData("LINEAR ACTUATOR ENCODER", robot.linearActuator.getCurrentPosition() + "  busy=" + robot.linearActuator.isBusy());
            telemetry.update();
            idle();
        }

        robot.linearActuator.setPower(0.0);
        DriveForward(.5);
        sleep(500);
        DriveForward(0);
        sleep(100);
        robot.centerWheel.setTargetPosition(2500);
        robot.centerWheel.setPower(.5);
        robot.centerWheel.setPower(0.0);
        DriveBackward(.5);
        sleep(500);
        DriveBackward(0);
        sleep(100);

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
////////
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
}



