package org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
import org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous.HardwareRobot;

import java.util.List;
import java.util.Locale;

@Autonomous
public class LeagueChampsAutonomousCraterSampleOnLanderMRGYRO extends LinearOpMode{
    HardwareRobot robot;
    ModernRoboticsI2cGyro gyro;                    // Additional Gyro device


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    enum MineralPosition
    {
        LEFT, CENTER, RIGHT;
    }
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

    MineralPosition position;
    public void runOpMode() {
        robot = new HardwareRobot();
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imuBase");

        robot.linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro =  hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Done.");
        telemetry.update();

        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        gyro.resetZAxisIntegrator();
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
                                    position = LeagueChampsAutonomousCraterSampleOnLanderMRGYRO.MineralPosition.LEFT;
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.update();
                                    break;


                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    position = LeagueChampsAutonomousCraterSampleOnLanderMRGYRO.MineralPosition.RIGHT;

                                    telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.update();
                                    break;


                                } else  if (goldMineralX > silverMineral1X && goldMineralX < silverMineral2X || goldMineralX < silverMineral1X && goldMineralX > silverMineral2X ) {
                                    position = LeagueChampsAutonomousCraterSampleOnLanderMRGYRO.MineralPosition.CENTER;

                                    telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.update();
                                    break;

                                }
                                else{
                                    Meet3AutonomousDepotHardCodedIMUSampleOnLander.MineralPosition position = Meet3AutonomousDepotHardCodedIMUSampleOnLander.MineralPosition.LEFT;
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
        ShiftRight(.6);
        sleep(200);
        StopDriving();
        DriveForward(.7);
        sleep(1450);
        StopDriving();
        ShiftRight(.7);
        sleep(800);
        StopDriving();
        DriveBackward(.7);
        sleep(1400);
        StopDriving();
        telemetry.addData("Checkpoint Reached!!", 1);
        telemetry.update();

        if(position == LeagueChampsAutonomousCraterSampleOnLanderMRGYRO.MineralPosition.LEFT ){

            ShiftRight(.7);
            sleep(1000);
            StopDriving();
            DriveForward(1);
            sleep(1400);
            StopDriving();

            ShiftRight(.7);
            sleep(2000);
            StopDriving();

            gyroTurn(.7,-35);
            StopDriving();

            ShiftRight(.7);
            sleep(2300);
            StopDriving();

            robot.markerDropper.setPosition(1);
            sleep(100);
            ShiftLeft(.6);
            sleep(200);
            StopDriving();
            gyroTurn(.4,12);
            StopDriving();

            DriveForward(1);
            robot.linExt.setPower(.4);
            sleep(4000);
            robot.linExt.setPower(0);
            robot.flipper.setPower(.7);
            sleep(450);
            StopDriving();

        }
        else if(position == LeagueChampsAutonomousCraterSampleOnLanderMRGYRO.MineralPosition.RIGHT){

            ShiftRight(.5);
            sleep(1750);
            StopDriving();
            DriveForward(-1);
            sleep(1400);
            StopDriving();
            ShiftRight(1);
            sleep(1800);
            StopDriving();
            gyroTurn(.5,32);
            StopDriving();
            ShiftRight(1);
            sleep(1100);
            StopDriving();
            robot.markerDropper.setPosition(1);
            ShiftLeft(.5);
            sleep(200);
            StopDriving();
            DriveForward(1);
            robot.linExt.setPower(.4);

            sleep(3600);
            StopDriving();
            robot.linExt.setPower(0);
            robot.flipper.setPower(1);
            sleep(300);
            StopDriving();

        }
        else if(position == LeagueChampsAutonomousCraterSampleOnLanderMRGYRO.MineralPosition.CENTER){
            ShiftRight(1);
            sleep(3200);
            StopDriving();
            robot.markerDropper.setPosition(1);
            //robot.leftWheel.setPower(-.3);
            //robot.rightWheel.setPower(.3);
            gyroTurn(.5,32);
            StopDriving();
            DriveForward(1);
            robot.linExt.setPower(.4);
            sleep(3300);
            StopDriving();
            robot.linExt.setPower(0);
            robot.flipper.setPower(1);
            sleep(300);
            StopDriving();

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


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftWheel.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftWheel.setTargetPosition(newLeftTarget);
            robot.rightWheel.setTargetPosition(newRightTarget);

            robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftWheel.setPower(speed);
            robot.rightWheel.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftWheel.isBusy() && robot.rightWheel.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftWheel.setPower(leftSpeed);
                robot.rightWheel.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftWheel.getCurrentPosition(),
                        robot.rightWheel.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftWheel.setPower(0);
            robot.rightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     **/
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftWheel.setPower(leftSpeed);
        robot.rightWheel.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}



