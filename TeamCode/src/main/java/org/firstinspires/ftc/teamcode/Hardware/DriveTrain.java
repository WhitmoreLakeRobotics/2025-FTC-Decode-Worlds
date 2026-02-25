package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.firstinspires.ftc.teamcode.Common.Settings;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class DriveTrain extends BaseHardware {

    //public DistanceSensor WallEway;  //auton wall distance sensor;
    //public DistanceSensor HumptyDumpty;  // there is another;

    private DcMotor LDM1 ;
    private DcMotor LDM2 ;
    private DcMotor RDM1 ;
    private DcMotor RDM2 ;

    public CommonGyro Gyro = new CommonGyro();
    // private Vision vision = new Vision();
    /*
     * Hardware Mappings
     */
    // public HardwareMap hardwareMap = null; // will be set in Child class

    public Mode Current_Mode;

    private boolean cmdComplete = true;

    private static double TURNSPEED_TELEOP = 0.80;

    private double LDM1Power;
    private double LDM2Power;
    private double RDM1Power;
    private double RDM2Power;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    private static final String TAGChassis = "8492 ";

    public static final double DTrain_NORMALSPEED = 65; //was 65 z=1.00;
    public static final double DTrain_SLOWSPEED = 0.35; // was 30;
    public static final double DTrain_FASTSPEED = 1.00; //was 85;
    private double SensorDrive = 0.35;

    private double Drive_Start;  //in inches
    private double Drive_Target;  //in inches
    private static final double Distance_Per_Rev = 4.09*3.14159;
    private static final double Gear_Ratio = 13.7;
    private  static final int Gyro_Tol  = 1; //was 3
    private static final double Ticks_Per_Inch = (Settings.GOBILDA_MOTOR_TICKS_PER_REV *  Gear_Ratio) / Distance_Per_Rev;
    private double bearing_AA = 0;
    private double speed_AA = 0;
    private int Target_Heading;
    private static final double driveTolAA = 0.25; //in inches
    private static final double diaTurnRaid = 12; //in inches //was 18.25
    private static final double turnDistPerDeg = ((3.14159 * diaTurnRaid)/360) * Ticks_Per_Inch; //inches per deg
    private static final double stagPos = 40;
    private static final double stagPow = 0.18;
    private final long visionThreshHold = 1000;
    private double sensorRange = 4000.0;
    private double sensorRangeLeftFront = 4000.0;
    private double sensorRangeRightFront = 4000.0;
    private double sensorRangeLeftSide = 4000.0;
    private double sensorRangeRightSide = 4000.0;
    private double sensorRangeRear = 4000.0;
    private final double sensorTol = 0.25;
    private SensorSel sensorSelection = SensorSel.UNKNOWN;
    private double lastTurnPower = 0;   // MJD

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {

        Gyro.telemetry = telemetry;
        Gyro.hardwareMap = hardwareMap;
        Gyro.init();

        // vision.telemetry = telemetry;
        // vision.hardwareMap = hardwareMap;
        // vision.init();

        RDM1 = hardwareMap.dcMotor.get("RDM1");
        LDM1 = hardwareMap.dcMotor.get("LDM1");
        LDM2 = hardwareMap.dcMotor.get("LDM2");
        RDM2 = hardwareMap.dcMotor.get("RDM2");

        if (LDM1 == null) {
            telemetry.log().add("LDM1 is null...");
        }
        if (LDM2 == null) {
            telemetry.log().add("LDM2 is null...");
        }
        if (RDM1 == null) {
            telemetry.log().add("RDM1 is null...");
        }
        if (RDM2 == null) {
            telemetry.log().add("RDM2 is null...");
        }

        LDM1.setDirection(DcMotor.Direction.REVERSE);
        LDM2.setDirection(DcMotor.Direction.FORWARD);
        RDM1.setDirection(DcMotor.Direction.FORWARD);
        RDM2.setDirection(DcMotor.Direction.REVERSE);

        LDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Drive Train", "Initialized");
        Current_Mode = Mode.STOPPED;
    }

    public void init_loop() {}

    public void start() {}

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop() {

        switch(Current_Mode){
            case TELEOP:
                break;

            case STOPPED:
                stopMotors();
                cmdComplete = true;
                break;

            case COMMAND_AA:
                break;

            case DRIVE_AA:
                doDrive();
                break;

            case VISION:
                aprilDrive();
                break;

            case DRIVE_BY_SENSOR:
                doDriveBySensor(sensorSelection);
                break;

            case COMMAND_TURN:
                doTurn();
                break;
        }
    }
    public void stop(){
        Current_Mode = Mode.STOPPED;
        cmdComplete = true;
        telemetry.addData(TAGChassis,"distance driven " + getPosInches());
        stopMotors();
    }

    public void cmdTeleOp(double Left_Y, double Left_X, double Right_X, double Current_Speed) {
        cmdComplete = false;
        Current_Mode = Mode.TELEOP;
        double Drive = Left_Y * Current_Speed;
        double Strafe = Left_X * Current_Speed;
        double Turn = Right_X * TURNSPEED_TELEOP;
        double Heading = Gyro.getGyroHeadingRadian();

        double NDrive = Strafe * Math.sin(Heading) + Drive * Math.cos(Heading);
        double NStrafe = Strafe * Math.cos(Heading) - Drive * Math.sin(Heading);

        LDM1Power = NDrive + NStrafe + Turn;
        RDM1Power = NDrive - NStrafe - Turn;
        LDM2Power = NDrive - NStrafe + Turn;
        RDM2Power = NDrive + NStrafe - Turn;

        doTeleop();
    }

    public double autoTurn(int newHeading){
        return calcTurn(newHeading);
    }

    public void visDrive(double Left_Y, double Left_X, double Right_X, double Current_Speed) {
        cmdComplete = false;
        Current_Mode = Mode.TELEOP;
        double Drive = Left_Y * Current_Speed;
        double Strafe = Left_X * Current_Speed;
        double Turn = Right_X * (1.0 -Drive) * TURNSPEED_TELEOP ;
        double Heading = Gyro.getGyroHeadingRadian();

        LDM1Power = Drive + Strafe + Turn;
        RDM1Power = Drive - Strafe - Turn;
        LDM2Power = Drive - Strafe + Turn;
        RDM2Power = Drive + Strafe - Turn;

        doTeleop();
    }

    public void doTeleop() {
        Current_Mode = Mode.TELEOP;

        double LDM1P = CommonLogic.CapValue(LDM1Power, minPower, maxPower);
        double LDM2P = CommonLogic.CapValue(LDM2Power, minPower, maxPower);
        double RDM1P = CommonLogic.CapValue(RDM1Power, minPower, maxPower);
        double RDM2P = CommonLogic.CapValue(RDM2Power, minPower, maxPower);

        LDM1.setPower(LDM1P);
        RDM1.setPower(RDM1P);
        LDM2.setPower(LDM2P);
        RDM2.setPower(RDM2P);
    }

    private void scaleMotorPower(){
        double MaxValue = 0;
        if (Math.abs(LDM2Power)  > MaxValue) MaxValue = LDM2Power;
        if (Math.abs(RDM1Power)  > MaxValue) MaxValue = RDM1Power;
        if (Math.abs(RDM2Power)  > MaxValue) MaxValue = RDM2Power;
        if (Math.abs(LDM1Power)  > MaxValue) MaxValue = LDM1Power;

        if (maxPower > 1) {
            scalePower(MaxValue);
        }
    }

    public void updateRange(double leftFrontRange,double rightFrontRange, double leftSideRange,
                            double rightSideRange , double rearRange){
        sensorRange = (leftFrontRange + rightFrontRange + leftSideRange + rightSideRange)/4;
        sensorRangeLeftFront = leftFrontRange;
        sensorRangeRightFront = rightFrontRange;
        sensorRangeLeftSide = leftSideRange;
        sensorRangeRightSide = rightSideRange;
        sensorRangeRear = rearRange;
    }

    private void scalePower(double ScalePower){
        LDM1Power=LDM1Power/ScalePower;
        LDM2Power=LDM2Power/ScalePower;
        RDM1Power=RDM1Power/ScalePower;
        RDM2Power=RDM2Power/ScalePower;
    }

    private void updateMotorPower(){
        scaleMotorPower();

        LDM1Power = CommonLogic.CapValue(LDM1Power, Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);
        LDM2Power = CommonLogic.CapValue(LDM2Power, Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);
        RDM1Power = CommonLogic.CapValue(RDM1Power, Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);
        RDM2Power = CommonLogic.CapValue(RDM2Power, Settings.REV_MIN_POWER, Settings.REV_MAX_POWER);

        LDM1.setPower(LDM1Power);
        LDM2.setPower(LDM2Power);
        RDM1.setPower(RDM1Power);
        RDM2.setPower(RDM2Power);
    }

    public void CmdDrive(double TargetDist,double Bearing, double speed, int Heading){
        Target_Heading = Heading;

        Drive_Target = TargetDist * Ticks_Per_Inch;

        resetEncoders();

        bearing_AA = Bearing;
        speed_AA = speed;

        cmdComplete = false;
        Current_Mode = Mode.DRIVE_AA;
        startDrive();
    }

    public double calcTurn(int tHeading){

        double current = Gyro.getGyroHeading();
        double error = tHeading - current;

        error = ((error + 540) % 360) - 180;

        if (Math.abs(error) < 3.0) {
            lastTurnPower = 0;
            return 0;
        }

        double turn = CommonLogic.PIDcalcTurn(stagPos, 0.04, current, tHeading);

        turn = 0.55 * turn + 0.45 * lastTurnPower;
        lastTurnPower = turn;

        telemetry.addData(TAGChassis,"turn power " + turn);
        return turn;
    }

    public void cmdTurn(int newHeading, double speed){
        speed_AA = 0.0;
        bearing_AA = 0.0;
        Target_Heading = newHeading;
        resetEncoders();
        cmdComplete = false;
        Current_Mode = Mode.COMMAND_TURN;
    }

    public void doTurn(){

        if ( CommonLogic.inRange(Gyro.getGyroHeading(),Target_Heading,Gyro_Tol)  ){
            stopMotors();
            Current_Mode = Mode.STOPPED;
        } else {
            startDrive();
        }
    }
    public void cmdDriveBySensors(double TargetDist,double Bearing, double speed, int Heading){
        cmdDriveBySensors(TargetDist,Bearing,speed,Heading, SensorSel.LEFT_SIDE);
    }

    public void cmdDriveBySensors(double TargetDist,double Bearing, double speed, int Heading,SensorSel sel){
        sensorSelection = sel;

        Target_Heading = Heading;

        Drive_Target = (TargetDist);

        resetEncoders();

        bearing_AA = Bearing;
        speed_AA = speed;
        SensorDrive = speed;

        cmdComplete = false;
        Current_Mode = Mode.DRIVE_BY_SENSOR;
        startDrive();
    }

    private void startDrive(){
        double Left_Y = Math.cos(Math.toRadians(bearing_AA));
        double Drive = Left_Y * speed_AA;
        double Strafe = Math.sin(Math.toRadians(bearing_AA)) * speed_AA;
        double Turn = calcTurn(Target_Heading) * (0.8 -Drive) * TURNSPEED_TELEOP ;
        double Heading = Gyro.getGyroHeadingRadian();

        double NDrive = Strafe * Math.sin(Heading) + Drive * Math.cos(Heading);
        double NStrafe = Strafe * Math.cos(Heading) - Drive * Math.sin(Heading);

        LDM1Power = NDrive + NStrafe + Turn;
        RDM1Power = NDrive - NStrafe - Turn;
        LDM2Power = NDrive - NStrafe + Turn;
        RDM2Power = NDrive + NStrafe - Turn;

        scaleMotorPower();

        LDM1.setPower(LDM1Power);
        RDM1.setPower(RDM1Power);
        LDM2.setPower(LDM2Power);
        RDM2.setPower(RDM2Power);
    }

    private void doDrive(){
        double distance = getPosInTicks();

        if(Drive_Target <= distance) {
            stopMotors();
            cmdComplete = true;
            Current_Mode = Mode.STOPPED;
        }
        else {
            telemetry.addData(TAGChassis,"distance driven " + getPosInches());
            startDrive();
        }
    }

    private void doDriveBySensor(SensorSel sel){
        speed_AA = (CommonLogic.PIDcalc(6.5, 0,GetSensorRange(sel),Drive_Target));

        startDrive();

        if(CommonLogic.inRange(GetSensorRange(sel),Drive_Target,sensorTol)) {
            stopMotors();
            cmdComplete = true;
            Current_Mode = Mode.STOPPED;
        }
    }

    private double GetSensorRange(SensorSel sel){
        double range = 0.0;
        switch (sel){
            case BOTH:
                range = Math.min(sensorRangeRightFront,sensorRangeLeftFront);
                break;
            case RIGHT_FRONT:
                break;
            case LEFT_FRONT:
                range = sensorRangeLeftFront;
                break;
            case LEFT_SIDE:
                break;
            case RIGHT_SIDE:
                range = sensorRangeRightSide;
                break;
            case REAR:
                range = sensorRangeRear;
                break;
            default:
                range = sensorRange;
                break;
        }
        return range;
    }

    private void resetEncoders(){
        LDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getPosInches(){
        double values = Math.abs(LDM1.getCurrentPosition());
        values += Math.abs(LDM2.getCurrentPosition());
        values += Math.abs(RDM1.getCurrentPosition());
        values += Math.abs(RDM2.getCurrentPosition());
        values = values/4;

        values = values/Ticks_Per_Inch;

        return values;
    }

    private double getPosInTicks(){
        double values = Math.abs(LDM1.getCurrentPosition());
        values += Math.abs(LDM2.getCurrentPosition());
        values += Math.abs(RDM1.getCurrentPosition());
        values += Math.abs(RDM2.getCurrentPosition());
        values = values/4;

        return values;
    }
    public void QuickAligenment() {
        //The intention of this method is to return a turn value based upon a desired alingment direction
        //this should override the right joystick
    }
    private void stopMotors(){
        LDM1.setPower(0);
        LDM2.setPower(0);
        RDM1.setPower(0);
        RDM2.setPower(0);
    }

    public void ResetGyro(){
        Gyro.imu.resetYaw();
    }

    public boolean getCmdComplete(){
        return cmdComplete;
    }

    public boolean isBusy() {
        return cmdComplete;
    }

    public void aprilDrive(){
        /*if (vision.getDesiredTag_staleTime_mSec() < visionThreshHold){
        }
        */
    }

    public enum SensorSel{
        RIGHT_FRONT,
        LEFT_FRONT,
        RIGHT_SIDE,
        LEFT_SIDE,
        BOTH,
        REAR,
        UNKNOWN;
    }

    public enum Mode{
        DRIVE_AA,
        COMMAND_AA,
        COMMAND_TURN,
        STOPPED,
        TELEOP,
        DRIVE_BY_SENSOR,
        VISION;
    }
    // Returns robot heading in DEGREES, CCW positive, normalized to 0–360  // MJD
    public double getCurrentHeadingDeg() {                                  // MJD
        double heading = Gyro.getGyroHeading();      // IMU yaw in degrees   // MJD
        heading = (heading % 360 + 360) % 360;       // normalize 0–360      // MJD
        return heading;                                                     // MJD
    }

    // Legacy method preserved for compatibility, now wraps the new one      // MJD
    public int getCurrentHeading() {                                        // MJD
        return (int) getCurrentHeadingDeg();                                // MJD
    }

}
