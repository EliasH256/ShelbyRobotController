package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_NUM_LEVS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_SPD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_SPD_DWN;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EX_MAX;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EX_MIN;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.WR_SENSE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumBot extends ShelbyBot
{
    private static final String TAG = "SJH_MEC";

    //public Claw claw = null;
    //Still need this for now for backward compatibility
    public MotorComponent elev;

    public MotorComponent elbowMotor = null;
    public MotorComponent extenderMotor = null;
    public MotorComponent bucketServo = null;
    public ServoComponent wristServo = null;
    public ServoComponent droneLauncherServo = null;
    public MotorComponent sweeperServo1 = null;
    public MotorComponent sweeperServo2 = null;
    public Intake intake = null;
    public boolean extenderStopped = true;

    public double logIntakeCurSpd = 0.0;

    public MecanumBot()
    {
        super();

        name= "B7252";
        bulkCachingMode =  LynxModule.BulkCachingMode.MANUAL;

        DRIVE_GEARS =
          new double[]{RobotConstants.DT_MOTOR.getGear(), RobotConstants.DT_EXT_GEAR_RATIO};

        WHEEL_DIAMETER_INCHES = RobotConstants.DT_WHEEL_DIAM;

        gyroInverted = false;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor)
    {
        RobotLog.dd(TAG, "MecanumBot init " + name);
        initCore(op);

        this.initDirSensor = initDirSensor;
        initSensors();
        initDriveMotors();
        initJointControl();
        initCapabilities();
    }

    @Override
    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, TAG + "Initializing drive motors");
        drive = new MecanumDriveLRR(imu);
        //Roadrunner mech drive starts with FL motor and goes counterclockwise (FL,BL,BR,FR)
        try
        {
            List<DcMotorEx> mtrs = ((MecanumDriveLRR)drive).getMotors();
            motors.put("FL", mtrs.get(0));
            motors.put("BL", mtrs.get(1));
            motors.put("BR", mtrs.get(2));
            motors.put("FR", mtrs.get(3));

            capMap.put("drivetrain", true);
//            CommonUtil.getInstance().getDashboard().displayText(6, "TRMB_MOTORS: " + mtrs.size());
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get motors in MecanumBot\n" + e.toString());
        }
    }

    private DigitalChannel redLED1;
    private DigitalChannel greenLED1;
    private DigitalChannel redLED2;
    private DigitalChannel greenLED2;



   public int pixelPieces = 0;

    @Override
    protected void initJointControl()
    {
        RobotLog.dd(TAG, "init Lifter MecanumBot\n");

        elbowMotor = new MotorComponent("elbow", hwMap);
        extenderMotor = new MotorComponent("extend", hwMap);
        bucketServo = new MotorComponent("bucket", hwMap);
        wristServo = new ServoComponent("wrist", hwMap);
        droneLauncherServo = new ServoComponent("droneLauncher", hwMap);
        sweeperServo1 = new MotorComponent("sweeper1", hwMap);
        sweeperServo2 = new MotorComponent("sweeper2", hwMap);
        intake = new Intake(hwMap);

        redLED1 = hwMap.get(DigitalChannel.class, "red1");
        greenLED1 = hwMap.get(DigitalChannel.class, "green1");
        redLED2 = hwMap.get(DigitalChannel.class, "red2");
        greenLED2 = hwMap.get(DigitalChannel.class, "green2");
        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

        elbowMotor.init(RobotConstants.EL_EXT_MOT, -2);
        elbowMotor.setDir(RobotConstants.EL_DIR);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setLevelOffset(
            RobotConstants.EL_LEVS[0],
            RobotConstants.EL_LEVS[1],
            RobotConstants.EL_LEVS[2],
            RobotConstants.EL_LEVS[3],
            RobotConstants.EL_LEVS[4],
            RobotConstants.EL_LEVS[5]);
        armLevel =0;

        extenderMotor.init(RobotConstants.EL_EX_MOT, 4.72);
        extenderMotor.setDir(RobotConstants.EXT_DIR);
        extenderMotor.setMode(STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extenderMotor.setLevelOffset(
            RobotConstants.EX_LEVS[0],
            RobotConstants.EX_LEVS[1]);

        sweeperServo1.init(RobotConstants.SWP_SRV);
        sweeperServo1.setDir(RobotConstants.SWP_DIR);

        sweeperServo2.init(RobotConstants.SWP_SRV);
        sweeperServo2.setDir(RobotConstants.SWP_DIR2);

        bucketServo.init(RobotConstants.SWP_SRV);
        bucketServo.setDir(RobotConstants.SWP_DIR2);
        bucketServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        wristServo.init(0,.82);
        wristServo.moveAt(1);

        droneLauncherServo.init(0,.60);
        droneLauncherServo.moveAt(1.0);
        droneLauncherServo.moveTo(0.25);
        if (droneLauncherServo != null)
        {
            RobotLog.dd(TAG,"drone launcher was init");
        }
        intake.init();


        bucketServoOn = servoState.STOPPED;
        intakesOn = false;
    }

    private final ElapsedTime updTimer = new ElapsedTime();
    public void update()
    {
        updTimer.reset();
        super.update();
        double botTime = updTimer.milliseconds();
        updTimer.reset();
        if(elbowMotor != null)
        {
            elbowMotor.update();
        }
        if(extenderMotor != null)
        {
            extenderMotor.update();
        }
        double lftTime = updTimer.milliseconds();
        updTimer.reset();

        updTimer.reset();

        if(VERBOSE)
        {
            RobotLog.dd(TAG, "UPD BOT:%.2f LFT:%.2f CLW:%.2f",
                        botTime, lftTime);
        }
        ledUpdate();
    }
    public void ledUpdate()
    {
        switch (pixelPieces)
		{
            case 1:
                greenLED1.setState(false);
                redLED1.setState(false);
                greenLED2.setState(false);
                redLED2.setState(false);
                break;
            case 3:
                greenLED1.setState(false);
                redLED1.setState(true);
                greenLED2.setState(false);
                redLED2.setState(true);
                break;
            case 2:
                greenLED1.setState(true);
                redLED1.setState(false);
                greenLED2.setState(true);
                redLED2.setState(false);
                break;
            case 0:
                greenLED1.setState(true);
                redLED1.setState(true);
                greenLED2.setState(true);
                redLED2.setState(true);
                break;


        }
    }

   public void initElbMot () throws InterruptedException {
        elbowMotor.moveToCnt(1000,.18);
        Thread.sleep(3000);
        elbowMotor.setMode(STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(RUN_USING_ENCODER);
        elbowMotor.moveToLevel(0,EL_SPD);
    }
    public void initExMot () throws InterruptedException {
        extenderMotor.moveToCnt(-2500,.3);
        Thread.sleep(4000);
        extenderMotor.setMode(STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(RUN_USING_ENCODER);
        extenderMotor.moveToCnt(0, 0.1);
    }

    public void setElbowMotor(double input){
        if(Math.abs( input) > 0.125) {
            elbowMotor.moveDist(input*20,  input);
           // elbowMotor.moveTo();
        }
        else {
            elbowMotor.stop();
        }
    }


    public void setWristServo(double input){
        if(Math.abs( input) > 0.125) {
            wristServo.moveAmount(-input * WR_SENSE );
        }
        else {

        }
    }

    public void setExtenderPower(double input){
        if(input >= .1 && extenderMotor.getCurEnc() < EX_MAX || input <= -.1 && extenderMotor.getCurEnc() > EX_MIN)
        {
            extenderMotor.moveAtControlRate(input);
            extenderStopped = false;
        }
        else
        {
            if (!extenderStopped)
            {
                extenderStopped = true;
                extenderMotor.moveAtControlRate(0);
            }
        }
    }

    private enum servoState{
        STOPPED,
        FORWARD,
        BACKWARD
    }

    public boolean purplePixelDrop = false;

    private servoState bucketServoOn;

    public void toggleBucketServoForward(){
        if(bucketServoOn == servoState.FORWARD)
            setBucketServo(servoState.STOPPED);
        else if(purplePixelDrop){
            setBucketServo(servoState.FORWARD);
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    setBucketServo(servoState.STOPPED);
                }
            };
            Timer timer = new Timer();
            timer.schedule(task, 650);
            purplePixelDrop = false;
        }
        else{
            setBucketServo(servoState.FORWARD);
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    setBucketServo(servoState.STOPPED);
                }
            };
            Timer timer = new Timer();
            timer.schedule(task, 465);
        }

    }
    public void outTwoPixels()
    {
        setBucketServo(servoState.FORWARD);
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                setBucketServo(servoState.STOPPED);
            }
        };
        Timer timer = new Timer();
        timer.schedule(task, 800);

    }

    public void toggleBucketServoBackward() {
        if(bucketServoOn == servoState.BACKWARD)
            setBucketServo(servoState.STOPPED);
        else
            setBucketServo(servoState.BACKWARD);
    }
public  int armLevel;
    public void armLevelUp(){
        if(armLevel != EL_NUM_LEVS){
            armLevel ++;
            elbowMotor.moveToLevel(armLevel, 1);
        }
    }

    public void armLevelDown() {
        if(armLevel != 0){
            armLevel--;
            elbowMotor.moveToLevel(armLevel, -1);
        }
    }

    public void setBucketServo(servoState state){
        RobotLog.dd(TAG, "in setBucketServo: %s", state);
        bucketServoOn = state;
        switch (state){
            case STOPPED:
                bucketServo.moveAtRate(0);
                break;
            case FORWARD:
                bucketServo.moveAtRate(1);
                break;
            case BACKWARD:
                bucketServo.moveAtRate(-1);
                break;

        }
    }

    public void extendr(){
        extenderMotor.moveToLevel(1,1);
    }

    public void destendr(){
        extenderMotor.moveToLevel(0,1);
    }

    public double getElSpd(int targetEncoder){
        if(targetEncoder > elbowMotor.getCurEnc()){
            return EL_SPD_DWN;
        }else {
            return EL_SPD;
        }
    }
    public boolean intakesOn;

    public void intakesOff(){
        if(intakesOn){
            toggleIntakes();
        }
    }

    public void toggleIntakes()
    {
        intakesOn = !intakesOn;

        if(intakesOn)
        {
            sweeperServo1.moveAtRate(0.3);
            sweeperServo2.moveAtRate(0.3);
            intake.setPwr(1.0);
            setBucketServo(servoState.BACKWARD);
        }
        else
        {
            sweeperServo1.moveAtRate(0);
            sweeperServo2.moveAtRate(0);
            intake.stop();
            setBucketServo(servoState.STOPPED);
        }
    }
}
