package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class Intake
{
    public Intake(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            intaker = hwMap.get(DcMotorEx.class, "intake");
            intaker.setDirection(RobotConstants.IN_DIR);
            intaker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intaker.setPower(0);
            lastIntakePwr = 0.0;
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake\n" + e.toString());
        }

        try
        {
            intakeSrvo = hwMap.get(CRServo.class, "intake");
            intakeSrvo.setDirection(RobotConstants.IN_DIR);
            intakeSrvo.setPower(0.0);
            lastIntakePwr = 0.0;
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake\n" + e.toString());
        }

        return success;
    }

    public String toString(){
        return String.format(Locale.US,
                "intake %5d %4.2f %4.2f",
                encPos, curSpd, curPwr);
    }

    public void update()
    {
        if(intaker != null)
        {
            encPos = intaker.getCurrentPosition();
            curSpd = intaker.getVelocity();
            curPwr = intaker.getPower();
        }
        else if(intakeSrvo != null)
        {
            curPwr = intakeSrvo.getPower();
        }
    }

    public void stop()
    {
        setPwr(0.0);
    }

    public void setPwr(double pwr)
    {
        if(intaker != null && pwr != lastIntakePwr)
        {
            intaker.setPower(pwr);
            lastIntakePwr = pwr;
        }
        else if(intakeSrvo != null && pwr != lastIntakePwr)
        {
            intakeSrvo.setPower(pwr);
            lastIntakePwr = pwr;
        }
    }

    private DcMotorEx intaker;
    private CRServo intakeSrvo;

    protected HardwareMap hwMap;
    private static final String TAG = "SJH_INT";
    private int encPos = 0;
    private double curSpd = 0.0;
    private double curPwr = 0.0;
    private double lastIntakePwr = 0.0;
}
