package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.util.AxesSigns;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class RobotConstants
{
  //Indicates whether to use InitTrajectories, InitTrajectories2, or Both for the Auton route
  public static TrajEnum trajType = TrajEnum.INIT_TRAJ_2;

  //params map is WIP
  public static Map<String, Double> params = new HashMap<>();

  //ColorSensor on drivetrain
  public static double CL_VALUE_THRESH = 0.45;

  public static double DT_XOFFSET = 0.0;

  //Spinner (duck wheel)
  public static double SP_POWER = 0.0;
  public static double SP_TURBOPWR = 0.8;
  public static double SP_TIMEOUT = 2.5;
  public static double SP_STEP = 0.005;
  public static DcMotorSimple.Direction SP_RED_DIR = DcMotorSimple.Direction.REVERSE;
  public static DcMotorSimple.Direction SP_BLU_DIR = DcMotorSimple.Direction.FORWARD;

  //Drop (dumper)
  public static double DP_TIMEOUT = 1.0;
  public static double DP_DUMP_STOW = 0.0;
  public static double DP_DUMP_DEPLOY = 1.0;
  public static double DP_DUMP_DEPLOY_AUTO = 1.0;
  public static double DP_RATE = 1.0;
  public static double DP_PREWAIT_H = 0.0;
  public static double DP_PREWAIT_M = 0.0;
  public static Servo.Direction DP_DIR = Servo.Direction.FORWARD;

  //Intake
  public static double IN_TIMEOUT = 1.5;
  public static double IN_AUTO_PWR = 1.0;
  public static double IN_AUTO_PWR_OUT = 1.0;
  public static double IN_TELE_PWR = 1.0;
  public static double IN_ARM_EXT_STOW = 0.0;
  public static double IN_ARM_EXT_IN = 0.0;
  public static double IN_ARM_EXT_DEPLOY = 0.2;
  public static DcMotorSimple.Direction IN_DIR = DcMotorSimple.Direction.REVERSE;
  public static Motors.MotorModel IN_MOTOR_MOT = Motors.MotorModel.GOBILDA_5202_71_2;
  public static double IN_GEAR = 1.0;
  public static double IN_ACT_SPD = 0.0;
  public static double IN_ACT_DLY = 0.0;
  public static double IN_ACT_DUR = 0.5;
  public static double IN_ACT_DIST = 3.0;

  //Elev

  public static int      EL_NUM_LEVS=3;
  public static int      EX_NUM_LEVS=2;
  public static double   EL_MAX_IPS = 1.0;
  public static double   EL_SPD = 1.0;
  public static double   EL_SPD_DWN = .3333333;
  public static double[] EL_LEVS;
  public static double[] EX_LEVS;
  public static int EL_MAX_ENCODER;
  public static int EL_MIN_ENCODER = 10;
  public static int   EX_MAX = 2850;
  public static int   EX_MIN = 3;
  public static DcMotorSimple.Direction EL_DIR = DcMotorSimple.Direction.REVERSE;

  public static DcMotorSimple.Direction EXT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction SWP_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction SWP_DIR2 = DcMotorSimple.Direction.FORWARD;
  public static Motors.MotorModel EL_EXT_MOT = Motors.MotorModel.GOBILDA_5202_139;
  public static Motors.MotorModel EL_EX_MOT = Motors.MotorModel.GOBILDA_5202_5_2;
    public static Double SWP_SRV = 4.8;

  public static DcMotorSimple.Direction TR_DIR = DcMotorSimple.Direction.FORWARD;
  public static double TR_SPD_SCL = 0.5;
  public static double TR_MAX_IPS = 5.0;
  public static double[] TR_LEVS = {0.0, 7.8};

  public static double  WR_SENSE =0.1;

  //Marker
  public static double MK_ARM_STOW   = 0.05;
  public static double MK_ARM_DEPLOY = 0.82;
  public static double MK_ARM_CARRY  = 0.28;
  public static int MK_NUM_LEVS = 3;
  public static double[] MK_LEVS;
  public static double MK_SPD = 0.2;
  public static double MK_DEPLOY_SPD = 1.0;
  public static double MK_CARRY_SPD = 0.3;

  public static double LF_ARM_ROT_SPD = 10;
  public static Motors.MotorModel LF_ARM_ROT_MOT = Motors.MotorModel.GOBILDA_5202_71_2;

  public static double LF_ARM_EXT_SPD = 30;
  public static double LF_ARM_EXT_PULLEY_DIAM=1.5;
  public static double LF_ARM_EXT_EXT_GR=Math.PI*LF_ARM_EXT_PULLEY_DIAM*2;
  //7252 now uses cascade rigging on 2 stage lift
  public static Motors.MotorModel LF_ARM_EXT_MOT = Motors.MotorModel.GOBILDA_5202_13_7;
  public static DcMotorSimple.Direction LF_ARM_EXT_DIR = DcMotorSimple.Direction.FORWARD;
  public static double LF_ARM_EXT_CPI=LF_ARM_EXT_MOT.getCpr()/LF_ARM_EXT_EXT_GR;
  public static double[] LF_EXT_LEVS = {0.0, 30, 60, 90}; //0, 142, 386, 627

  public static double LF_STOW_DEG =  0.0;

  public static int LF_ARM_ROT_STOW = 0;
  public static int LF_ARM_ROT_LOW =  0;
  public static int LF_ARM_ROT_MID =  0;
  public static int LF_ARM_ROT_HIGH = 500;
  public static int LF_ARM_ROT_HITL = 500;
  public static int LF_ARM_ROT_GRAB = 500;
  public static int LF_ARM_ROT_MAX =  500;
  public static int LF_ARM_ROT_HERE = Integer.MAX_VALUE;
  public static DcMotorSimple.Direction LF_ARM_ROT_DIR = DcMotorSimple.Direction.FORWARD;
  public static double LF_ARM_ROT_GEAR = 1.0;

  //ImageProcessing
  public static int    IP_CAM_WID = 640;
  public static int    IP_CAM_HGT = 480;
  public static double IP_IMG_TOP = 0.00;
  public static double IP_IMG_BOT = 1.00;
  public static double IP_IMG_LFT = 0.00;
  public static double IP_IMG_RGT = 1.00;

  //These values are for red
  public static double IP_HUE_MIN_RED = 0.0;
  public static double IP_HUE_MAX_RED = 10.0;
  public static double IP_SAT_MIN_RED = 101.0;
  public static double IP_SAT_MAX_RED = 255.0;
  public static double IP_VAL_MIN_RED = 101.0;
  public static double IP_VAL_MAX_RED = 255.0;

  //These values are for blue
  public static double IP_HUE_MIN_BLUE = 105.0;
  public static double IP_HUE_MAX_BLUE = 180.0;
  public static double IP_SAT_MIN_BLUE = 101.0;
  public static double IP_SAT_MAX_BLUE = 255.0;
  public static double IP_VAL_MIN_BLUE = 101.0;
  public static double IP_VAL_MAX_BLUE = 255.0;

  public static double POSE_EQUAL = 1;


  //DT
  public static PIDFCoefficients SH_PID = new PIDFCoefficients(80.0, 0.0, 0.0,14.9);

  public static DcMotorSimple.Direction LD_DIR = DcMotorSimple.Direction.REVERSE;

  public static final double MMPERIN = 25.4;
  public static Motors.MotorModel DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
  public static double DT_CPMR = DT_MOTOR.getCpr(); //counts per motor output shaft rev
  public static double DT_MAX_RPM = DT_MOTOR.getRpm();
  public static double DT_EXT_GEAR_RATIO = 1.0;
  public static double DT_GEAR_RATIO = DT_MOTOR.getGear() * DT_EXT_GEAR_RATIO;
  public static double DT_CPWR = DT_CPMR * DT_EXT_GEAR_RATIO; //counts per whl rev
  public static double DT_WHEEL_DIAM =  96.0 / MMPERIN; //4.0 for tilerunner
  public static double DT_CIRCUM = DT_WHEEL_DIAM * Math.PI;
  public static double DC_ECIRC = DT_CIRCUM * DT_EXT_GEAR_RATIO;
  public static double DT_CPI = DT_CPMR / DC_ECIRC;
  public static double DT_IPC = 1.0/DT_CPI;
  public static double DC_RPM2VEL = DC_ECIRC / 60.0;

  public static double DT_TRACK_WIDTH = 16.34;
  public static double BOT_LEN = 18.0;
  public static double BOT_WID = 18.0;

  public static final double DT_SAF_IPS = 30.0;
  public static double DT_MAX_IPS;
  public static final double DT_SAF_CPS = DT_SAF_IPS * DT_CPI;
  public static double DT_MAX_CPS;
  public static double BUTT_SPD = .9987;
  public static double BORD_SPD = .2;
  public static Chassis bot= Chassis.B7252;
  public static Field.Alliance alliance = Field.Alliance.RED;
  public static PositionOption startPos = Field.StartPos.START_STACKS;
  public static PositionOption autonStrategy = Field.Route.QUALIFIER_ROUTE;

  public static ShelbyBot.DriveDir  DT_DIR = ShelbyBot.DriveDir.PUSHER;
  public static DcMotorSimple.Direction DT_LDIR = DcMotorSimple.Direction.REVERSE;
  public static DcMotorSimple.Direction DT_RDIR = DcMotorSimple.Direction.FORWARD;

  public static AxesOrder HUB_ORDER = AxesOrder.ZYX;
  public static AxesSigns HUB_SIGNS = AxesSigns.PPP;

  //CamServo info
  public static double CAM_RED_1 = 0.38;
  public static double CAM_RED_2 = 0.18;
  public static double CAM_BLU_1 = 0.12;
  public static double CAM_BLU_2 = 0.28;
  public static double CAM_STOW  = 0.28;

  //Defender info
  public static double DefendStow = 0.50;
  public static double DefendDeploy1Ring = 0.50;
  public static double DefendDeploy2Ring = 0.50;
  public static double DefendDeploy3Ring = 0.50;


  //Following variables are related to RoadRunner and should be tuned for each bot
  public static double LATERAL_MULTIPLIER = 1.2; //1.12;

  public static PIDFCoefficients MOTOR_VELO_PID =
      new PIDFCoefficients(20, 0, 0.3, 12.8);

  /*
   * These are the feedforward parameters used to model the drive motor behavior. If you are using
   * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
   * motor encoders or have elected not to use them for velocity control, these values should be
   * empirically tuned.
   */
  public static double kV = 1.0 / rpmToVelocity(DT_MAX_RPM);
  public static double kA = 0;
  public static double kStatic = 0;

  private static boolean kVsetManual = false;

  /*
   * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
   * Set this flag to false if drive encoders are not present and an alternative localization
   * method is in use (e.g., tracking wheels).
   *
   * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
   * from DriveVelocityPIDTuner.
   */
  public static boolean RUN_USING_ENCODER = true;

  /*
   * These values are used to generate the trajectories for you robot. To ensure proper operation,
   * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
   * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
   * small and gradually increase them later after everything is working. All distance units are
   * inches.
   */

  public static double MAX_VEL = 50; //RR tune  maxVel 59.96
  public static double MAX_ACCEL = 35;
  public static double MAX_ANG_VEL = Math.toRadians(180);
  public static double MAX_ANG_ACCEL = Math.toRadians(180);

  public static double SLW_VEL = 35;
  public static double SLW_ACCEL = 30;
  public static double SLW_ANG_VEL = Math.toRadians(180);
  //public static double SLW_ANG_ACCEL = Math.toRadians(120);

  public static TrajectoryVelocityConstraint defVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(MAX_ANG_VEL),
          new MecanumVelocityConstraint(MAX_VEL, DT_TRACK_WIDTH)
      ));
  public static TrajectoryAccelerationConstraint defAccelConstraint
      = new ProfileAccelerationConstraint(MAX_ACCEL);

  public static TrajectoryVelocityConstraint slwVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(SLW_ANG_VEL),
          new MecanumVelocityConstraint(SLW_VEL, DT_TRACK_WIDTH)
      ));
  public static TrajectoryAccelerationConstraint slwAccelConstraint
      = new ProfileAccelerationConstraint(SLW_ACCEL);

  public static TrajectoryVelocityConstraint stkVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(Math.toRadians(90)),
          new MecanumVelocityConstraint(20, DT_TRACK_WIDTH)
      ));

  public static TrajectoryVelocityConstraint telVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(Math.toRadians(200)),
          new MecanumVelocityConstraint(56.0, DT_TRACK_WIDTH)
      ));

  public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7.5, 0, 0);
  public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

  public static final boolean logVerbose = false;

  public static final String TAG = "SJH_RBC";

  public RobotConstants()
  {
    this(Chassis.B7252);
  }

  public RobotConstants(Chassis chassis)
  {
    RobotLog.dd(TAG, "RobotConstants ctor");
    init(chassis);
  }

  public static void init(Chassis chas, Field.Alliance alliance,
                          PositionOption startPos, double xOffset)
  {
    RobotConstants.alliance = alliance;
    RobotConstants.startPos = startPos;
    RobotConstants.DT_XOFFSET = xOffset;
    RobotLog.dd(TAG, "RobotConstants.init() " + alliance + " " + startPos);
    init(chas);
  }



  public static void init(Chassis chas)
  {
    bot = chas;
    RobotLog.dd(TAG, "RobotConstants.init() " + chas);

    Field.StartPos sPos = (Field.StartPos)startPos;
    Field.Route strtgy = (Field.Route)autonStrategy;

    switch (bot)
    {
      case GENERIC_CHASIS:
          RobotLog.dd(TAG, "Calibrating for Bot %s", bot);
          EL_MIN_ENCODER = -2700;
          EL_MAX_ENCODER = 200;
          EL_SPD = 1;
          EL_SPD_DWN = .333333333;
          EL_DIR = DcMotorSimple.Direction.REVERSE;
          EL_EXT_MOT = Motors.MotorModel.GOBILDA_5202_19_2;
          EL_NUM_LEVS = 6;
          EL_LEVS = new double[EL_NUM_LEVS];

          /* 2023 Robot Elbow Angles */
          EL_LEVS[0] = 0.2;
          EL_LEVS[1] = 2;
          EL_LEVS[2] = 5.6;
          EL_LEVS[3] = 6.0;
          EL_LEVS[4] = 6.5;
          EL_LEVS[5] = 8;

          EX_NUM_LEVS = 2;
          EX_LEVS = new double[EL_NUM_LEVS];
          EX_LEVS[0] = 0;
          EX_LEVS[1] = 0;

          IP_IMG_TOP = 0.20;
          IP_IMG_BOT = 0.80;
          IP_IMG_LFT = 0.05;
          IP_IMG_RGT = 0.95;


          DT_LDIR = DcMotorSimple.Direction.REVERSE;
          DT_RDIR = DcMotorSimple.Direction.FORWARD;

          BOT_LEN = 13.125;
          BOT_WID = 13.125;

          BUTT_SPD = .49;
          BORD_SPD = .3;
          MAX_VEL = 50;
          LATERAL_MULTIPLIER = 1.01;
          DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
          DT_EXT_GEAR_RATIO = 0.997;
          DT_WHEEL_DIAM = 96.0/MMPERIN;
          /* Track Width tuned via Road Runner Quick Start */
          DT_TRACK_WIDTH = 14;
          MOTOR_VELO_PID = new PIDFCoefficients(24.0, 0, 2.5, 12.8); //Needs RR tuning
          /* Heading & Translation PIDs tuned via Road Runner Quick Start */
          TRANSLATIONAL_PID = new PIDCoefficients(8, 2, 0.0);
          HEADING_PID = new PIDCoefficients(8.5, 1, 0.0);

          /* kV & KA tuned via Road Runner Quick Start in the Feed Forward */
          kV = 0.0185;
          kA = 0.003;
          kStatic = 0.05;

          kVsetManual = true;
          RUN_USING_ENCODER = false;

          POSE_EQUAL = 0.5;


        break;

      case B7253:
      case B7252:
          RobotLog.dd(TAG, "Calibrating for Bot %s", bot);
          EL_MIN_ENCODER = -2700;
          EL_MAX_ENCODER = 200;
          EL_SPD = 1;
          EL_SPD_DWN = .333333333;
          EL_DIR = DcMotorSimple.Direction.REVERSE;
          EL_EXT_MOT = Motors.MotorModel.GOBILDA_5202_19_2;
          EL_NUM_LEVS = 6;
          EL_LEVS = new double[EL_NUM_LEVS];

          /* 2023 Robot Elbow Angles */
          EL_LEVS[0] = 0.2;
          EL_LEVS[1] = 2;
          EL_LEVS[2] = 5.6;
          EL_LEVS[3] = 6.0;
          EL_LEVS[4] = 6.5;
          EL_LEVS[5] = 8;

          EX_NUM_LEVS = 2;
          EX_LEVS = new double[EL_NUM_LEVS];
          EX_LEVS[0] = 0;
          EX_LEVS[1] = 0;

          IP_IMG_TOP = 0.20;
          IP_IMG_BOT = 0.80;
          IP_IMG_LFT = 0.05;
          IP_IMG_RGT = 0.95;

          DT_LDIR = DcMotorSimple.Direction.REVERSE;
          DT_RDIR = DcMotorSimple.Direction.FORWARD;

          BOT_LEN = 13.125;
          BOT_WID = 13.125;

          BUTT_SPD = .49;
          BORD_SPD = .3;
          MAX_VEL = 42.155;
		  LATERAL_MULTIPLIER = 1.31;
          DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
          DT_EXT_GEAR_RATIO = 0.997;
          DT_WHEEL_DIAM = 96.0/MMPERIN;
          /* Track Width tuned via Road Runner Quick Start */

          DT_TRACK_WIDTH = 13.95;
          MOTOR_VELO_PID = new PIDFCoefficients(8.05, 0, 5.0, 13.45 );
          /* Heading & Translation PIDs tuned via Road Runner Quick Start */
          TRANSLATIONAL_PID = new PIDCoefficients(6.0, 0, .25);
          HEADING_PID = new PIDCoefficients(5.0, 0.4, 0.35);

          /* kV & KA tuned via Road Runner Quick Start in the Feed Forward */
//          kV = 0.0185;
//          kA = 0.003;
//          kStatic = 0.05;
          kV = 0.0;
          kA = 0.0;

          MAX_ACCEL = 30;
          MAX_ANG_VEL = Math.toRadians(120);
          MAX_ANG_ACCEL = Math.toRadians(60);
          kVsetManual = false;
          RUN_USING_ENCODER = true;
          POSE_EQUAL = 0.5;

        break;
      default:
        DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
        break;
    }

    DT_CPMR = DT_MOTOR.getCpr();
    DT_MAX_RPM = DT_MOTOR.getRpm();

    DT_GEAR_RATIO = DT_MOTOR.getGear() * DT_EXT_GEAR_RATIO;
    //DT_CPWR = DT_CPMR / DT_EXT_GEAR_RATIO;

    DT_CIRCUM = DT_WHEEL_DIAM * Math.PI;
    DC_ECIRC = DT_CIRCUM * DT_EXT_GEAR_RATIO;
    DT_CPI = DT_CPMR / DC_ECIRC;
    DC_RPM2VEL = DC_ECIRC / 60.0;

    DT_MAX_IPS = DT_MAX_RPM/60.0 * DT_CIRCUM;
    DT_MAX_CPS = DT_MAX_IPS * DT_CPI;

    if(!kVsetManual) kV = 1.0 / rpmToVelocity(DT_MAX_RPM);

    defVelConstraint =
        new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new MecanumVelocityConstraint(MAX_VEL, DT_TRACK_WIDTH)
        ));

    defAccelConstraint
        = new ProfileAccelerationConstraint(MAX_ACCEL);

    slwVelConstraint =
        new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(SLW_ANG_VEL),
            new MecanumVelocityConstraint(SLW_VEL, DT_TRACK_WIDTH)
        ));

    slwAccelConstraint
        = new ProfileAccelerationConstraint(SLW_ACCEL);

    params.put("SP_POWER", SP_POWER);
    params.put("SP_TIMEOUT", SP_TIMEOUT);
    params.put("DP_TIMEOUT", DP_TIMEOUT);
    params.put("IN_TIMEOUT", IN_TIMEOUT);
    params.put("IN_AUTO_PWR", IN_AUTO_PWR);
    params.put("IN_TELE_PWR", IN_TELE_PWR);
    params.put("EL_MAX_IPS", EL_MAX_IPS);
  }

  public enum Chassis
  {
    B7252,
    B7253,
    GENERIC_CHASIS,


  }

  public enum TrajEnum{
    INIT_TRAJ_1,
    INIT_TRAJ_2,
    INIT_TRAJ_BOTH
  }


//  public static double  = 1.0;

  public static double encoderTicksToInches(double ticks)
  {
    return ticks * DT_IPC;
  }

  public static double rpmToVelocity(double rpm)
  {
    return rpm * DC_RPM2VEL;
  }

  public static double getMotorVelocityF(double ticksPerSecond)
  {
    // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
    return 32767 / ticksPerSecond;
  }

  public static void info()
  {
    RobotLog.dd(TAG, "Chassis: %s", bot);
    RobotLog.dd(TAG, " DT_Track_Width: %4.1f", DT_TRACK_WIDTH);
    RobotLog.dd(TAG, " DT_Wheel_Diam: %4.1f", DT_WHEEL_DIAM);
    RobotLog.dd(TAG, " DT_CPI: %5.2f", DT_CPI);
    RobotLog.dd(TAG, " DT_CPMR: %4.1f", DT_CPMR);
    RobotLog.dd(TAG, " DT_CPWR: %4.1f", DT_CPWR);
    RobotLog.dd(TAG, " DC_RPM2VEL: %4.1f", DC_RPM2VEL);
    RobotLog.dd(TAG, " DT_EXT_GEAR: %4.1f", DT_EXT_GEAR_RATIO);
    RobotLog.dd(TAG, " DT_MAX_CPS: %4.1f", DT_MAX_CPS);
    RobotLog.dd(TAG, " DT_MAX_IPS: %4.1f", DT_MAX_IPS);
    RobotLog.dd(TAG, " DT_SAF_CPS: %4.1f", DT_SAF_CPS);
    RobotLog.dd(TAG, " DT_SAF_IPS: %4.1f", DT_SAF_IPS);
    RobotLog.dd(TAG, " DT_LDIR: %s", DT_LDIR.toString());
    RobotLog.dd(TAG, " DT_RDIR: %s", DT_RDIR.toString());
    RobotLog.dd(TAG, " DT_DIR: %s", DT_DIR.toString());
    RobotLog.dd(TAG, " DT_kA: %4.2f ", kA);
    RobotLog.dd(TAG, " DT_kV: %4.2f ", kV);
    RobotLog.dd(TAG, " DT_LAT_MULT: %4.2f ", LATERAL_MULTIPLIER);
    RobotLog.dd(TAG, " DT_MAX_VEL: %4.2f ", MAX_VEL);
    RobotLog.dd(TAG, " DT_MAX_ACCEL: %4.2f ", MAX_ACCEL);
    RobotLog.dd(TAG, " DT_MAX_ANG_VEL: %4.2f ", MAX_ANG_VEL);
    RobotLog.dd(TAG, " DT_MAX_ANG_ACCEL: %4.2f ", MAX_ANG_ACCEL);
    RobotLog.dd(TAG, " DT_RUN_USING_ENCODER: %s ", RUN_USING_ENCODER);
    RobotLog.dd(TAG, " DT_MOTOR_VELO_PID: %4.2f %4.2f %4.2f %4.2f",
        MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f);
    RobotLog.dd(TAG, " DT_TRANSLT_PID: %4.2f %4.2f %4.2f",
        TRANSLATIONAL_PID.kP, TRANSLATIONAL_PID.kI, TRANSLATIONAL_PID.kD);
    RobotLog.dd(TAG, " DT_HEADING_PID: %4.2f %4.2f %4.2f",
        HEADING_PID.kP, HEADING_PID.kI, HEADING_PID.kD);
    RobotLog.dd(TAG, "IN_DIR: %s", IN_DIR);
  }
}
