package org.firstinspires.ftc.teamcode.robot;

public class Motors
{
  //RevHD, Gobilda, Neverest classic/orbital have 7 rising edges of Channel A per revolution
  //with a quadrature encoder (4 total edges - A rise, B rise, A fall, B fall) for a total
  //of 28 counts per pre-gear box encoder/pre-geared motor internal shaft revolution (CPER).
  //Counts per post-geared motor output shaft revolution (CPOR or just CPR) is CPER * motorGearing
  //This is already included in CPRs in enum below.
  //For example: GOBILDA_5202_19_2 has CPER=28 and GEAR=19.2.  So CPR=28*19.2=537.6

  //Matrix motors appear to use 2 different size planetary gears:
  //46/11 and a 46.17.  These produce gearing of 3.706 and 5.182
  //Which are the gearings of the two fastest motors after the 1:1
  //Other motor gearboxes are combos of these two
  // G1 = 1.0+46.0/17.0;
  // G2 = 1.0+46.0/11.0;

  public enum MotorModel
  {
    REV_CORE_HEX(288, 72, 125), //90deg motor, 4 CPER
    REV_HD_HEX_SPUR_40(1120, 40, 150),
    REV_HD_HEX_SPUR_20(560, 20, 300),
    REV_HD_HEX_PLANETARY_20(537.6, 19.2, 312.5),
    AM_NEVEREST_BARE(28, 1, 6600),
    AM_NEVEREST_CLASSIC_60(1680, 60, 105),
    AM_NEVEREST_CLASSIC_40(1120, 40, 160),
    AM_NEVEREST_ORBITAL_20(537.6, 19.2, 340),
    AM_NEVEREST_ORBITAL_3_7(103.6, 3.7, 1780),
    TETRIX_TORQUENADO_60(1440, 60, 100), //Tetrix have 6 rises of A: 24 CPER?!?
    TETRIX_TORQUENADO_40(960, 40, 150),
    TETRIX_TORQUENADO_20(480, 20, 300),
    MR_MATRIX_BARE(28, 1, 5994),
    GOBILDA_5201_53(1497.325, 53.475, 105), //Gobilda spur motors discontinued
    GOBILDA_5201_26(723.24, 25.83, 210),
    GOBILDA_5202_188(5264, 188, 30),
    GOBILDA_5202_139(3892, 139, 43),
    GOBILDA_5202_99_5(2786, 99.5, 60),
    GOBILDA_5202_71_2(1992.615, 71.1648, 84),
    GOBILDA_5202_50_9(1425.2, 50.9, 117),
    GOBILDA_5202_26_9(753.2, 26.9, 223),
    GOBILDA_5202_19_2(537.69, 19.2031, 312),
    GOBILDA_5202_13_7(383.6, 13.7, 435),
    GOBILDA_5202_5_2(145.6, 5.2, 1150),
    GOBILDA_5202_3_7(103.6, 3.7, 1620),
    GOBILDA_5200_1(28, 1, 6000);

    MotorModel(double cpr, double gear, double maxRPM)
    {
      this.cpr  = cpr;
      this.gear = gear;
      this.rpm  = maxRPM;
    }

    public double getCpr()
    {
      return cpr;
    }

    public double getGear()
    {
      return gear;
    }

    public double getRpm()
    {
      return rpm;
    }

    private final double cpr;
    private final double gear;
    private final double rpm;
  }
}
