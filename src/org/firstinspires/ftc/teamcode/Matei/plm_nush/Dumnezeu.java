//package org.firstinspires.ftc.teamcode.Matei.plm_nush;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
////import org.firstinspires.ftc.teamcode.Encodere;
//
//public class Dumnezeu extends LinearOpMode {
//
//    private final DcMotor[] motors = new DcMotor[4];
//    private final Encodere encodere = new Encodere(motors);
//    // private final Gyro gyro = new Gyro(motors);
//
//    public void mergi(double power, int distace) {
//        encodere.mergi(power, distace);
//    }
//
//    public void stafe(double power, int distace) {
//        encodere.strafe(power, distace);
//    }
//
//    public void rotate(int grade, double power)  {
//        //gyro.rotire(grade, power);
//    }
//
//    public void checkDirection(){
//        // gyro.checkDirection();
//    }
//
//    public void setMotorsPower(double lfm, double lrm, double rfm, double rrm) {
//        encodere.setMotorsPower(lfm, lrm, rfm, rrm);
//    }
//
//    public void init(HardwareMap hardwareMap){
//        String[] names = {"front_left_motor", "back_left_motor", "front_right_motor", "back_right_motor"};
//
//        for(int i = 0; i < 4; i++){
//            motors[i] = hardwareMap.get(DcMotor.class, names[i]);
//            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//
//        motors[0].setDirection(DcMotor.Direction.REVERSE);
//        motors[1].setDirection(DcMotor.Direction.REVERSE);
//        motors[2].setDirection(DcMotor.Direction.REVERSE);
//
//        //gyro.init(hardwareMap);
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {}
//
//}
//
///*
// *               __
// *              / _)  (GRRRRR)
// *       .-^^^-/ /
// *   __/       /
// *  <__.|_|-|_|
// *
// *
// *
// *
// *              ######                ######
// *            ###     ####        ####     ###
// *           ##          ###    ###          ##
// *           ##             ####             ##
// *           ##             ####             ##
// *           ##           ##    ##           ##
// *           ##         ###      ###         ##
// *            ##  ########################  ##
// *         ######    ###            ###    ######
// *     ###     ##    ##              ##    ##     ###
// *  ###         ## ###      ####      ### ##         ###
// * ##           ####      ########      ####           ##
// *##             ###     ##########     ###             ##
// * ##           ####      ########      ####           ##
// *  ###         ## ###      ####      ### ##         ###
// *     ###     ##    ##              ##    ##     ###
// *         ######    ###            ###    ######
// *            ##  ########################  ##
// *           ##         ###      ###         ##
// *           ##           ##    ##           ##
// *           ##             ####             ##
// *           ##             ####             ##
// *           ##          ###    ###          ##
// *            ###     ####        ####     ###
// *              ######                ######
// *
// *
// *
// *
// *
// *
// *                   _________
// *                  |######|  \
// *                  |######|  |
// *                  |######|  |
// *          ________|######|__|________
// *         |########################|  \
// *         |########################|  |
// *         |________########________|  |
// *          \_______|######|  _______\_|
// *                  |######|  |
// *                  |######|  |
// *                  |######|  |
// *                  |######|  |
// *                  |######|  |
// *                  |######|  |
// *                  |######|  |
// *                  |______|__/
// *
// *
// *
// *
// *
// *          ._____________________________________________________.
// *          |         |                                |          |
// *          |         |                                |          |
// *          |         |                                |          |
// *          |_________|                                |__________|
// *          |          ._________.           ._________.          |
// *          |          |         |           |         |          |
// *          |          |         |           |         |          |
// *          |          |         |           |         |          |
// *          |          |_________|           |_________|          |
// *          |_________.                                 ._________|
// *          |         |                                 |         |
// *          |_________|_________________________________|_________|
// *          |         |                                 |         |
// *          |_________|                                 |_________|
// *          |                                                     |
// *          |                                                     |
// *          |                                                     |
// *          |                                                     |
// *          |                                                     |
// *          |              █                      █               |
// *          |                                                     |
// *          |                                                     |
// *          |        ||        ||           ||         ||         |
// *          |        ||        ||           ||         ||         |
// *          |        ||        ||           ||         ||         |
// *          |        ||        ||           ||         ||         |
// *          |________||________||___________||_________||_________|
// *
// *
// *
// *
// *
// */