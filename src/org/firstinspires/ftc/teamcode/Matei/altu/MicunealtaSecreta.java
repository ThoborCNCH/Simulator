// package org.firstinspires.ftc.teamcode.Matei.altu;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// @Autonomous(name = "MicunealtaSecreta", group = "a")
// public class MicunealtaSecreta extends MickeyMouse {

//     @Override
//     public void runOpMode() {
//         init(hardwareMap);
//         waitForStart();
//         if (opModeIsActive()) {
//             runBitch(1, 28);
//             // sleep(1000);
//             idle();
//             roteste(-44);
//             idle();
//             roteste(37);
//             idle();
//             int rand = (int) Math.floor(Math.random() * 3 + 1);
//             telemetry.addData("caz", rand);
//             telemetry.update();
//             B();
//             // A();
//             // C();
//             if(rand == 1){
//                 // A();
//             }
//             else if(rand == 2){
//                 // B();
//             }
//             else{
//                 // C();
//             }
// //            invartaTeAnPl(1, 45);
// //
//             // sleep(1000);
// //
// //        pct A:
//             // runBitch(0.1, 30);

// //        pct B:
// //        runBitch(1, 80);
// //        daTeCaMiBagPl(1, -20);
// //        runBitch(1, -20);

// //        pct C:
// //        runBitch(1, 100);
// //        mergiAnPulea(-0.5, 0.3, -1, 1,
// //                -16, -32, 0, 46);
// //        runBitch(1,-20);
// //        daTeCaMiBagPl(1, 80);
// //        invartaTeAnPl(1, -90);

//         }
//     }
//      public void A(){
//         runBitch(1, 60);
//         daTeCaMiBagPl(1, -15);
//     }

//     public void B(){
//         runBitch(1, 100);
//         daTeCaMiBagPl(1, 15);
//         // sleep(1000);
//         runBitch(1, -40);
//     }

//     public void C(){
//         runBitch(1, 130);
//         daTeCaMiBagPl(1, -15);
//         sleep(1000);
//         runBitch(0.1, -65);
//         setMotorsPower(0,0,0,0);
//     }
// }



package org.firstinspires.ftc.teamcode.Matei.altu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MicunealtaSecreta", group = "a")
public class MicunealtaSecreta extends MickeyMouse {

    @Override
    public void runOpMode() {
        init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            runBitch(0.4, 20);
            idle();
            roteste(-40);
            idle();

//            telemetry.addData("du-te la ", detect());
//            telemetry.update();
//            int caz = 0;
//            if(detect() == "A"){
//                // telemetry.addData("du-te la ", detect());
//                // telemetry.update();
//                caz = 1;
//            }
//            else if(detect() == "B"){
//                // telemetry.addData("du-te la ", detect());
//                // telemetry.update();
//                caz = 2;
//            }
//            else if(detect() == "C"){
//                // telemetry.addData("du-te la ", detect());
//                // telemetry.update();
//                caz = 3;
//            }
            idle();
            roteste(40);
            idle();
            int rand = (int) Math.floor(Math.random() * 3 + 1);

//            if(caz == 1){
//                // telemetry.addData("du-te la ", detect());
//                // telemetry.update();
//                A();
//            }
//            else if(caz == 2){
//                // telemetry.addData("du-te la ", detect());
//                // telemetry.update();
//                B();
//            }
//            else {
//                // telemetry.addData("du-te la ", detect());
//                // telemetry.update();
//                C();
//            }
            // else{
            //     telemetry.addData("asasdad", "pl");
            //     telemetry.update();
            // }
            // B();
            // A();
            // C();

        }
    }

    public void A(){
        telemetry.addData("as", "a");
        telemetry.update();
        runBitch(0.6, 38);
        daTeCaMiBagPl(0.6, -10);
    }

    public void B(){
        telemetry.addData("as", "b");
        telemetry.update();
        runBitch(0.6, 61);
        daTeCaMiBagPl(0.6, 13);
        sleep(1500);
        runBitch(0.6, -20);
    }

    public void C(){
        telemetry.addData("as", "c");
        telemetry.update();
        runBitch(0.6, 78);
        daTeCaMiBagPl(0.6, -10);
        sleep(1000);
        runBitch(0.6, -41);
        setMotorsPower(0,0,0,0);
    }
}



