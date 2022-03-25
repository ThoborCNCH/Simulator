package org.firstinspires.ftc.teamcode.Matei.incercare;


public class Trasee {
    private final SfPavel sfPavel;

    public Trasee(SfPavel sfPavel) {
        this.sfPavel = sfPavel;
    }

    public void albastruMergiLaPunctulA() {
        //varianta 1
//        sfPavel.mergi(1, 44);
//        sfPavel.stafe(1, 20);
//        sfPavel.stafe(1, -23);

        //varianta 2
        sfPavel.mergi(1, 30);
        //drop
        sfPavel.mergiCiudat(
                0.5, 1, 0.3, 1,
                -16, -32, 0, 46);
        sfPavel.mergi(1, -10);
        sfPavel.rotire(0.3, -90);
    }

    public void albastruMergiLaPunctulB() {
        //varianta 1
//        sfPavel.mergi(1, 65);
//        sfPavel.mergi(1, -15);

        //varianta 2
        sfPavel.mergi(1, 50);
        sfPavel.stafe(1, -15);
        sfPavel.mergi(1, -10);
    }

    public void albastruMergiLaPunctulC() {
        //varianta 1
//        sfPavel.mergi(1, 90);
//        sfPavel.stafe(1, 20);
//        sfPavel.stafe(1, -20);
//        sfPavel.mergi(1, -40);


        sfPavel.mergi(1, 75);
        //drop
        sfPavel.mergiCiudat(
                0.5, 1, 0.3, 1,
                -16, -32, 0, 46);
        sfPavel.mergi(1, -10);
        sfPavel.rotire(0.3, -90);
        sfPavel.mergi(1, -50);
    }

    public void albastruDreapta() {
        sfPavel.stafe(1, -7);
        sfPavel.mergi(1, 75);
    }

    public void rosuMergiLaPunctulA() {
//        sfPavel.mergi(1, 44);
//        sfPavel.stafe(1, -20);
//        sfPavel.stafe(1, 23);

        //varianta 2
        sfPavel.mergi(1, 30);
        //drop
//        sfPavel.mergiCiudat(1, 32, 32, 0, -32);
        sfPavel.mergi(1, -10);
        sfPavel.rotire(0.3, 90);
    }

    public void rosuMergiLaPunctulB() {
        sfPavel.mergi(1, 65);
        sfPavel.mergi(1, -15);
    }

    public void rosuMergiLaPunctulC() {
        sfPavel.mergi(1, 90);
        sfPavel.stafe(1, -20);
        sfPavel.stafe(1, 20);
        sfPavel.mergi(1, -40);
    }

    public void rosustanga() {
        sfPavel.stafe(1, 7);
        sfPavel.mergi(1, 75);
    }
}

