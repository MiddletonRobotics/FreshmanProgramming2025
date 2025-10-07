package org.firstinspires.ftc.teamcode.utilties;

public enum BallType {
    PURPLE(123,102,194),
    GREEN(0,0,0);

    final int r;
    final int g;
    final int b;

    BallType(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public int getR() {
        return r;
    }

    public int getG() {
        return g;
    }

    public int getB() {
        return b;
    }
}