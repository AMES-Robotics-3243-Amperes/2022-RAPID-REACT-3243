package frc.robot;

public class JoyCurveOption {
    private final int firstPower, secondPower;
    private final double aCoeff;

    public JoyCurveOption(int firstPower, int secondPower, double aCoeff) {
        this.firstPower = firstPower;
        this.secondPower = secondPower;
        this.aCoeff = aCoeff;
    }

    public int getFirstPower() {
        return firstPower;
    }

    public int getSecondPower() {
        return secondPower;
    }

    public double getACoefficient() {
        return aCoeff;
    }

    public double getBCoefficient() {
        return (1-aCoeff);
    }
}
