package frc.robot;

/**
 * The JoyCurveOption Class contains all the values for a joystick curve function.
 * In combination with a SendableChooser, this allows us to create several presets for joystick curves
 * which drivers can switch between depending on their preference
 */
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
