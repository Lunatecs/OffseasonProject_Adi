package frc.robot;

import java.util.function.DoubleSupplier;

public class SetDoubleSupplier implements DoubleSupplier{

    double value =0;
    @Override
    public double getAsDouble() {
       return value;
    }

    public void set(double value) {
        this.value = value;
    }
    


}
