import java.util.Arrays;

public class Path {

    double[] coeff;
    double[] firstDir;
    double[] secondDir;

    private double radius; // 0.4; 
    private double width; // width of the robot
    
    private double radiusInner, radiusOuter;

    public Path(PathDescriptor pd) {
        coeff = pd.A.solve(pd.b).transpose().getData()[0];

        firstDir = getDerivative(coeff);
        secondDir = getDerivative(firstDir);

        System.out.println(Arrays.toString(coeff));
    }
    

    public double getWidth() {
    		return width;
    }

    public Point getPoint(double t) {
        double y = 0;

        for (int degree = coeff.length - 1; degree >= 0; degree--) {
            y += coeff[coeff.length - 1 - degree] * Math.pow(t, degree);
        }

        return new Point(t, y);
    }

    public String toString() {
        String s = "y = ";

        for (int i = 0; i < coeff.length; i++) {
            s += coeff[i] + "x^" + i + " + ";
        }

        return s;
    }


    public double getRadius(double[] coeff, double x) {
        /**
         * r = sqrt (1 + f'(t)²)³ / f"(t)
         */
        radius = Math.pow(Math.sqrt(1 + Math.pow(evaluate(firstDir, x), 2)), 3) / evaluate(secondDir, x);
        return radius;

    }

    // [ 3, 9, 2, 4]
    // 3x^3 9x^2 2x, 4
    private double evaluate(double[] array, double x) {

        double sum = 0.0;

        for (int i = 0; i < array.length - 1; i++) {
            sum += Math.pow(x, array.length - i - 1) * array[i];
        }

        return sum;
    }


    private double[] getDerivative(double[] coeff) {
        // 3rd  2nd 1st none
        // [3,  0,  5,  9]
        double derivative[] = new double[coeff.length - 1];
        for (int i = 0; i < coeff.length - 1; i++) {
            derivative[i] = coeff[i] * (coeff.length - i - 1);
        }

        return derivative;
    }
    
    /**
     * Get the outer radius
     * @return
     */
    public double getOuterRadius() {
    		return radius + 1/2 * width;
    }

    /**
     * Get the inner radius
     * @return
     */
    public double getInnerRadius() {
    		return radius - 1/2 * width;
    }
    
    /**
     * Calculate the inner velocity given the velocity
     * @param velocity
     * @return
     */
    public double calculateInnerVelocity(double velocity) {
    	
	    	// 	x = inner velocity
	    	// r1 = inner radius
	    	// r2 = outer radius
    		// V = 1/2 x + 1/2 (r1/r2 * x);
    		// x = 2V / (1 + r1/r2)

    		return (2 * velocity) / ( 1 + getInnerRadius() / getOuterRadius());
    }
    
    
    
    
    
   
}
