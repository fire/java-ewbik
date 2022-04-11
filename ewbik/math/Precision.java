package math;

public final class Precision {

    public static final double SAFE_MIN_DOUBLE;
    public static final double EPSILON_DOUBLE;
    public static final double SAFE_MIN_FLOAT;
    public static final double EPSILON_FLOAT;
    private static final long EXPONENT_OFFSET_DOUBLE = 1023l;
    private static final long EXPONENT_OFFSET_FLOAT = 127;

    static {
        EPSILON_DOUBLE = Double.longBitsToDouble((EXPONENT_OFFSET_DOUBLE - 53l) << 52);
        SAFE_MIN_DOUBLE = Double.longBitsToDouble((EXPONENT_OFFSET_DOUBLE - 1022l) << 52);

        EPSILON_FLOAT = Double.longBitsToDouble((EXPONENT_OFFSET_FLOAT - 24l) << 23);
        SAFE_MIN_FLOAT = Double.longBitsToDouble((EXPONENT_OFFSET_FLOAT - 126) << 23);
    }

    public static class LocalizedFormats {
        public static String ROTATION_MATRIX_DIMENSIONS = "a {0}x{1} matrix cannot be a rotation matrix";
    }

    public static class ZeroException extends Exception {

        public ZeroException(String nORM, float squareNorm) {
            // TODO Auto-generated constructor stub
        }

    }

    public static class CardanEulerSingularityException extends Exception {

        public CardanEulerSingularityException(boolean b) {
            // TODO Auto-generated constructor stub
        }

    }

    public static class NotARotationMatrixException extends Exception {

        public NotARotationMatrixException(String rOTATION_MATRIX_DIMENSIONS, int length, int length2) {
            // TODO Auto-generated constructor stub
        }

        public NotARotationMatrixException(String cLOSEST_ORTHOGONAL_MATRIX_HAS_NEGATIVE_DETERMINANT, float det) {
            // TODO Auto-generated constructor stub
        }

    }

    public class MathArithmeticException extends Exception {

        public MathArithmeticException(String zERO_NORM_FOR_ROTATION_DEFINING_VECTOR) {
        }

    }

    public class MathIllegalArgumentException extends Exception {
        public MathIllegalArgumentException(String zeroNormForRotationAxis) {

        }

    }

}
