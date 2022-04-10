package ewbik.math;

public class RotationOrder {

        public static Vector3 X = new Vector3(1, 0, 0);
        public static Vector3 Y = new Vector3(0, 1, 0);
        public static Vector3 Z = new Vector3(0, 0, 1);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around X, then around Y, then
         * around Z
         */
        public static RotationOrder XYZ = new RotationOrder("XYZ", X, Y, Z);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around X, then around Z, then
         * around Y
         */
        public static RotationOrder XZY = new RotationOrder("XZY", X, Z, Y);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Y, then around X, then
         * around Z
         */
        public static RotationOrder YXZ = new RotationOrder("YXZ", Y, X, Z);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Y, then around Z, then
         * around X
         */
        public static RotationOrder YZX = new RotationOrder("YZX", Y, Z, X);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Z, then around X, then
         * around Y
         */
        public static RotationOrder ZXY = new RotationOrder("ZXY", Z, X, Y);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Z, then around Y, then
         * around X
         */
        public static RotationOrder ZYX = new RotationOrder("ZYX", Z, Y, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around X, then around Y, then
         * around X
         */
        public static RotationOrder XYX = new RotationOrder("XYX", X, Y, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around X, then around Z, then
         * around X
         */
        public static RotationOrder XZX = new RotationOrder("XZX", X, Z, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Y, then around X, then
         * around Y
         */
        public static RotationOrder YXY = new RotationOrder("YXY", Y, X, Y);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Y, then around Z, then
         * around Y
         */
        public static RotationOrder YZY = new RotationOrder("YZY", Y, Z, Y);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Z, then around X, then
         * around Z
         */
        public static RotationOrder ZXZ = new RotationOrder("ZXZ", Z, X, Z);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Z, then around Y, then
         * around Z
         */
        public static RotationOrder ZYZ = new RotationOrder("ZYZ", Z, Y, Z);

        /**
         * Name of the rotations order.
         */
        private String name;

        /**
         * Axis of the first rotation.
         */
        private Vector3 a1;

        /**
         * Axis of the second rotation.
         */
        private Vector3 a2;

        /**
         * Axis of the third rotation.
         */
        private Vector3 a3;

        /**
         * @param name name of the rotation order
         * @param a1   axis of the first rotation
         * @param a2   axis of the second rotation
         * @param a3   axis of the third rotation
         */
        public RotationOrder(String string, Vector3 a1, Vector3 a2, Vector3 a3) {
                this.name = string;
                this.a1 = a1;
                this.a2 = a2;
                this.a3 = a3;
        }

        /**
         * Get a string representation of the instance.
         *
         * @return a string representation of the instance (in fact, its name)
         */
        @Override
        public String toString() {
                return name;
        }

        /**
         * Get the axis of the first rotation.
         *
         * @return axis of the first rotation
         */
        public Vector3 getA1() {
                return a1;
        }

        /**
         * Get the axis of the second rotation.
         *
         * @return axis of the second rotation
         */
        public Vector3 getA2() {
                return a2;
        }

        /**
         * Get the axis of the second rotation.
         *
         * @return axis of the second rotation
         */
        public Vector3 getA3() {
                return a3;
        }

}