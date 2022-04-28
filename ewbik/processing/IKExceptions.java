package processing;

public final class IKExceptions extends Exception {

    public static NullParentForBoneException NullParentForBoneException() {
        return new NullParentForBoneException();
    }

    public static class NullParentForBoneException extends NullPointerException {
        private static final long serialVersionUID = 24957056215695010L;

        public NullParentForBoneException() {
            super();
        }
    }

}
