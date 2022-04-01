package ewbik.processing;

import ewbik.data.EWBIKLoader;
import ewbik.data.EWBIKSaver;
import ik.Bone;
import ik.IKPin;
import processing.Skeleton3D;

import java.util.Collection;

public final class IO {

    /**
     * Return a single precision (float) version of the armature in stored in the
     * specified filepath
     *
     * @param path
     * @return the Armature, or null if the file does not specify an armature
     */
    public static Skeleton3D LoadArmature_singlePrecision(String path) {
        EWBIKLoader newLoader = new EWBIKLoader();
        @SuppressWarnings("unchecked")
        Collection<Skeleton3D> ArmatureList = (Collection<Skeleton3D>) newLoader.importSinglePrecisionArmatures(path,
                ewbik.processing.sceneGraph.Transform3D.class, Bone.class, Skeleton3D.class, ewbik.processing.singlePrecision.Kusudama.class, ewbik.processing.singlePrecision.LimitCone.class, IKPin.class);
        for (Skeleton3D a : ArmatureList) {
            return a;
        }
        return null;
    }

    /**
     * save the given armature into the specified filepath
     *
     * @param path
     * @param loadInto
     */
    public static void SaveArmature(String path, Skeleton3D toSave) {
        EWBIKSaver newSaver = new EWBIKSaver();
        newSaver.saveArmature(toSave, path);
    }

}
