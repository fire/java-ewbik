package ewbik.data;

import InverseKinematics.LoadManager;
import InverseKinematics.Saveable;
import InverseKinematics.TypeIdentifier;
import ewbik.asj.data.JSONArray;
import ewbik.asj.data.JSONObject;
import ewbik.asj.data.StringFuncs;
import ewbik.math.Quaternion;
import processing.Bone;
import processing.IKPin;
import processing.Node3D;
import processing.Skeleton3D;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

public class EWBIKLoader {
    LoadManager floatBackedLoader = new LoadManager();

    public static Object parsePrimitive(Class keyClass, String toParse) {
        if (keyClass == String.class)
            return toParse;
        if (keyClass == Float.class)
            return Float.parseFloat(toParse);
        if (keyClass == Double.class)
            return Double.parseDouble(toParse);
        if (keyClass == Long.class)
            return Long.parseLong(toParse);
        if (keyClass == Boolean.class)
            return Boolean.parseBoolean(toParse);
        if (keyClass == Integer.class)
            return Integer.parseInt(toParse);
        if (keyClass == Byte.class)
            return Byte.parseByte(toParse);
        else
            return null;
    }

    public static void setTempLoadDirectory(String tempLoadDirectory) {
        tempLoadDirectory = tempLoadDirectory;
    }

    /**
     * NOTE: in order to load custom (extended classes), those classes MUST have a
     * default constructor!
     *
     * @param filePath       location of file to import
     * @param AxesClass      the class object you've used to extend the Axes
     *                       class. If null, Axes will be used.
     * @param BoneClass      the class object you've used to extend the Bone
     *                       class. If null, Bone will be used.
     * @param ArmatureClass  the class object you've used to extend the
     *                       Skeleton3D class. If null, Skeleton3D will
     *                       be used.
     * @param KusudamaClass  the class object you've used to extend the
     *                       Kusudama class. If null, Kusudama will
     *                       be used.
     * @param LimitConeClass the class object you've used to extend the
     *                       LimitCone class. If null, LimitCone
     *                       will be used.
     * @param IKPinClass     the class object you've used to extend the
     *                       IKPin class. If null, IKPin will be
     *                       used.
     * @return a list of all instantiated armatures specified by the input file.
     */

    public Collection<? extends Skeleton3D> importSinglePrecisionArmatures(String filepath,
            Class<? extends Node3D> AxesClass,
            Class<? extends Bone> BoneClass,
            Class<? extends Skeleton3D> ArmatureClass,
            Class<? extends ewbik.processing.singlePrecision.Kusudama> KusudamaClass,
            Class<? extends ewbik.processing.singlePrecision.LimitCone> LimitConeClass,
            Class<? extends IKPin> IKPinClass) {
        File selection = new File(filepath);
        JSONObject loadFile = StringFuncs.loadJSONObject(selection);
        clearCurrentLoadObjects();
        return floatBackedLoader.loadJSON(loadFile,
                AxesClass,
                BoneClass,
                ArmatureClass,
                KusudamaClass,
                LimitConeClass,
                IKPinClass);
    }

    public void updateArmatureSegments() {
        floatBackedLoader.updateArmatureSegments();
    }

    public void clearCurrentLoadObjects() {
        floatBackedLoader.clearCurrentLoadObjects();
    }

    public String getCurrentFilePath() {
        return floatBackedLoader.getCurrentFilePath();
    }

    /**
     * takes a JSONObject and parses it into the format specified by the
     * TypeIdentifier.
     * The Value parameter can be another hashmap, and this
     * will nest hashmaps from jsonObjects accordingly.
     *
     * @param json
     * @param result
     */
    public <T extends Object, V extends Object> HashMap<T, V> hashMapFromJSON(JSONObject json, HashMap<T, V> result,
            TypeIdentifier ti) {
        return floatBackedLoader.hashMapFromJSON(json, result, ti);
    }

    /**
     * takes a JSONObject and parses it into the format specified by the
     * TypeIdentifier.
     * The Value parameter can be another hashmap, and this
     * will nest hashmaps from jsonObjects accordingly.
     *
     * @param json
     * @param result
     */
    public <T extends Object, V extends Object> HashMap<T, V> hashMapFromJSON(JSONObject json, TypeIdentifier ti) {
        return floatBackedLoader.hashMapFromJSON(json, ti);
    }

    /**
     * returns the appropriate object from the load hashmaps based on the
     * identityHash and keyClass.
     * if the object is not found, returns null
     *
     * @param keyClass
     * @param identityHash
     * @return
     */

    public Saveable getObjectFromClassMaps(Class keyClass, String identityHash) {
        return floatBackedLoader.getObjectFromClassMaps(keyClass, identityHash);
    }

    public <T extends Object> void arrayListFromJSONArray(JSONArray jsonArray, ArrayList<T> list, Class c) {
        for (int i = 0; i < jsonArray.size(); i++) {
            Object item = jsonArray.get(i);
            if (c == ewbik.math.Vector3.class)
                list.add((T) new ewbik.math.Vector3(jsonArray.getJSONArray(i)));
            else if (c == Quaternion.class)
                list.add((T) new Quaternion(jsonArray.getJSONArray(i)));
            else if (c.getName().startsWith("java.lang"))
                list.add((T) parsePrimitive(c, "" + jsonArray.get(i)));
            else {
                String sitem = Number.class.isAssignableFrom(item.getClass()) ? "" + item : (String) item;
                list.add((T) getObjectFromClassMaps(c, sitem));
            }
        }
    }

}
