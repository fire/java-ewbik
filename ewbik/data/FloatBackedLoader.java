package ewbik.data;

import ewbik.asj.LoadManager;
import ewbik.asj.Saveable;
import ewbik.asj.TypeIdentifier;
import ewbik.asj.data.JSONArray;
import ewbik.asj.data.JSONObject;
import ewbik.asj.data.StringFuncs;
import ewbik.ik.*;
import ewbik.math.Quaternion;
import ewbik.processing.singlePrecision.*;
import ik.Bone;
import ik.IKPin;
import processing.Node3D;
import processing.Skeleton3D;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import ewbik.processing.singlePrecision.KusudamaTwist;

public final class FloatBackedLoader extends LoadManager {

    public File currentFilePath;

    public HashMap<String, JSONObject> axesJSONObjects = new HashMap<>();
    public HashMap<String, Node3D> axesLoadObjects = new HashMap<>();

    public HashMap<String, JSONObject> armatureJSONObjects = new HashMap<>();
    public HashMap<String, Skeleton3D> armatureLoadObjects = new HashMap<>();

    public HashMap<String, JSONObject> boneJSONObjects = new HashMap<>();
    public HashMap<String, Bone> boneLoadObjects = new HashMap<>();

    public HashMap<String, ewbik.processing.singlePrecision.Kusudama> kusudamaLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> kusudamaJSONObjects = new HashMap<>();

    public HashMap<String, KusudamaTwist> KusudamaTwistLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> KusudamaTwistJSONObjects = new HashMap<>();

    public HashMap<String, IKPin> IKPinLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> IKPinJSONObjects = new HashMap<>();

    public boolean fileCorruptionDetected = false;

    private String tempLoadDirectory;

    private boolean Load = false;

    public FloatBackedLoader() {

    }

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

    public Collection<? extends Skeleton3D> importFile(File selection,
            Class<? extends Node3D> AxesClass,
            Class<? extends Bone> BoneClass,
            Class<? extends Skeleton3D> ArmatureClass,
            Class<? extends ewbik.processing.singlePrecision.Kusudama> KusudamaClass,
            Class<? extends KusudamaTwist> KusudamaTwistClass,
            Class<? extends IKPin> IKPinClass,
            EWBIKLoader loader) {
        JSONObject loadFile = StringFuncs.loadJSONObject(selection);
        clearCurrentLoadObjects();
        return loadJSON(loadFile,
                AxesClass,
                BoneClass,
                ArmatureClass,
                KusudamaClass,
                KusudamaTwistClass,
                IKPinClass);
    }

    public Collection<? extends Skeleton3D> loadJSON(JSONObject loadFile,
            Class<? extends Node3D> AxesClass,
            Class<? extends Bone> BoneClass,
            Class<? extends Skeleton3D> ArmatureClass,
            Class<? extends ewbik.processing.singlePrecision.Kusudama> KusudamaClass,
            Class<? extends KusudamaTwist> KusudamaTwistClass,
            Class<? extends IKPin> IKPinClass) {
        clearCurrentLoadObjects();

        AxesClass = AxesClass == null ? Node3D.class : AxesClass;
        BoneClass = BoneClass == null ? Bone.class : BoneClass;
        ArmatureClass = ArmatureClass == null ? Skeleton3D.class : ArmatureClass;
        KusudamaClass = KusudamaClass == null ? ewbik.processing.singlePrecision.Kusudama.class : KusudamaClass;
        KusudamaTwistClass = KusudamaTwistClass == null ? KusudamaTwist.class : KusudamaTwistClass;
        IKPinClass = IKPinClass == null ? IKPin.class : IKPinClass;

        createEmptyLoadMaps(axesJSONObjects, axesLoadObjects, loadFile.getJSONArray("node_3d"), AxesClass);
        createEmptyLoadMaps(boneJSONObjects, boneLoadObjects, loadFile.getJSONArray("bones"), BoneClass);
        createEmptyLoadMaps(armatureJSONObjects, armatureLoadObjects, loadFile.getJSONArray("skeleton_3d"),
                ArmatureClass);
        createEmptyLoadMaps(kusudamaJSONObjects, kusudamaLoadObjects, loadFile.getJSONArray("kusudamas"),
                KusudamaClass);
        createEmptyLoadMaps(KusudamaTwistJSONObjects, KusudamaTwistLoadObjects, loadFile.getJSONArray("kusudama_twists"),
                KusudamaTwistClass);
        createEmptyLoadMaps(IKPinJSONObjects, IKPinLoadObjects, loadFile.getJSONArray("ik_pins"), IKPinClass);

        loadGenerally(axesJSONObjects, axesLoadObjects);
        loadGenerally(IKPinJSONObjects, IKPinLoadObjects);
        loadGenerally(KusudamaTwistJSONObjects, KusudamaTwistLoadObjects);
        loadGenerally(kusudamaJSONObjects, kusudamaLoadObjects);
        loadGenerally(boneJSONObjects, boneLoadObjects);
        loadGenerally(armatureJSONObjects, armatureLoadObjects);

        for (Saveable s : allLoadedObjects)
            s.notifyOfLoadCompletion();

        updateArmatureSegments();

        System.gc();

        return armatureLoadObjects.values();
    }

    public void updateArmatureSegments() {
        Collection<Skeleton3D> armatures = armatureLoadObjects.values();
        for (Skeleton3D a : armatures) {
            a.refreshArmaturePins();
        }
    }

    public void clearCurrentLoadObjects() {

        axesJSONObjects.clear();
        axesLoadObjects.clear();

        armatureJSONObjects.clear();
        armatureLoadObjects.clear();

        boneJSONObjects.clear();
        boneLoadObjects.clear();

        kusudamaLoadObjects.clear();
        kusudamaJSONObjects.clear();
        KusudamaTwistLoadObjects.clear();
        KusudamaTwistJSONObjects.clear();

        allLoadedObjects.clear();

    }

    public String getCurrentFilePath() {
        if (currentFilePath == null) {
            return "";
        } else {
            return currentFilePath.getAbsolutePath();
        }
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
        Class keyClass = null;
        if (ti.key.getClass() == Class.class) {
            keyClass = (Class) ti.key;
        }
        Class valueClass = ti.value.getClass();
        if (valueClass == TypeIdentifier.class && keyClass != null) {
            Collection<String> jKeys = json.keys();
            for (String jk : jKeys) {
                JSONObject jValue = json.getJSONObject(jk);
                T keyObject = (T) getObjectFromClassMaps(keyClass, jk);

                HashMap<?, ?> innerHash = new HashMap<>();
                hashMapFromJSON(jValue, innerHash, (TypeIdentifier) ti.value);

                result.put(keyObject, (V) innerHash);
                return result;
            }
        } else {
            valueClass = (Class) ti.value;
            Collection<String> jKeys = json.keys();
            for (String jk : jKeys) {

                boolean javaClass = keyClass.getName().startsWith("java.lang");
                Object keyObject = javaClass ? parsePrimitive(keyClass, jk) : getObjectFromClassMaps(keyClass, jk);
                Object valueObject = null;
                Object obj = json.get(jk);
                valueObject = valueClass.getName().startsWith("java.lang") ? parsePrimitive(valueClass, "" + obj)
                        : getObjectFromClassMaps(valueClass, json.getString(jk));
                result.put((T) keyObject, (V) valueObject);
            }
            return result;
        }
        return result;
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
        Class keyClass = null;
        if (ti.key.getClass() == Class.class) {
            keyClass = (Class) ti.key;
        }
        Class valueClass = ti.value.getClass();
        HashMap<T, V> result = new HashMap<>();
        if (valueClass == TypeIdentifier.class && keyClass != null) {
            Collection<String> jKeys = json.keys();
            for (String jk : jKeys) {
                JSONObject jValue = json.getJSONObject(jk);
                T keyObject = (T) getObjectFromClassMaps(keyClass, jk);
                HashMap<?, ?> innerHash = new HashMap<>();
                hashMapFromJSON(jValue, innerHash, (TypeIdentifier) ti.value);
                result.put(keyObject, (V) innerHash);
                return result;
            }
        } else {
            valueClass = (Class) ti.value;
            Collection<String> jKeys = json.keys();
            for (String jk : jKeys) {

                boolean javaClass = keyClass.getName().startsWith("java.lang");
                Object keyObject = javaClass ? parsePrimitive(keyClass, jk) : getObjectFromClassMaps(keyClass, jk);
                Object valueObject = null;
                String hash = json.getString(jk);
                valueObject = getObjectFromClassMaps(valueClass, hash);
                result.put((T) keyObject, (V) valueObject);
            }
            return result;
        }
        return result;
    }

    /*
     * public Object parsePrimitive(Class keyClass, String toParse) {
     * if(keyClass == String.class) return toParse;
     * if(keyClass == Float.class) return Float.parseFloat(toParse);
     * if(keyClass == Double.class) return Double.parseDouble(toParse);
     * if(keyClass == Long.class) return Long.parseLong(toParse);
     * if(keyClass == Boolean.class) return Boolean.parseBoolean(toParse);
     * if(keyClass == Integer.class) return Integer.parseInt(toParse);
     * if(keyClass == Byte.class) return Byte.parseByte(toParse);
     * else return null;
     * }
     */

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
        Saveable result = null;

        if (Node3D.class.isAssignableFrom(keyClass))
            result = (Saveable) axesLoadObjects.get(identityHash);
        else if (Skeleton3D.class.isAssignableFrom(keyClass))
            result = (Saveable) armatureLoadObjects.get(identityHash);
        else if (Bone.class.isAssignableFrom(keyClass))
            result = (Saveable) boneLoadObjects.get(identityHash);
        else if (ewbik.processing.singlePrecision.Kusudama.class.isAssignableFrom(keyClass))
            result = (Saveable) kusudamaLoadObjects.get(identityHash);
        else if (KusudamaTwist.class.isAssignableFrom(keyClass))
            result = (Saveable) KusudamaTwistLoadObjects.get(identityHash);
        else if (IKPin.class.isAssignableFrom(keyClass))
            result = (Saveable) IKPinLoadObjects.get(identityHash);

        return result;
    }

    public <T extends Saveable> T getObjectFor(Class objectClass, JSONObject j, String hash) {
        if (j.hasKey(hash)) {
            return (T) getObjectFromClassMaps(objectClass, j.getString(hash));
        } else
            return null;
    }

    public void setTempLoadDirectory(String tempLoadDirectory) {
        tempLoadDirectory = tempLoadDirectory;
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
