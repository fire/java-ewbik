package InverseKinematics;

import ewbik.asj.data.JSONArray;
import ewbik.asj.data.JSONObject;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import ewbik.data.EWBIKLoader;

public class LoadManager {

    private final boolean Load = false;
    public HashMap<Class, HashMap<String, JSONObject>> jsonObjects = new HashMap<>();
    public HashMap<Class, HashMap<String, Saveable>> classObjects = new HashMap<>();
    public ArrayList<Saveable> allLoadedObjects = new ArrayList<>();
    public File currentFilePath;
    public HashMap<String, JSONObject> axesJSONObjects = new HashMap<>();
    public HashMap<String, Node3D> axesLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> armatureJSONObjects = new HashMap<>();
    public HashMap<String, Skeleton3D> armatureLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> boneJSONObjects = new HashMap<>();
    public HashMap<String, Bone> boneLoadObjects = new HashMap<>();
    public HashMap<String, Kusudama> kusudamaLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> kusudamaJSONObjects = new HashMap<>();
    public HashMap<String, LimitCone> LimitConeLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> LimitConeJSONObjects = new HashMap<>();
    public HashMap<String, IKPin> IKPinLoadObjects = new HashMap<>();
    public HashMap<String, JSONObject> IKPinJSONObjects = new HashMap<>();
    public boolean fileCorruptionDetected = false;
    private String tempLoadDirectory;

    public <T> void createEmptyLoadMaps(Map<String, JSONObject> jMap, Map<String, ? super T> oMap, JSONArray jArr,
            Class<T> c) {
        try {
            for (int i = 0; i < jArr.size(); i++) {
                JSONObject jo = jArr.getJSONObject(i);
                String id = jo.getString("identityHash");

                jMap.put(id, jo);
                Object created = c.newInstance();
                oMap.put(id, (T) created);
                allLoadedObjects.add((Saveable) created);
            }
        } catch (InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    /**
     * This function should be called when initializing the loader object so that it
     * knows
     * which keys in the JSON file correspond to which classes you intend to load.
     *
     * @param classMap a map of Class objects and their corresponding
     *                 JSON key names.
     */
    public void initializeClassMaps(HashMap<String, Class> classMap) {
        for (String k : classMap.keySet()) {
            Class c = classMap.get(k);
            if (jsonObjects.get(k) == null) {
                jsonObjects.put(c, new HashMap<>());
            }
            if (classObjects.get(c) == null) {
                classObjects.put(c, new HashMap<>());
            }
        }
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
                                                       Class<? extends Kusudama> KusudamaClass,
                                                       Class<? extends LimitCone> LimitConeClass,
                                                       Class<? extends IKPin> IKPinClass,
                                                       EWBIKLoader loader) {
        JSONObject loadFile = ewbik.asj.data.StringFuncs.loadJSONObject(selection);
        clearCurrentLoadObjects();
        return loadJSON(loadFile,
                AxesClass,
                BoneClass,
                ArmatureClass,
                KusudamaClass,
                LimitConeClass,
                IKPinClass);
    }

    public Collection<? extends Skeleton3D> loadJSON(JSONObject loadFile,
                                                     Class<? extends Node3D> AxesClass,
                                                     Class<? extends Bone> BoneClass,
                                                     Class<? extends Skeleton3D> ArmatureClass,
                                                     Class<? extends Kusudama> KusudamaClass,
                                                     Class<? extends LimitCone> LimitConeClass,
                                                     Class<? extends IKPin> IKPinClass) {
        clearCurrentLoadObjects();

        AxesClass = AxesClass == null ? Node3D.class : AxesClass;
        BoneClass = BoneClass == null ? Bone.class : BoneClass;
        ArmatureClass = ArmatureClass == null ? Skeleton3D.class : ArmatureClass;
        KusudamaClass = KusudamaClass == null ? Kusudama.class : KusudamaClass;
        LimitConeClass = LimitConeClass == null ? LimitCone.class : LimitConeClass;
        IKPinClass = IKPinClass == null ? IKPin.class : IKPinClass;

        createEmptyLoadMaps(axesJSONObjects, axesLoadObjects, loadFile.getJSONArray("node_3d"), AxesClass);
        createEmptyLoadMaps(boneJSONObjects, boneLoadObjects, loadFile.getJSONArray("bones"), BoneClass);
        createEmptyLoadMaps(armatureJSONObjects, armatureLoadObjects, loadFile.getJSONArray("skeleton_3d"),
                ArmatureClass);
        createEmptyLoadMaps(kusudamaJSONObjects, kusudamaLoadObjects, loadFile.getJSONArray("kusudamas"),
                KusudamaClass);
        createEmptyLoadMaps(LimitConeJSONObjects, LimitConeLoadObjects, loadFile.getJSONArray("limit_cones"),
                LimitConeClass);
        createEmptyLoadMaps(IKPinJSONObjects, IKPinLoadObjects, loadFile.getJSONArray("ik_pins"), IKPinClass);

        loadGenerally(axesJSONObjects, axesLoadObjects);
        loadGenerally(IKPinJSONObjects, IKPinLoadObjects);
        loadGenerally(LimitConeJSONObjects, LimitConeLoadObjects);
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
        LimitConeLoadObjects.clear();
        LimitConeJSONObjects.clear();

        allLoadedObjects.clear();

    }

    public String getCurrentFilePath() {
        if (currentFilePath == null) {
            return "";
        } else {
            return currentFilePath.getAbsolutePath();
        }
    }

    public <T extends Saveable> T getObjectFor(Class objectClass, JSONObject j, String hash) {
        if (j.hasKey(hash)) {
            return (T) getObjectFromClassMaps(objectClass, j.getString(hash));
        } else
            return null;
    }

    /**
     * general loader for when nothing fancy is required (I should make pretty much
     * everything use this eventually)
     *
     * @param jsonForm
     * @param saveableForm
     */
    public void loadGenerally(HashMap<String, JSONObject> jsonForm, HashMap<String, ? extends Saveable> saveableForm) {
        Collection<String> K = jsonForm.keySet();
        for (String k : K) {
            JSONObject kj = jsonForm.get(k);
            Saveable si = saveableForm.get(k);
            si.loadFromJSONObject(kj, this);
        }
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
        Saveable result = null;

        if (Node3D.class.isAssignableFrom(keyClass))
            result = axesLoadObjects.get(identityHash);
        else if (Skeleton3D.class.isAssignableFrom(keyClass))
            result = armatureLoadObjects.get(identityHash);
        else if (Bone.class.isAssignableFrom(keyClass))
            result = boneLoadObjects.get(identityHash);
        else if (Kusudama.class.isAssignableFrom(keyClass))
            result = kusudamaLoadObjects.get(identityHash);
        else if (LimitCone.class.isAssignableFrom(keyClass))
            result = LimitConeLoadObjects.get(identityHash);
        else if (IKPin.class.isAssignableFrom(keyClass))
            result = IKPinLoadObjects.get(identityHash);

        return result;
    }

    public void setTempLoadDirectory(String tempLoadDirectory) {
        tempLoadDirectory = tempLoadDirectory;
    }

    public <T extends Object> void arrayListFromJSONArray(JSONArray jsonArray, ArrayList<T> list, Class c) {

        for (int i = 0; i < jsonArray.size(); i++) {
            Object item = jsonArray.get(i);

            if (c == ewbik.math.Vector3.class)
                list.add((T) new ewbik.math.Vector3(jsonArray.getJSONArray(i)));
            else if (c == ewbik.math.Quaternion.class)
                list.add((T) new ewbik.math.Quaternion(jsonArray.getJSONArray(i)));
            else if (c.getName().startsWith("java.lang"))
                list.add((T) LoadManager.parsePrimitive(c, "" + jsonArray.get(i)));
            else {
                String sitem = Number.class.isAssignableFrom(item.getClass()) ? "" + item : (String) item;
                list.add((T) getObjectFromClassMaps(c, sitem));
            }
        }
    }
}
