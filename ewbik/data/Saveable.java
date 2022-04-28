package data;

import ewbik.asj.data.JSONObject;

public interface Saveable {

    JSONObject getSaveJSON(SaveManager saveManager);

    void notifyOfSaveIntent(SaveManager saveManager);

    void notifyOfSaveCompletion(SaveManager saveManager);

    void loadFromJSONObject(JSONObject j, LoadManager l);

    /**
     * called on all loaded objects once the full load sequence has been completed
     */
    default void notifyOfLoadCompletion() {
    }

    boolean isLoading();

    void setLoading(boolean loading);

    void makeSaveable(SaveManager saveManager);

    default String getIdentityHash() {
        String result = "";
        result += System.identityHashCode(this);
        String className = this.getClass().getSimpleName();
        result += "-" + className;
        return result;
    }
}
