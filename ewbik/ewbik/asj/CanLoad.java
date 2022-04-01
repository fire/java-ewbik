package ewbik.asj;

/**
 * This interface defines objects which can self populate from JSONObjects but don't
 * need to register themselves with the savestate tracker.
 *
 * @author Eron Gjoni
 */
public interface CanLoad {
    /**
     * @param j
     * @return should return an instance of itself for chaining
     */
    public CanLoad populateSelfFromJSON(ewbik.asj.data.JSONObject j);

    public ewbik.asj.data.JSONObject toJSONObject();

}