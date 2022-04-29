package InverseKinematics;

/**
 * This interface defines objects which can self populate from JSONObjects but
 * don't
 * need to register themselves with the savestate tracker.
 *
 * @author Eron Gjoni
 */
public interface CanLoad {
    /**
     * @param j
     * @return should return an instance of itself for chaining
     */
    CanLoad populateSelfFromJSON(ewbik.asj.data.JSONObject j);

    ewbik.asj.data.JSONObject toJSONObject();

}