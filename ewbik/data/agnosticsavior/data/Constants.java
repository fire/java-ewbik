package ewbik.asj.data;

/**
 * These are mostly just constants potentially useful for saving and loading
 * JSOn files.
 */
public interface Constants {
    // platform IDs for StringFuncs.platform
    int OTHER = 0;
    int WINDOWS = 1;
    int MACOSX = 2;
    int LINUX = 3;

    String[] platformNames = {
            "other", "windows", "macosx", "linux"
    };
    String WHITESPACE = " \t\n\r\f\u00A0";

    int GROUP = 0;
    int POINT = 2; // primitive
    int POINTS = 3; // vertices
    int LINE = 4; // primitive
    int TRIANGLE = 8; // primitive
    int TRIANGLES = 9; // vertices
    int TRIANGLE_STRIP = 10; // vertices
    int TRIANGLE_FAN = 11; // vertices
    int QUAD = 16; // primitive
    int QUADS = 17; // vertices
    int QUAD_STRIP = 18; // vertices
    int POLYGON = 20; // in the end, probably cannot
    int PATH = 21; // separate these two
    int RECT = 30; // primitive
    int ELLIPSE = 31; // primitive
    int ARC = 32; // primitive
    int SPHERE = 40; // primitive
    int BOX = 41; // primitive

    int OPEN = 1;
    int CLOSE = 2;

    int CORNER = 0;
    int CORNERS = 1;
    int RADIUS = 2;
    int CENTER = 3;
    int DIAMETER = 3;
    int CHORD = 2;
    int PIE = 3;
    int BASELINE = 0;
    int TOP = 101;
    int BOTTOM = 102;
    int NORMAL = 1;

    int CLAMP = 0;
    int REPEAT = 1;

    int MODEL = 4;

}
