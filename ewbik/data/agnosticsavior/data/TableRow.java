package ewbik.asj.data;

import java.io.PrintWriter;

/**
 * @webref data:composite
 * @see Table
 * @see Table#addRow()
 * @see Table#removeRow(int)
 * @see Table#clearRows()
 * @see Table#getRow(int)
 * @see Table#rows()
 */
public interface TableRow {

    /**
     * @param column ID number of the column to reference
     * @webref tablerow:method
     * @brief Get an String value from the specified column
     * @see TableRow#getInt(int)
     * @see TableRow#getFloat(int)
     */
    String getString(int column);

    /**
     * @param columnName title of the column to reference
     */
    String getString(String columnName);

    /**
     * @param column ID number of the column to reference
     * @webref tablerow:method
     * @brief Get an integer value from the specified column
     * @see TableRow#getFloat(int)
     * @see TableRow#getString(int)
     */
    int getInt(int column);

    /**
     * @param columnName title of the column to reference
     */
    int getInt(String columnName);

    /**
     * @param column ID number of the column to reference
     * @brief Get a long value from the specified column
     * @see TableRow#getFloat(int)
     * @see TableRow#getString(int)
     */

    long getLong(int column);

    /**
     * @param columnName title of the column to reference
     */
    long getLong(String columnName);

    /**
     * @param column ID number of the column to reference
     * @webref tablerow:method
     * @brief Get a float value from the specified column
     * @see TableRow#getInt(int)
     * @see TableRow#getString(int)
     */
    float getFloat(int column);

    /**
     * @param columnName title of the column to reference
     */
    float getFloat(String columnName);

    /**
     * @param column ID number of the column to reference
     * @brief Get a double value from the specified column
     * @see TableRow#getInt(int)
     * @see TableRow#getString(int)
     */
    double getDouble(int column);

    /**
     * @param columnName title of the column to reference
     */
    double getDouble(String columnName);

    /**
     * @param column ID number of the target column
     * @param value  value to assign
     * @webref tablerow:method
     * @brief Store a String value in the specified column
     * @see TableRow#setInt(int, int)
     * @see TableRow#setFloat(int, float)
     */
    void setString(int column, String value);

    /**
     * @param columnName title of the target column
     */
    void setString(String columnName, String value);

    /**
     * @param column ID number of the target column
     * @param value  value to assign
     * @webref tablerow:method
     * @brief Store an integer value in the specified column
     * @see TableRow#setFloat(int, float)
     * @see TableRow#setString(int, String)
     */
    void setInt(int column, int value);

    /**
     * @param columnName title of the target column
     */
    void setInt(String columnName, int value);

    /**
     * @param column ID number of the target column
     * @param value  value to assign
     * @brief Store a long value in the specified column
     * @see TableRow#setFloat(int, float)
     * @see TableRow#setString(int, String)
     */
    void setLong(int column, long value);

    /**
     * @param columnName title of the target column
     */
    void setLong(String columnName, long value);

    /**
     * @param column ID number of the target column
     * @param value  value to assign
     * @webref tablerow:method
     * @brief Store a float value in the specified column
     * @see TableRow#setInt(int, int)
     * @see TableRow#setString(int, String)
     */
    void setFloat(int column, float value);

    /**
     * @param columnName title of the target column
     */
    void setFloat(String columnName, float value);

    /**
     * @param column ID number of the target column
     * @param value  value to assign
     * @brief Store a double value in the specified column
     * @see TableRow#setFloat(int, float)
     * @see TableRow#setString(int, String)
     */
    void setDouble(int column, double value);

    /**
     * @param columnName title of the target column
     */
    void setDouble(String columnName, double value);

    /**
     * @return count of all columns
     * @webref tablerow:method
     * @brief Get the column count.
     */
    int getColumnCount();

    /**
     * @param columnName title of the target column
     * @return type of the column
     * @brief Get the column type.
     */
    int getColumnType(String columnName);

    /**
     * @param column ID number of the target column
     */
    int getColumnType(int column);

    /**
     * @return list of all column types
     * @brief Get the all column types
     */
    int[] getColumnTypes();

    /**
     * @param column ID number of the target column
     * @return title of the column
     * @webref tablerow:method
     * @brief Get the column title.
     */
    String getColumnTitle(int column);

    /**
     * @return list of all column titles
     * @brief Get the all column titles
     */
    String[] getColumnTitles();

    void write(PrintWriter writer);

    void print();
}
