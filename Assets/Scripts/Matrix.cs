using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

[Serializable]
public struct Matrix
{
    public int rowLength
    {
        get { return elements.Length; }
    }
    public int columnLength
    {
        get { return elements[0].Length; }
    }
    public static Matrix Columns(params Matrix[] columns)
    {
        var m = new Matrix();
        m.elements = new float[columns[0].rowLength][];
        for (int i = 0; i < columns[0].rowLength; i++)
        {
            m.elements[i] = new float[columns.Length];
            for (int k = 0; k < columns.Length; k++)
            {
                m.elements[i][k] = columns[k].elements[i][0];
            }
        }
        return m;
    }
    public static Matrix Column(List<float> rows)
    {
        Matrix m = new Matrix();
        m.elements = new float[rows.Count][];
        for (int i = 0; i < rows.Count; i++)
        {
            m.elements[i] = new float[] { rows[i] };
        }
        return m;
    }
    public static Matrix operator +(Matrix left, Matrix right)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var rightUpper = right.rowLength - 1;
        var colLength = left.columnLength;
        for (int i = 0, j = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            j = (i > rightUpper) ? rightUpper : i;
            for (int k = 0; k < colLength; k++)
            {
                m.elements[i][k] = left.elements[i][k] + right.elements[j][k];
            }
        }
        return m;
    }
    public static Matrix operator +(Matrix left, float right)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var colLength = left.columnLength;
        for (int i = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            for (int k = 0; k < colLength; k++)
            {
                m.elements[i][k] = left.elements[i][k] + right;
            }
        }
        return m;
    }
    internal static Matrix ColumnExtend(Matrix col1, Matrix col2)
    {
        var m = new Matrix();
        m.elements = new float[col1.rowLength + col2.rowLength][];
        //m.elements.Length
        for (int i = 0; i < col1.rowLength; i++)
        {
            m.elements[i] = new float[] { col1.elements[i][0] };
        }
        for (int i = col1.rowLength, k = 0; i < m.rowLength; i++, k++)
        {
            m.elements[i] = new float[] { col2.elements[k][0] };
        }
        return m;
    }
    // 所有元素乘以-1
    public static Matrix operator -(Matrix left)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var colLength = left.columnLength;
        for (int i = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            for (int k = 0; k < colLength; k++)
            {
                m.elements[i][k] = -left.elements[i][k];
            }
        }
        return m;
    }
    // left 的行数必须大于等于 right
    public static Matrix operator -(Matrix left, Matrix right)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var rightUpper = right.rowLength - 1;
        var colLength = left.columnLength;
        for (int i = 0, j = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            j = (i > rightUpper) ? rightUpper : i;
            for (int k = 0; k < colLength; k++)
            {
                m.elements[i][k] = left.elements[i][k] - right.elements[j][k];
            }
        }
        return m;
    }
    internal static Matrix Row(Vector3 v)
    {
        var m = new Matrix();
        m.elements = new float[1][];
        m.elements[0] = new float[] { v.x, v.y, v.z };
        return m;
    }
    internal static Matrix Rows(Vector3[] vs)
    {
        var m = new Matrix();
        m.elements = new float[vs.Length][];
        for (int i = 0; i < vs.Length; i++)
        {
            m.elements[i] = new float[] { vs[i].x, vs[i].y, vs[i].z };
        }
        return m;
    }
    public static Matrix operator *(Matrix left, Matrix right)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var rRowUpper = right.rowLength - 1;
        var rColUpper = right.columnLength - 1;
        var colLength = left.columnLength;
        for (int i = 0, j = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            j = (i > rRowUpper) ? rRowUpper : i;
            for (int k = 0, k2 = 0; k < colLength; k++)
            {
                k2 = (k > rColUpper) ? rColUpper : k;
                var l = left.elements[i][k];
                var r = right.elements[j][k2];
                m.elements[i][k] = l * r;
            }
        }
        return m;
    }
    public static Matrix operator *(Matrix left, float right)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var colLength = left.columnLength;
        for (int i = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            for (int k = 0; k < colLength; k++)
            {
                m.elements[i][k] = left.elements[i][k] * right;
            }
        }
        return m;
    }
    public static Matrix operator *(float left, Matrix m)
    {
        return m * left;
    }
    public static Matrix operator /(Matrix left, Matrix right)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var rightUpper = right.rowLength - 1;
        var colLength = left.columnLength;
        for (int i = 0, j = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            j = (i > rightUpper) ? rightUpper : i;
            for (int k = 0; k < colLength; k++)
            {
                m.elements[i][k] = left.elements[i][k] / right.elements[j][k];
            }
        }
        return m;
    }
    public static Matrix operator /(Matrix left, float right)
    {
        var m = new Matrix();
        m.elements = new float[left.rowLength][];
        var colLength = left.columnLength;
        for (int i = 0; i < left.rowLength; i++)
        {
            m.elements[i] = new float[colLength];
            for (int k = 0; k < colLength; k++)
            {
                m.elements[i][k] = left.elements[i][k] / right;
            }
        }
        return m;
    }
    public static Matrix operator +(int left, Matrix m)
    {
        return m + left;
    }
    public static Matrix operator <(Matrix m, float right)
    {
        var list = new List<float>();
        for (int i = 0; i < m.rowLength; i++)
        {
            if (m.elements[i][0] < right)
                list.Add(i);
        }
        var result = Matrix.Column(list);
        return result;
    }
    public static Matrix operator >(Matrix m, float right)
    {
        var list = new List<float>();
        for (int i = 0; i < m.rowLength; i++)
        {
            if (m.elements[i][0] > right)
                list.Add(i);
        }
        var result = Matrix.Column(list);
        return result;
    }
    public float[] this[int row]
    {
        get { return elements[row]; }
        set
        {
            elements[row] = new float[value.Length];
            Array.Copy(value, elements[row], columnLength);
        }
    }
    public float this[int row, int column]
    {
        get { return elements[row][column]; }
        set { elements[row][column] = value; }
    }
    public Matrix this[int row, Vector2 range]
    {
        get { throw null; }
    }
    public Matrix this[Vector2 range, int column]
    {
        get { throw null; }
    }
    public Matrix this[Vector2 rows, Vector2 columns]
    {
        get { throw null; }
    }
    // 储存 i*j 个元素 
    public float[][] elements;
    public string ToString(string title)
    {
        string s = title + " len:" + rowLength.ToString();
        for (int i = 0; i < rowLength; i++)
        {
            s += "\r\n";
            for (int j = 0; j < columnLength; j++)
            {
                s += " " + elements[i][j].ToString();
            }
        }
        return s;
    }
    internal Matrix GetRow(int row) // 获取第 row 行的行矩阵
    {
        Matrix m = new Matrix();
        var columnLength = elements[0].Length; // 列数
        m.elements = new float[1][];
        m.elements[0] = new float[columnLength];
        for (int i = 0; i < columnLength; i++)
        {
            m.elements[0][i] = elements[row][i];
        }
        return m;
    }
    internal Matrix GetRows(int[] rows)
    {
        Matrix m = new Matrix();
        m.elements = new float[rows.Length][];
        for (int i = 0; i < rows.Length; i++)
        {
            m.elements[i] = new float[columnLength];
            Array.Copy(elements[rows[i]], m.elements[i], columnLength);
        }
        return m;
    }
    internal Matrix GetRows(Matrix rows)
    {
        Matrix m = new Matrix();
        m.elements = new float[rows.rowLength][];
        for (int i = 0; i < rows.rowLength; i++)
        {
            m.elements[i] = new float[3];
            Array.Copy(elements[(int)rows[i][0]], m.elements[i], columnLength);
        }
        return m;
    }
    public Matrix ColumnExtend(Matrix m)
    {
        return Matrix.ColumnExtend(this, m);
    }
    internal Matrix GetColumn(int column)
    {
        column--; // 从自然数变为0开始的索引
        Matrix m = new Matrix();
        m.elements = new float[rowLength][];
        for (int i = 0; i < rowLength; i++)
        {
            m.elements[i] = new float[] { elements[i][column] };
        }
        return m;
    }
    internal Matrix GetColumns(Matrix columns)
    {
        throw new NotImplementedException();
    }
    internal void SetRows(Matrix rows, Matrix columns)
    {
        throw new NotImplementedException();
    }
    internal void SetRows(int[] index, Matrix row)
    {
        for (int i = 0; i < index.Length; i++)
        {
            this.elements[index[i]] = row.elements[i];
        }
    }
    internal void SetRow(int row, float value)
    {
        elements[row] = new float[] { value };
    }
    internal void SetRow(int rows, Matrix columns)
    {
        throw new NotImplementedException();
    }
    internal void SetVector3(int i, float dot)
    {
        throw new NotImplementedException();
        //elements[i] = new float
    }
    internal Vector3[] ToVector3Array()
    {
        throw new NotImplementedException();
    }
    internal Vector3 ToVector3()
    {
        throw new NotImplementedException();
    }
    internal Matrix rowVectorLength
    {
        get
        {
            Matrix m = new Matrix();
            m.elements = new float[rowLength][];
            for (int i = 0; i < rowLength; i++)
            {
                m.elements[i] = new float[] { GetVector3(i).magnitude };
            }
            return m;
        }
    }
    public Vector3 GetVector3(int row)
    {
        return new Vector3(elements[row][0], elements[row][1], elements[row][2]);
    }
    public float[] GetVector3Array(int row)
    {
        return new float[] { elements[row][0], elements[row][1], elements[row][2] };
    }
    public float[] GetVector3ArrayNormalized(int row)
    {
        var v3 = new Vector3(elements[row][0], elements[row][1], elements[row][2]);
        v3 = v3.normalized;
        return new float[] { v3.x, v3.y, v3.z };
    }
    public Matrix rowVectorNormalized
    {
        get
        {
            Matrix m = new Matrix();
            m.elements = new float[rowLength][];
            for (int i = 0; i < rowLength; i++)
            {
                m.elements[i] = GetVector3ArrayNormalized(i);
            }
            return m;
        }
    }
}
