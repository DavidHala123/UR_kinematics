using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ur_kinematics
{
    internal class matrix_math
    {
        public double[][] SubtractMatrices(double[][] matrixA, double[][] matrixB)
        {
            int rowsA = matrixA.Length;
            int colsA = matrixA[0].Length;
            int rowsB = matrixB.Length;
            int colsB = matrixB[0].Length;

            // Ensure both matrices have the same dimensions
            if (rowsA != rowsB || colsA != colsB)
            {
                throw new ArgumentException("Matrix dimensions must be the same for subtraction.");
            }

            // Create a new matrix to store the result
            double[][] result = new double[rowsA][];

            // Subtract the matrices element-wise
            for (int i = 0; i < rowsA; i++)
            {
                for (int j = 0; j < colsA; j++)
                {
                    result[i][j] = matrixA[i][j] - matrixB[i][j];
                }
            }

            return result;
        }

        public double[] CrossProduct(double[][] vectorA, double[][] vectorB)
        {
            if (vectorA.Length != 3 || vectorB.Length != 3 || vectorA[0].Length != 1 || vectorB[0].Length != 1)
            {
                throw new ArgumentException("Both vectors must have exactly 3 elements.");
            }

            double[] crossProduct = new double[3];

            crossProduct[0] = vectorA[1][0] * vectorB[2][0] - vectorA[2][0] * vectorB[1][0];
            crossProduct[1] = vectorA[2][0] * vectorB[0][0] - vectorA[0][0] * vectorB[2][0];
            crossProduct[2] = vectorA[0][0] * vectorB[1][0] - vectorA[1][0] * vectorB[0][0];

            return crossProduct;
        }


        public double[][] InvertMatrix(double[][] matrix)
        {
            int n = matrix.Length;

            // Initialize the identity matrix
            double[][] identity = new double[n][];
            for (int i = 0; i < n; i++)
            {
                identity[i] = new double[n];
                identity[i][i] = 1.0;  // Diagonal elements set to 1
            }

            // Create a copy of the original matrix
            double[][] augmented = new double[n][];
            for (int i = 0; i < n; i++)
            {
                augmented[i] = new double[n];
                for (int j = 0; j < n; j++)
                {
                    augmented[i][j] = matrix[i][j];
                }
            }

            // Perform Gaussian elimination
            for (int i = 0; i < n; i++)
            {
                // Pivot: find the largest value in the current column
                double maxElement = Math.Abs(augmented[i][i]);
                int maxRow = i;
                for (int k = i + 1; k < n; k++)
                {
                    if (Math.Abs(augmented[k][i]) > maxElement)
                    {
                        maxElement = Math.Abs(augmented[k][i]);
                        maxRow = k;
                    }
                }

                // Swap rows to place pivot on the diagonal
                double[] temp = augmented[i];
                augmented[i] = augmented[maxRow];
                augmented[maxRow] = temp;

                double[] tempIdentity = identity[i];
                identity[i] = identity[maxRow];
                identity[maxRow] = tempIdentity;

                // Make the pivot 1 by dividing the row by the pivot element
                double pivot = augmented[i][i];
                if (pivot == 0)
                {
                    throw new InvalidOperationException("Matrix is singular and cannot be inverted.");
                }
                for (int j = 0; j < n; j++)
                {
                    augmented[i][j] /= pivot;
                    identity[i][j] /= pivot;
                }

                // Eliminate other rows
                for (int k = 0; k < n; k++)
                {
                    if (k != i)
                    {
                        double factor = augmented[k][i];
                        for (int j = 0; j < n; j++)
                        {
                            augmented[k][j] -= factor * augmented[i][j];
                            identity[k][j] -= factor * identity[i][j];
                        }
                    }
                }
            }

            // The identity matrix now contains the inverse
            return identity;
        }


        public double[][] MultiplyMatrices(double[][] matrix1, double[][] matrix2)
        {
            int rows1 = matrix1.Length;
            int cols1 = matrix1[0].Length;
            int rows2 = matrix2.Length;
            int cols2 = matrix2[0].Length;

            if (cols1 != rows2)
            {
                throw new ArgumentException("Number of columns in matrix1 must match number of rows in matrix2.");
            }
            double[][] result = new double[rows1][];
            for (int i = 0; i < rows1; i++)
            {
                result[i] = new double[cols2];
            }

            for (int i = 0; i < rows1; i++)
            {
                for (int j = 0; j < cols2; j++)
                {
                    result[i][j] = 0;
                    for (int k = 0; k < cols1; k++)
                    {
                        result[i][j] += matrix1[i][k] * matrix2[k][j];
                    }
                }
            }

            return result;
        }
    }
}
