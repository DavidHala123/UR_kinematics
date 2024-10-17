using ur_kinematics;

//double[][] jointsPose = new double[][]
//{
//    new double[] { 0, 0, 0, 0, 0, 0 },  // Tx, Ty, Tz, Rx, Ry, Rz
//    new double[] { 0, 0, 1, 0, 0, 90 }, // A translation along z-axis, with 90° rotation around z-axis
//    new double[] { 1, 0, 0, 0, 0, 0 }   // A translation along x-axis, no rotation
//};

//double[][] rotationMatrix(double[] jointPose) 
//{
//    double[][] Rx = new double[3][]
//    {
//        new double[] { 1, 0, 0 },
//        new double[] { 0, Math.Cos(jointPose[3] * Math.PI / 180), -Math.Sin(jointPose[3] * Math.PI / 180) },
//        new double[] { 0, Math.Sin(jointPose[3] * Math.PI / 180), Math.Cos(jointPose[3] * Math.PI / 180) }
//    };
//    double[][] Ry = new double[3][]
//    {
//        new double[] { Math.Cos(jointPose[4] * Math.PI / 180), 0, Math.Sin(jointPose[4] * Math.PI / 180) },
//        new double[] { 0, 1, 0},
//        new double[] {-Math.Sin(jointPose[4] * Math.PI / 180), 0, Math.Cos(jointPose[4] * Math.PI / 180) }
//     };
//    double[][] Rz = new double[3][]
//    {
//        new double[] { Math.Cos(jointPose[5] * Math.PI / 180), -Math.Sin(jointPose[5] * Math.PI / 180), 0 },
//        new double[] { Math.Sin(jointPose[5] * Math.PI / 180), Math.Cos(jointPose[5] * Math.PI / 180), 0},
//        new double[] { 0, 0, 1 }
//    };

//    double[][] resultRzy = MultiplyMatrices(Rz, Ry);

//    double[][] resultRzyx = MultiplyMatrices(resultRzy, Rx);

//    return resultRzyx;
//}

//static double[][] MultiplyMatrices(double[][] matrix1, double[][] matrix2)
//{
//    int rows1 = matrix1.Length;
//    int cols1 = matrix1[0].Length;
//    int rows2 = matrix2.Length;
//    int cols2 = matrix2[0].Length;

//    if (cols1 != rows2)
//    {
//        throw new ArgumentException("Number of columns in matrix1 must match number of rows in matrix2.");
//    }
//    double[][] result = new double[rows1][];
//    for (int i = 0; i < rows1; i++)
//    {
//        result[i] = new double[cols2];
//    }

//    for (int i = 0; i < rows1; i++)
//    {
//        for (int j = 0; j < cols2; j++)
//        {
//            result[i][j] = 0;
//            for (int k = 0; k < cols1; k++)
//            {
//                result[i][j] += matrix1[i][k] * matrix2[k][j];
//            }
//        }
//    }

//    return result;
//}

//double[][] homogenious_t(double[][] rMatrix, double[] tMatrix) 
//{
//    double[][] matA = new double[][]
//    {
//        new double[]{0,0,0,0 },
//        new double[]{0,0,0,0 },
//        new double[]{0,0,0,0 },
//        new double[]{0,0,0,0 }
//    };

//    matA[0][0] = rMatrix[0][0];
//    matA[0][1] = rMatrix[0][1];
//    matA[0][2] = rMatrix[0][2];

//    matA[1][0] = rMatrix[1][0];
//    matA[1][1] = rMatrix[1][1];
//    matA[1][2] = rMatrix[1][2];

//    matA[2][0] = rMatrix[2][0];
//    matA[2][1] = rMatrix[2][1];
//    matA[2][2] = rMatrix[2][2];

//    matA[0][3] = tMatrix[0];
//    matA[1][3] = tMatrix[1];
//    matA[2][3] = tMatrix[2];

//    matA[3][0] = 0;
//    matA[3][1] = 0;
//    matA[3][2] = 0;
//    matA[3][3] = 1;

//    return matA;
//}


//double[][] forward_kin(double[][] jointsPose) 
//{
//    double[][] matA = homogenious_t(rotationMatrix(jointsPose[0]), new double[] { jointsPose[0][0], jointsPose[0][1], jointsPose[0][2] });

//    for (int i = 1; i < jointsPose.Length; i++) 
//    {
//        double[][] matB = homogenious_t(rotationMatrix(jointsPose[i]), new double[] { jointsPose[i][0], jointsPose[i][1], jointsPose[i][2] });

//        matA = MultiplyMatrices(matA, matB);
//    }

//    return matA;
//}

void interpretResult(double[][] inpT, double tolerance)
{
    double[][] intOut = inpT;
    for (int i = 0; i < inpT.Length; i++)
    {
        for (int j = 0; j < inpT[1].Length; j++)
        {
            if (Math.Abs(inpT[i][j]) < tolerance)
                intOut[i][j] = 0;
            else
                intOut[i][j] = inpT[i][j];
        }
    }

    Console.WriteLine($"{intOut[0][0]} {intOut[0][1]} {intOut[0][2]} {intOut[0][3]}");
    Console.WriteLine($"{intOut[1][0]} {intOut[1][1]} {intOut[1][2]} {intOut[1][3]}");
    Console.WriteLine($"{intOut[2][0]} {intOut[2][1]} {intOut[2][2]} {intOut[2][3]}");
    Console.WriteLine($"{intOut[3][0]} {intOut[3][1]} {intOut[3][2]} {intOut[3][3]}");
}

robot_kinematics rk = new robot_kinematics(RobotType.UR20);
//double[] pose = rk.forward_kin([30, 0, 40, 25, 40, 36]);
double[] jointRot = rk.inverse_kin([100, 100, 50, 30, 60, 30]);
//Console.WriteLine($"{pose[0]} {pose[1]} {pose[2]}, {pose[3]} {pose[4]} {pose[5]}");
