using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations.Schema;
using System.Data;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Text;
using System.Threading.Tasks;
using System.Transactions;
using mitoSoft.Matrices;
namespace ur_kinematics
{
    public enum RobotType
    {
        UR10,
        UR20
    }

    internal class robot_kinematics
    {

        private double[] theta;
        private double[] alpha;
        private double[] a;
        private double[] d;
        private List<double[][]> t_matrices = new List<double[][]>();
        double[][] eye_matrix = new double[3][]
        {
            new double[] {0,0,0},
            new double[] {0,0,0},
            new double[] {0,0,0}
        };
        double[][] m0 = new double[3][]
        {
            new double[] {0},
            new double[] {0},
            new double[] {0},
        };
        double[][] m1 = new double[3][]
        {
            new double[] {0},
            new double[] {0},
            new double[] {1},
        };
        matrix_math matrixM;

        public robot_kinematics(RobotType type)
        {
            switch (type)
            {
                case RobotType.UR20:
                    this.theta = new double[6] { 0, 0, 0, 0, 0, 0 };
                    this.alpha = new double[6] { Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0 };
                    this.a = new double[6] { 0, -0.8620, -0.7287, 0, 0, 0 };
                    this.d = new double[6] { 0.2363, 0, 0, 0.2010, 0.1593, 0.1543 };
                    break;
            }

            matrixM = new matrix_math();
        }

        private double[][] get_homog_t(double[] in_vector)
        {
            double[][] out_transform = new double[4][]
            {
                new double[4]{0,0,0,0},
                new double[4]{0,0,0,0},
                new double[4]{0,0,0,0},
                new double[4]{0,0,0,0}
            };
            double[][] Rz = new double[3][]
            {
                new double[3]{ Math.Cos(in_vector[3]), -Math.Sin(in_vector[3]), 0},
                new double[3]{ Math.Sin(in_vector[3]), Math.Cos(in_vector[3]), 0},
                new double[3]{ 0, 0, 1}
            };
            double[][] Ry = new double[3][]
            {
                new double[3]{ Math.Cos(in_vector[4]), 0, Math.Sin(in_vector[4])},
                new double[3]{ 0, 1, 0},
                new double[3]{ -Math.Sin(in_vector[4]), 0, Math.Cos(in_vector[4]) }
            };
            double[][] Rx = new double[3][]
            {
                new double[3]{ 1, 0, 0},
                new double[3]{ 0, Math.Cos(in_vector[5]), -Math.Sin(in_vector[5]) },
                new double[3]{ 0, Math.Sin(in_vector[5]), Math.Cos(in_vector[5]) }
            };

            var Rzy = matrixM.MultiplyMatrices(Rz, Ry);
            double[][] Rot_matrix = matrixM.MultiplyMatrices(Rzy, Rx);

            out_transform[0][0] = Rot_matrix[0][0];
            out_transform[0][1] = Rot_matrix[0][1];
            out_transform[0][2] = Rot_matrix[0][2];
            out_transform[0][3] = in_vector[0];

            out_transform[1][0] = Rot_matrix[1][0];
            out_transform[1][1] = Rot_matrix[1][1];
            out_transform[1][2] = Rot_matrix[1][2];
            out_transform[1][3] = in_vector[1];

            out_transform[2][0] = Rot_matrix[2][0];
            out_transform[2][1] = Rot_matrix[2][1];
            out_transform[2][2] = Rot_matrix[2][2];
            out_transform[2][3] = in_vector[2];

            out_transform[3][3] = 1.0;


            return out_transform;
        }

        double[][] DH_matrix(double theta, double alpha, double a, double d)
        {
            return new double[4][]
            {
                new double[] { Math.Cos(theta), -Math.Sin(theta) * Math.Cos(alpha), Math.Sin(theta) * Math.Sin(alpha), a * Math.Cos(theta) },
                new double[] { Math.Sin(theta), Math.Cos(theta) * Math.Cos(alpha), -Math.Cos(theta) * Math.Sin(alpha), a * Math.Sin(theta) },
                new double[] { 0, Math.Sin(alpha), Math.Cos(alpha), d },
                new double[] { 0, 0, 0, 1}
            };
        }








        private double[] get_pose_vector(double[][] homog_t)
        {
            double rot_y = Math.Atan2(Math.Sqrt(Math.Pow(homog_t[2][1], 2) + Math.Pow(homog_t[2][0], 2)), -homog_t[2][0]);
            double rot_x = Math.Atan2(homog_t[0][0] / Math.Cos(rot_y), homog_t[1][0] / Math.Cos(rot_y));
            double rot_z = Math.Atan2(homog_t[2][2] / Math.Cos(rot_y), homog_t[2][1] / Math.Cos(rot_y));
            return [homog_t[0][3], homog_t[1][3], homog_t[2][3], rot_x * 180 / Math.PI, rot_y * 180 / Math.PI, rot_z * 180 / Math.PI];
        }





        public double[] forward_kin(double[] jointsRot_deg)
        {
            double[][] matA = DH_matrix((jointsRot_deg[0] * Math.PI / 180) + theta[0], alpha[0], a[0], d[0]);

            for (int i = 1; i < theta.Length; i++)
            {
                double[][] matB = DH_matrix((jointsRot_deg[i] * Math.PI / 180) + theta[i], alpha[i], a[i], d[i]);

                matA = matrixM.MultiplyMatrices(matA, matB);
            }

            return get_pose_vector(matA);
        }

        private double[][] getTranslation_homogT(double[][] homogT)
        {
            return new double[][]
            {
                new double[] { homogT[0][3] },
                new double[] { homogT[1][3] },
                new double[] { homogT[2][3] }
            };
        }

        private double[][] getRotation_homogT(double[][] homogT)
        {
            return new double[][]
            {
                new double[] { homogT[0][0], homogT[0][1], homogT[0][2] },
                new double[] { homogT[1][0], homogT[1][1], homogT[1][2] },
                new double[] { homogT[2][0], homogT[2][1], homogT[2][2] }
            };
        }


        //https://people.ohio.edu/williams/html/PDF/UniversalRobotKinematics.pdf

        public double[][] inverse_kin(double[] endPoint)
        {
            endPoint[3] = endPoint[3] * Math.PI / 180;
            endPoint[4] = endPoint[4] * Math.PI / 180;
            endPoint[5] = endPoint[5] * Math.PI / 180;

            double[][] jointAngles = new double[theta.Length][];

            double[][] endPoint_transform = new double[theta.Length][];

            endPoint_transform = get_homog_t(endPoint);
            var x6 = -endPoint_transform[0][3];
            var y6 = endPoint_transform[1][3];
            var d6 = d[3];


            double[] th12 = [-x6 + Math.Sqrt(y6 * y6 + x6 * x6 - d6 * d6) / (d6 - y6), -x6 - Math.Sqrt(y6 * y6 + x6 * x6 - d6 * d6) / (d6 - y6)];
            double[] theta1 = [2 * Math.Atan(th12[0]), 2 * Math.Atan(th12[1])]; //THETA1 has 2 options

            //-----------------

            var th6_1_th0 = endPoint_transform[0][1] * Math.Sin(theta1[0]) - endPoint_transform[1][1] * Math.Cos(theta1[0]);
            var th6_2_th0 = endPoint_transform[1][0] * Math.Cos(theta1[0]) - endPoint_transform[0][0] * Math.Sin(theta1[0]);
            var th6_1_th1 = endPoint_transform[0][1] * Math.Sin(theta1[1]) - endPoint_transform[1][1] * Math.Cos(theta1[1]);
            var th6_2_th1 = endPoint_transform[1][0] * Math.Cos(theta1[1]) - endPoint_transform[0][0] * Math.Sin(theta1[1]);

            double[] theta6 = [Math.Atan2(th6_1_th0, th6_2_th0), Math.Atan2(th6_1_th1, th6_2_th1)];

            //------------------

            var th5_1_th0 = (endPoint_transform[1][0] * Math.Cos(theta1[0]) - endPoint_transform[0][0] * Math.Sin(theta1[0])) * Math.Cos(theta6[0]) + (endPoint_transform[0][1] * Math.Sin(theta1[0]) - endPoint_transform[1][1] * Math.Cos(theta1[0])) * Math.Sin(theta6[0]);
            var th5_2_th0 = endPoint_transform[0][2] * Math.Sin(theta1[0]) - endPoint_transform[1][2] * Math.Cos(theta1[0]);

            var th5_1_th1 = (endPoint_transform[1][0] * Math.Cos(theta1[1]) - endPoint_transform[0][0] * Math.Sin(theta1[1])) * Math.Cos(theta6[1]) + (endPoint_transform[0][1] * Math.Sin(theta1[1]) - endPoint_transform[1][1] * Math.Cos(theta1[1])) * Math.Sin(theta6[1]);
            var th5_2_th1 = endPoint_transform[0][2] * Math.Sin(theta1[1]) - endPoint_transform[1][2] * Math.Cos(theta1[1]);

            double[] theta5 = [Math.Atan2(th5_1_th0, th5_2_th0), Math.Atan2(th5_1_th1, th5_2_th1)];

            //-------------

            var A_1 = (endPoint_transform[2][0] * Math.Cos(theta6[0]) - endPoint_transform[2][1] * Math.Sin(theta6[0])) / Math.Cos(theta5[0]);
            var A_2 = (endPoint_transform[2][0] * Math.Cos(theta6[1]) - endPoint_transform[2][1] * Math.Sin(theta6[1])) / Math.Cos(theta5[1]);

            var B_1 = endPoint_transform[2][1] * Math.Cos(theta6[0]) + endPoint_transform[2][0] * Math.Sin(theta6[0]);
            var B_2 = endPoint_transform[2][1] * Math.Cos(theta6[1]) + endPoint_transform[2][0] * Math.Sin(theta6[1]);

            var alp_1 = -endPoint_transform[0][3] * Math.Cos(theta1[0]) - endPoint_transform[1][3] * Math.Sin(theta1[0]) - d[4] * A_1;
            var alp_2 = -endPoint_transform[0][3] * Math.Cos(theta1[1]) - endPoint_transform[1][3] * Math.Sin(theta1[1]) - d[4] * A_2;

            var bet_1 = endPoint_transform[2][3] - d[4] * B_1;
            var bet_2 = endPoint_transform[2][3] - d[4] * B_2;

            var E_1 = -2 * a[1] * bet_1;
            var E_2 = -2 * a[1] * bet_2;

            var F_1 = -2 * a[1] * alp_1;
            var F_2 = -2 * a[1] * alp_2;


            var G_1 = Math.Pow(a[1], 2) + Math.Pow(alp_1, 2) + Math.Pow(bet_1, 2) - Math.Pow(a[2], 2);
            var G_2 = Math.Pow(a[1], 2) + Math.Pow(alp_2, 2) + Math.Pow(bet_2, 2) - Math.Pow(a[2], 2);

            double[] th23 = [-F_1 + Math.Sqrt(E_1 * E_1 + F_1 * F_1 - G_1 * G_1) / (G_1 - E_1), -F_1 - Math.Sqrt(E_1 * E_1 + F_1 * F_1 - G_1 * G_1) / (G_1 - E_1),
                             -F_2 + Math.Sqrt(E_2 * E_2 + F_2 * F_2 - G_2 * G_2) / (G_2 - E_2), -F_2 - Math.Sqrt(E_2 * E_2 + F_2 * F_2 - G_2 * G_2) / (G_2 - E_2)];

            double[] theta2 = [2 * Math.Atan(th23[0]), 2 * Math.Atan(th23[1]), 2 * Math.Atan(th23[2]), 2 * Math.Atan(th23[3])];


            //-----------------

            var th3_1_a = alp_1 - a[1] * Math.Sin(theta2[0]);
            var th3_1_b = bet_1 - a[1] * Math.Cos(theta2[0]);

            var th3_2_a = alp_1 - a[1] * Math.Sin(theta2[1]);
            var th3_2_b = bet_1 - a[1] * Math.Cos(theta2[1]);

            var th3_3_a = alp_2 - a[1] * Math.Sin(theta2[2]);
            var th3_3_b = bet_2 - a[1] * Math.Cos(theta2[2]);

            var th3_4_a = alp_2 - a[1] * Math.Sin(theta2[3]);
            var th3_4_b = bet_2 - a[1] * Math.Cos(theta2[3]);

            double[] theta3 = [Math.Atan2(th3_1_a, th3_1_b) - theta2[0], Math.Atan2(th3_2_a, th3_2_b) - theta2[1], Math.Atan2(th3_3_a, th3_3_b) - theta2[2], Math.Atan2(th3_4_a, th3_4_b) - theta2[3]];

            //-----------------

            double[] theta4 = [Math.Atan2(A_1, B_1) - theta2[0] - theta3[0], Math.Atan2(A_1, B_1) - theta2[1] - theta3[1], Math.Atan2(A_2, B_2) - theta2[2] - theta3[2], Math.Atan2(A_2, B_2) - theta2[3] - theta3[3]];

            jointAngles[0] = new double[theta1.Length];
            jointAngles[1] = new double[theta2.Length];

            jointAngles[2] = new double[theta3.Length];
            jointAngles[3] = new double[theta4.Length];

            jointAngles[4] = new double[theta5.Length];
            jointAngles[5] = new double[theta6.Length];

            jointAngles[0] = theta1;
            jointAngles[1] = theta2;
            jointAngles[2] = theta3;
            jointAngles[3] = theta4;
            jointAngles[4] = theta5;
            jointAngles[5] = theta6;

            return jointAngles;
        }
    }
}
