using ur_kinematics;

robot_kinematics rk = new robot_kinematics(RobotType.UR20);

// Initialization {TCP/IP Universal Robots}
//  Read Data:
UR_Stream_Data.ip_address = "192.168.114.129";
//  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms)
UR_Stream_Data.time_step = 8;
//  Write Data:
UR_Control_Data.ip_address = "192.168.114.129";
//  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms)
UR_Control_Data.time_step = 8;

// Start Stream {Universal Robots TCP/IP}
UR_Stream ur_stream_robot = new UR_Stream();
ur_stream_robot.Start();

// Start Control {Universal Robots TCP/IP}
UR_Control ur_ctrl_robot = new UR_Control();
ur_ctrl_robot.Start();



double[][] jointRot = rk.inverse_kin([-0.144, -0.436, 0.215, 0.76, -2.7, -2.06]);
UR_Control_Data.J_Orientation = [jointRot[0][0] * 180 / Math.PI, jointRot[1][0] * 180 / Math.PI, jointRot[2][0] * 180 / Math.PI, jointRot[3][0] * 180 / Math.PI, jointRot[4][0] * 180 / Math.PI, jointRot[5][0] * 180 / Math.PI];
UR_Control_Data.acceleration = "10";
UR_Control_Data.velocity = "200";


Console.WriteLine("[INFO] Stop (y):");
// Stop communication
string stop_rs = Convert.ToString(Console.ReadLine());
if (stop_rs == "y")
{

    ur_stream_robot.Destroy();
    ur_ctrl_robot.Destroy();

    // Application quit
    Environment.Exit(0);
}