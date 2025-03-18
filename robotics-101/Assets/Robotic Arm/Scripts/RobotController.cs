using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
public class RobotController
{
    private TcpClient client;
    private NetworkStream stream;
    public string robotIp = "192.168.3.11";  // Robot IP
    public int port = 3920;   // CRI Port

    private float sendCount = 1;

    public RobotController(string robotIp, int port)
    {
        this.robotIp = robotIp;
        this.port = port;
    }

    private bool isConnected = false;

    public async Task ConnectToRobot()
    {
        try
        {
            client = new TcpClient();
            await client.ConnectAsync(robotIp, port);
            stream = client.GetStream();
            isConnected = true;
            Debug.Log("Connected to robot controller.");
            _ = Task.Run(() => StartListeningForDataAsync());
        }
        catch (Exception ex)
        {
           Debug.LogError($"Connection failed: {ex.Message}");
        }
    }

    public bool IsConnected()
    {
        return isConnected;
    }

    private async Task StartListeningForDataAsync()
    {
        byte[] buffer = new byte[1024];
        try
        {
            while (isConnected && stream != null)
            {
                int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string incomingData = Encoding.ASCII.GetString(buffer, 0, bytesRead);
                    if (incomingData.Contains("POSJOINTCURRENT") || incomingData.Contains("FRAMEROBOT #base"))
                    {
                        //Debug.Log("Incoming Data: " + incomingData);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Debug.Log("Error reading incoming data: " + ex.Message);
        }
    }

    public void SendJointCommand(float[] jointAngles, float velocity)
    {
        if (!isConnected) return;

        string userCommand = string.Format("CMD Move Joint {0} {1} {2} {3} {4} {5} 0 0 0 {6}",
            (-jointAngles[0]).ToString("F2", CultureInfo.InvariantCulture),
            jointAngles[1].ToString("F2", CultureInfo.InvariantCulture),
            jointAngles[2].ToString("F2", CultureInfo.InvariantCulture),
            jointAngles[3].ToString("F2", CultureInfo.InvariantCulture),
            jointAngles[4].ToString("F2", CultureInfo.InvariantCulture),
            jointAngles[5].ToString("F2", CultureInfo.InvariantCulture),
            velocity.ToString("F2", CultureInfo.InvariantCulture));

        SendCommand(userCommand);
    }

    public void ResetAndEnable()
    {
        if (!isConnected) return;

        SendResetCommand();
        SendEnableCommand();
    }

    private void SendEnableCommand()
    {
        string userCommand = "CMD Enable";
        SendCommand(userCommand);
        }

    private void SendResetCommand()
    {
        string userCommand = "CMD Reset";
        SendCommand(userCommand);
    }

    public void SendCommand(string cmd)
    {
        if (!isConnected) return;

        string msg = "CRISTART " + sendCount.ToString() + " " + cmd + " CRIEND";
        try
        {
            byte[] outStream = Encoding.ASCII.GetBytes(msg);
            stream.Write(outStream, 0, outStream.Length);
            stream.Flush();

            sendCount++;
            if (sendCount >= 10000)
                sendCount = 1;

            Debug.Log(msg);
        }
        catch (Exception) { }
    }

    public void Disconnect()
    {
        if (stream != null)
        {
            stream.Close();
            stream.Dispose();
            stream = null;
        }

        if (client != null)
        {
            client.Close();
            client.Dispose();
            client = null;
        }

        isConnected = false;
        Debug.Log("Disconnected from robot.");
    }
}
