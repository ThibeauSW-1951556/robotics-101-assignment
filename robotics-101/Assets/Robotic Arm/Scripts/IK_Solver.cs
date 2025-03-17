using NUnit.Framework;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class IK_Solver : MonoBehaviour
{
    public enum Axis { X, Y, Z };


    [Header("Joint Pivots")]
    public GameObject joint1Pivot;
    public GameObject joint2Pivot; 
    public GameObject joint3Pivot;
    public GameObject joint4Pivot;
    public GameObject joint5Pivot;
    public GameObject joint6Pivot;
    
    [Space(10)] // Adds 10 pixels of space in the inspector
    [Header("Arm Lengths")]
    public float L1;
    public float L2;
    public float L3;
    public float L4;

    [Space(10)] // Adds 10 pixels of space in the inspector
    [Header("Control Settings")]
    public float Speed;
    public float AngleOfAttack;

    [Space(10)] // Adds 10 pixels of space in the inspector
    [Header("Target Settings")]
    public GameObject target;

    [Space(10)] // Adds 10 pixels of space in the inspector
    [Header("Connection Settings")]
    [Tooltip("Check this before running to ensure the robot is connected.")]
    public bool PhysicalConnection = false;
    public string RobotIP = "192.168.3.11";  // Robot IP
    public int RobotPort = 3920;
    public bool ResetAndEnableRobot = false;


    private readonly float[] currentAngles = new float[6]; // Array for angles initialized with 0
    private readonly float[] currentSpeeds = new float[6]; // Array for speeds initialized with 0

    private readonly float maxSpeed = 40f;  // The max acceleration in degrees per second
    private readonly float acceleration = 8f;       // Acceleration in degrees per second squared


    private IK_Calculator iK_Calculator;
    private RobotController robotController;


    private bool isSingularityWarning = false;
    private Dictionary<int, float> lastWarningAngles = new Dictionary<int, float>();

    private Vector3 last3D = Vector3.zero;

    private float[] targetAngles = new float[6];


    void Start()
    {
        if (PhysicalConnection) StartCoroutine(StartRobotConnection());

        bool jointPivotsAreValid = ValidatePivotObjects();

        if (!jointPivotsAreValid) Quit();

        if (Speed <= 0f)
        {
            Debug.LogError("Please fill in a Speed value in the UI.");
            Quit();
        }

        if (target == null)
        {
            Debug.LogError("Please assign the Target to a valid GameObject (prefer Sphere).");
            Quit();
        }

        for (int i = 0; i < currentAngles.Length; i++)
        {
            currentAngles[i] = 0f;
            currentSpeeds[i] = 0f;
        }

        iK_Calculator = new IK_Calculator(L1, L2, L3);
    }

    private void OnApplicationQuit()
    {
        robotController.Disconnect();
    }

    private IEnumerator StartRobotConnection()
    {
        robotController = new RobotController(robotIp: RobotIP, port: RobotPort);

        var connectTask = robotController.ConnectToRobot();

        while (!connectTask.IsCompleted)
        {
            yield return null; // Wait until the task is completed
        }
    }


    // Update is called once per frame
    void Update()
    {
        if (robotController != null && ResetAndEnableRobot)
        {
            robotController.ResetAndEnable();
            ResetAndEnableRobot = false;
        }

        var target3D = target.transform.position;
        var actual3D = Calculate3DofLocation(target3D);

        var targetJointAngles = iK_Calculator.Calculate(actual3D);

        var currentJointAngles = new Vector3(joint1Pivot.transform.localEulerAngles.y, joint2Pivot.transform.localEulerAngles.x, joint3Pivot.transform.localEulerAngles.x);

        targetJointAngles = CheckOutOfBoundsSingularity(currentJointAngles, targetJointAngles);

        MoveJoint(joint1Pivot, 0, -179f, 179f, targetJointAngles.x, Axis.Y);
        MoveJoint(joint2Pivot, 1, -80, 140, targetJointAngles.y, Axis.X);
        MoveJoint(joint3Pivot, 2, -80, 140, targetJointAngles.z, Axis.X);

        float jointAngle5 = (AngleOfAttack + 90) - targetJointAngles.y - targetJointAngles.z;

        MoveJoint(joint5Pivot, 4, -95, 95, jointAngle5, Axis.X);

        if (robotController != null && (last3D == Vector3.zero || last3D != actual3D))
        {
            robotController.SendJointCommand(targetAngles, Speed);
            last3D = actual3D;
        }


    }

    /// <summary>
    /// Moves a specified joint between a minimum and maximum angle, applying acceleration and deceleration.
    /// </summary>
    /// <param name="joint">The joint GameObject to move.</param>
    /// <param name="minAngle">The minimum angle the joint can move to.</param>
    /// <param name="maxAngle">The maximum angle the joint can move to.</param>
    private void MoveJoint(GameObject joint, int jointIndex, float minAngle, float maxAngle, float targetAngle, Axis axis)
    {
        targetAngle = ClampAngle(jointIndex, targetAngle, minAngle, maxAngle);

        targetAngles[jointIndex] = targetAngle;

        bool movingForward = currentAngles[jointIndex] < targetAngle;

        float currentAngle = currentAngles[jointIndex];
        float currentSpeed = currentSpeeds[jointIndex];

        // Calculate the distance to the target
        float distanceToTarget = Mathf.Abs(targetAngle - currentAngle);

        float decelerationDistance = CalculateDecelerationDistance(currentSpeed, acceleration);

        // Adjust speed based on distance (accelerate and decelerate)
        if (distanceToTarget < decelerationDistance) // Decelerate when close to the target
        {
            currentSpeed = Mathf.Max(currentSpeed - acceleration * Time.deltaTime, 0);
        }
        else // Accelerate when far from the target
        {
            currentSpeed = Mathf.Min(currentSpeed + acceleration * Time.deltaTime, Speed);
        }

        // Ensure the speed stays within the valid range
        currentSpeed = ClampSpeed(currentSpeed);

        // Calculate the amount to rotate this frame
        float rotationThisFrame = currentSpeed * Time.deltaTime;

        // Ensure we don't overshoot the target
        if (rotationThisFrame > distanceToTarget) rotationThisFrame = distanceToTarget;

        // Update the current angle based on the rotation
        currentAngle += movingForward ? rotationThisFrame : -rotationThisFrame;

        UpdateJointRotation(joint, currentAngle, axis);

        currentAngles[jointIndex] = currentAngle;
        currentSpeeds[jointIndex] = currentSpeed;
    }

    /// <summary>
    /// Updates the rotation of a GameObject on the specified axis.
    /// </summary>
    /// <param name="joint">The GameObject whose rotation will be updated.</param>
    /// <param name="currentAngle">The current angle to set on the specified axis.</param>
    /// <param name="axis">The axis on which to apply the rotation (X, Y, or Z).</param>
    private void UpdateJointRotation(GameObject joint, float currentAngle, Axis axis)
    {
        Vector3 newRotation = joint.transform.localRotation.eulerAngles;

        switch (axis)
        {
            case Axis.X:
                newRotation.x = currentAngle;
                newRotation.y = 0;
                newRotation.z = 0;
                break;
            case Axis.Y:
                newRotation.x = 0;
                newRotation.y = currentAngle;
                newRotation.z = 0;

                break;
            case Axis.Z:
                newRotation.x = 0;
                newRotation.y = 0;
                newRotation.z = currentAngle;
                break;
        }

        joint.transform.localRotation = Quaternion.Euler(newRotation);
    }

    /// <summary>
    /// Calculates the degrees necessary to decelerate from a given speed to 0.
    /// </summary>
    /// <param name="speed">The current speed in degrees per second.</param>
    /// <param name="deceleration">The deceleration factor in degrees per second squared.</param>
    /// <returns>The degrees needed to decelerate to 0.</returns>
    private float CalculateDecelerationDistance(float speed, float deceleration)
    {
        if (deceleration <= 0)
        {
            Debug.LogError("Deceleration must be greater than 0.");
            return 0f;
        }

        return (speed * speed) / (2 * deceleration);
    }

    /// <summary>
    /// Clamps the speed between 0 and the maximum speed.
    /// </summary>
    /// <param name="speed">The current speed to clamp.</param>
    /// <returns>The clamped speed.</returns>
    private float ClampSpeed(float speed)
    {
        return Mathf.Clamp(speed, 0.0f, maxSpeed);
    }

    /// <summary>
    /// Clamps the joint's angle between the specified minimum and maximum achievable angles.
    /// Logs warnings only once when the angle exceeds its limits.
    /// </summary>
    /// <param name="jointIndex">The index of the joint being clamped.</param>
    /// <param name="angle">The current angle of the joint.</param>
    /// <param name="minAngle">The minimum allowed angle.</param>
    /// <param name="maxAngle">The maximum allowed angle.</param>
    /// <returns>The clamped angle within the specified range.</returns>
    private float ClampAngle(int jointIndex, float angle, float minAngle, float maxAngle)
    {
        float clampedAngle = Mathf.Clamp(angle, minAngle, maxAngle);

        // Check if the joint is outside the valid range and log only once per limit
        if (angle < minAngle)
        {
            if (!lastWarningAngles.ContainsKey(jointIndex) || lastWarningAngles[jointIndex] != minAngle)
            {
                Debug.LogWarning($"[Joint {jointIndex}] Angle {angle}° is below the minimum limit ({minAngle}°). Clamping to {minAngle}°.");
                lastWarningAngles[jointIndex] = minAngle;
            }
        }
        else if (angle > maxAngle)
        {
            if (!lastWarningAngles.ContainsKey(jointIndex) || lastWarningAngles[jointIndex] != maxAngle)
            {
                Debug.LogWarning($"[Joint {jointIndex}] Angle {angle}° exceeds the maximum limit ({maxAngle}°). Clamping to {maxAngle}°.");
                lastWarningAngles[jointIndex] = maxAngle;
            }
        }
        else
        {
            // If the joint is within range, remove it from the warning dictionary
            if (lastWarningAngles.ContainsKey(jointIndex))
            {
                lastWarningAngles.Remove(jointIndex);
            }
        }

        return clampedAngle;
    }

    /// <summary>
    /// Validates all pivot GameObjects by ensuring they are assigned and named correctly.
    /// </summary>
    /// <returns>True if all pivots are valid, false otherwise.</returns>
    private bool ValidatePivotObjects()
        {
            bool joint1PivotIsValid = validatePivotObject(joint1Pivot, "joint-1-pivot");
            bool joint2PivotIsValid = validatePivotObject(joint2Pivot, "joint-2-pivot");
            bool joint3PivotIsValid = validatePivotObject(joint3Pivot, "joint-3-pivot");
            bool joint4PivotIsValid = validatePivotObject(joint4Pivot, "joint-4-pivot");
            bool joint5PivotIsValid = validatePivotObject(joint5Pivot, "joint-5-pivot");
            bool joint6PivotIsValid = validatePivotObject(joint6Pivot, "joint-6-pivot");

            return joint1PivotIsValid && joint2PivotIsValid && joint3PivotIsValid && joint4PivotIsValid && joint5PivotIsValid && joint6PivotIsValid;
        }

    /// <summary>
    /// Validates a single pivot GameObject.
    /// Ensures the GameObject is assigned and its name matches the expected value.
    /// Logs errors to the console if validation fails.
    /// </summary>
    /// <param name="obj">The GameObject to validate.</param>
    /// <param name="expectedName">The expected name of the GameObject.</param>
    /// <returns>True if the GameObject is assigned and named correctly, false otherwise.</returns>
    private bool validatePivotObject(GameObject obj, string expectedName)
    {
        if (obj == null)
        {
            Debug.LogError($"The GameObject field is not assigned in the inspector for expected name: {expectedName}");
            return false;
        }
        else if (obj.name != expectedName)
        {
            Debug.LogError($"The GameObject '{obj.name}' does not match the expected name: {expectedName}");
            return false;
        }

        Debug.Log($"GameObject '{obj.name}' is properly assigned and matches the expected name.");
        return true;
    }


    /// <summary>
    /// Checks the target joint angles for NaN values. If any angle is NaN, the robotic arm stops by retaining the current joint angles.
    /// </summary>
    /// <param name="currentJointAngles">The current joint angles of the robotic arm.</param>
    /// <param name="targetJointAngles">The target joint angles to be checked and potentially applied.</param>
    /// <returns>The updated joint angles if valid, otherwise the current joint angles to stop movement.</returns>
    private Vector3 CheckOutOfBoundsSingularity(Vector3 currentJointAngles, Vector3 targetJointAngles)
    {
        // Check if any of the target joint angles are NaN
        if (float.IsNaN(targetJointAngles.x) || float.IsNaN(targetJointAngles.y) || float.IsNaN(targetJointAngles.z))
        {
            if (!isSingularityWarning)
            {
                // Log a warning and retain current joint angles
                Debug.LogWarning("Detected NaN in target joint angles. Retaining current joint angles to prevent movement.");
            }

            isSingularityWarning = true;
            return currentJointAngles;
        }

        isSingularityWarning = false;

        // If all angles are valid, return the target joint angles
        return targetJointAngles;
    }


    /// <summary>
    /// Quits the program.
    /// </summary>
    private void Quit()
    {
        #if UNITY_EDITOR
                UnityEditor.EditorApplication.isPlaying = false;
#else
                Application.Quit();
               
#endif
    }

    /// <summary>
    /// Calculates the 3-DOF (Degrees of Freedom) location after applying rotations around the XY and XZ planes.
    /// </summary>
    /// <param name="target3D">The initial target position in 3D space.</param>
    /// <returns>The final calculated position after applying rotations.</returns>
    private Vector3 Calculate3DofLocation(Vector3 target3D)
    {
        // Adjust the Y-position by adding the L4 offset
        Vector3 tempTargetPoint = new Vector3(target3D.x, target3D.y + L4, target3D.z);

        Vector3 startPoint = target3D;    // Pivot point for rotation
        Vector3 targetPoint = tempTargetPoint;   // Point to rotate

        // Calculate the base rotation angle using the Z and X coordinates
        var baseRotation = CalculateTheta(tempTargetPoint.z, tempTargetPoint.x);

        double angleZY = 90 - AngleOfAttack; // Rotation around the XY plane (Z-axis)
        double angleXZ = baseRotation;       // Rotation around the XZ plane (Y-axis)

        // First rotation in the XY-plane (ZY perspective)
        Vector3 rotatedZY = RotateAroundXY(targetPoint, startPoint, angleZY);

        // Second rotation in the XZ-plane
        return RotateAroundXZ(rotatedZY, startPoint, angleXZ);
    }

    /// <summary>
    /// Rotates a point around another point in the XY-plane (rotation around the Z-axis).
    /// </summary>
    /// <param name="point">The point to rotate.</param>
    /// <param name="pivot">The pivot point around which to rotate.</param>
    /// <param name="angleDegrees">The angle of rotation in degrees.</param>
    /// <returns>The rotated point as a <see cref="Vector3"/>.</returns>
    private Vector3 RotateAroundXY(Vector3 point, Vector3 pivot, double angleDegrees)
    {
        double angleRadians = angleDegrees * Math.PI / 180.0;

        // Translate to origin
        double translatedX = point.x - pivot.x;
        double translatedY = point.y - pivot.y;

        // Apply rotation in the XY-plane
        double rotatedX = translatedX * Math.Cos(angleRadians) - translatedY * Math.Sin(angleRadians);
        double rotatedY = translatedX * Math.Sin(angleRadians) + translatedY * Math.Cos(angleRadians);

        // Translate back to original position
        return new Vector3(
            (float)(rotatedX + pivot.x),
            (float)(rotatedY + pivot.y),
            point.z); // Z remains unchanged
    }

    /// <summary>
    /// Rotates a point around another point in the XZ-plane (rotation around the Y-axis).
    /// </summary>
    /// <param name="point">The point to rotate.</param>
    /// <param name="pivot">The pivot point around which to rotate.</param>
    /// <param name="angleDegrees">The angle of rotation in degrees.</param>
    /// <returns>The rotated point as a <see cref="Vector3"/>.</returns>
    private Vector3 RotateAroundXZ(Vector3 point, Vector3 pivot, double angleDegrees)
    {
        double angleRadians = angleDegrees * Math.PI / 180.0;

        // Translate to origin
        double translatedX = point.x - pivot.x;
        double translatedZ = point.z - pivot.z;

        // Apply rotation in the XZ-plane
        double rotatedX = translatedX * Math.Cos(angleRadians) - translatedZ * Math.Sin(angleRadians);
        double rotatedZ = translatedX * Math.Sin(angleRadians) + translatedZ * Math.Cos(angleRadians);

        // Translate back to original position
        return new Vector3(
            (float)(rotatedX + pivot.x),
            point.y,  // Y remains unchanged
            (float)(rotatedZ + pivot.z));
    }

    /// <summary>
    /// Calculates the angle (in degrees) from the given Z and X coordinates using the arctangent function.
    /// </summary>
    /// <param name="z">The Z-coordinate.</param>
    /// <param name="x">The X-coordinate.</param>
    /// <returns>The calculated angle in degrees.</returns>
    private float CalculateTheta(float z, float x)
    {
        return toDegrees(ATan2(z, x));
    }

    /// <summary>
    /// Returns the arctangent of the specified coordinates, considering the correct quadrant.
    /// </summary>
    /// <param name="x">The X-coordinate.</param>
    /// <param name="z">The Z-coordinate.</param>
    /// <returns>The angle in radians.</returns>
    private float ATan2(float x, float z)
    {
        return Mathf.Atan2(x, z);
    }

    /// <summary>
    /// Converts a radian value to degrees.
    /// </summary>
    /// <param name="value">The value in radians.</param>
    /// <returns>The converted value in degrees.</returns>
    private float toDegrees(float value)
    {
        return (value * 180) / Mathf.PI;
    }
}
