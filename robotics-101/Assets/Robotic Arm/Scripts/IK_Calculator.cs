using System;
using Unity.VisualScripting;
using UnityEngine;

public class IK_Calculator
{
    private float L1, L2, L3;

    public IK_Calculator(float L1, float L2, float L3) { 
        this.L1 = L1;
        this.L2 = L2;
        this.L3 = L3;
    }

    public Vector3 Calculate(Vector3 targetPoint3D)
    {
        if (targetPoint3D == Vector3.zero) return Vector3.zero;

        float theta = 0; // Joint 1 Angle (Base Rotation)
        float gamma = 0;// Joint 2 Angle
        float alpha = 0;// Joint 3 Angle

        var targetAngles = new Vector3(theta, gamma, alpha);

        return targetAngles;
    }
}
