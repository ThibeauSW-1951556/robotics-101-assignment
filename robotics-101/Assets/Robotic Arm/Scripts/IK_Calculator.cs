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

        //float theta = 0; // Joint 1 Angle (Base Rotation)
        float theta = Mathf.Atan2(targetPoint3D.x, targetPoint3D.z) * Mathf.Rad2Deg;

         // Project the target point onto the arm's plane for IK calculation
        float projectedX = Mathf.Sqrt(targetPoint3D.x * targetPoint3D.x + targetPoint3D.z * targetPoint3D.z);
        float projectedY = targetPoint3D.y;
        
        
        // Calculate Euclidean distance to the target
        float targetDistance = Mathf.Sqrt(projectedX * projectedX + projectedY * projectedY);
        
        // Ensure the target is within reach, otherwise scale down
        if (targetDistance > L1 + L2 + L3) {
            float scaleFactor = (L1 + L2 + L3 - 0.001f) / targetDistance;
            projectedX *= scaleFactor;
            projectedY *= scaleFactor;
            targetDistance = L1 + L2 + L3 - 0.001f;
        }
        
        // Calculate the coordinates relative to Joint 2 (after taking L1 into account)
        float x = projectedX;
        float y = projectedY - L1; // Adjust for the height of J1->J2
        
        // Calculate distance from J2 to target
        float distanceJ2ToTarget = Mathf.Sqrt(x*x + y*y);
        
        // Ensure this distance is within reach of L2+L3 otherwise scale down
        if (distanceJ2ToTarget > L2 + L3) {
            float scaleFactor = (L2 + L3 - 0.001f) / distanceJ2ToTarget;
            x *= scaleFactor;
            y *= scaleFactor;
            distanceJ2ToTarget = L2 + L3 - 0.001f;
        }

        float gamma = 0;// Joint 2 Angle
        float alpha = 0;// Joint 3 Angle

        var targetAngles = new Vector3(theta, gamma, alpha);

        return targetAngles;
    }
}
