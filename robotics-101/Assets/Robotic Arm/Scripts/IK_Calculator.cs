using System;
using Unity.VisualScripting;
using UnityEngine;

public class IK_Calculator
{
    private float L1, L2, L3;

    public IK_Calculator(float L1, float L2, float L3)
    { 
        this.L1 = L1;
        this.L2 = L2;
        this.L3 = L3;
    }

    public Vector3 Calculate(Vector3 targetPoint3D)
    {
        if (targetPoint3D == Vector3.zero) return Vector3.zero;

        // Joint 1 Angle (Base Rotation)
        float theta = Mathf.Atan2(targetPoint3D.x, targetPoint3D.z) * Mathf.Rad2Deg;

        // Project the target point onto the arm's plane for IK calculation
        float projectedX = Mathf.Sqrt(targetPoint3D.x * targetPoint3D.x + targetPoint3D.z * targetPoint3D.z);
        float projectedY = targetPoint3D.y;

        // Calculate Euclidean distance to the target
        float targetDistance = Mathf.Sqrt(projectedX * projectedX + projectedY * projectedY);

        // Ensure the target is within reach, otherwise scale down
        if (targetDistance > L1 + L2 + L3)
        {
            float scaleFactor = (L1 + L2 + L3 - 0.001f) / targetDistance;
            projectedX *= scaleFactor;
            projectedY *= scaleFactor;
            targetDistance = L1 + L2 + L3 - 0.001f;
        }

        // Calculate the coordinates relative to Joint 2 (after taking L1 into account)
        float x = projectedX;
        float y = projectedY - L1; // Adjust for the height of J1->J2

        // Compute γ₂
        float gamma = CalculateY2(x, y);

        // **Update x and y to reflect J3's position based on γ₂**
        float updatedX = L2 * Mathf.Cos(gamma * Mathf.Deg2Rad);
        float updatedY = L2 * Mathf.Sin(gamma * Mathf.Deg2Rad);

        // Compute α (Joint 3 / Elbow angle) using the updated J3 position
        float cosAlpha = ((L2 * L2) + (L3 * L3) - (updatedX * updatedX) - (updatedY * updatedY)) / (2 * L2 * L3);
        cosAlpha = Mathf.Clamp(cosAlpha, -1f, 1f);
        float alpha = Mathf.Acos(cosAlpha) * Mathf.Rad2Deg;

        return new Vector3(theta, gamma, alpha);
    }

    public float CalculateY2(float x, float y)
    {
        // Compute γ₂₁
        float gamma21 = 90f; // Given as 90° in the diagram

        // Compute γ₂₂
        float gamma22 = Mathf.Atan2(y - L1, x) * Mathf.Rad2Deg;

        // Compute 'a' using the Euclidean distance formula
        float aSquared = x * x + (y - L1) * (y - L1);
        float a = Mathf.Sqrt(aSquared);

        // Compute γ₂₃ using the cosine rule
        float cosGamma23 = (aSquared + L2 * L2 - L3 * L3) / (2 * a * L2);
        cosGamma23 = Mathf.Clamp(cosGamma23, -1f, 1f);
        float gamma23 = Mathf.Acos(cosGamma23) * Mathf.Rad2Deg;

        // Compute γ₂
        float gamma2 = gamma21 + gamma22 + gamma23;

        return gamma2;
    }
}
