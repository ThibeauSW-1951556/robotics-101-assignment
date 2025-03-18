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
        float projectedX = Mathf.Sqrt(Mathf.Pow(targetPoint3D.x, 2) + Mathf.Pow(targetPoint3D.z, 2));
        float projectedY = targetPoint3D.y - 3.5f; // to not clip (look into this later cause left doesn't seem to clip at the same time as right)

        // Calculate the coordinates relative to Joint 2 (after taking L1 into account)
        float x = projectedX;
        float y = projectedY - L1; // Adjust for the height of J1->J2

        // Compute γ₂
        float gamma = CalculateY2(x, y);

        // **Update x and y to reflect J3's position based on γ₂**
        float updatedX = L2 * Mathf.Cos(gamma * Mathf.Deg2Rad);
        float updatedY = L2 * Mathf.Sin(gamma * Mathf.Deg2Rad);

        // Compute alpha using the updated position
        float cosAlpha = (Mathf.Pow(L2, 2) + Mathf.Pow(L3, 2)- Mathf.Pow(updatedX, 2)- Mathf.Pow(updatedY, 2)) / (2 * L2 * L3);
        cosAlpha = Mathf.Clamp(cosAlpha, -1f, 1f);
        float alpha = Mathf.Acos(cosAlpha) * Mathf.Rad2Deg;

        return new Vector3(theta, gamma, alpha);
    }

    public float CalculateY2(float x, float y)
    {
        float gamma21 = 90f;

        float gamma22 = Mathf.Atan2(y - L1, x) * Mathf.Rad2Deg;

        // Compute 'a' using the Euclidean distance formula
        float aSquared = x * x + (y - L1) * (y - L1);
        float a = Mathf.Sqrt(aSquared);

        float cosGamma23 = (aSquared + Mathf.Pow(L2, 2)  - Mathf.Pow(L3, 2)) / (2 * a * L2);
        cosGamma23 = Mathf.Clamp(cosGamma23, -1f, 1f);
        float gamma23 = Mathf.Acos(cosGamma23) * Mathf.Rad2Deg;

        float gamma2 = gamma21 + gamma22 + gamma23;

        return gamma2;
    }
}