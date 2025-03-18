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
        //L1 is 25.2
        //L2 is 23.7
        //L3 is 29.7

        //y comes in as 27.7
        
        //H is shown as 46.06789 when blue dot is at X-20 and Z41.5
        //alpha is shown as 166.9587 in this case
        //gamma is shown as 125.2029 in this case

        //H is shown as 46.88816 when X is 17.5 and Z is 43.5
        //alpha is shown as 122.7208 in this case
        //gamma is shown as 99.9571 in this case

        //H is shown as 53 when X is 0 and Z is 53
        //alpha is shown as 119.1031 in this case
        //gamma is shown as 127.3335 in this case

        if (targetPoint3D == Vector3.zero) return Vector3.zero;

        float H = Mathf.Sqrt(Mathf.Pow(targetPoint3D.x, 2) + Mathf.Pow(targetPoint3D.z, 2));

        // Joint 1 Angle (Base Rotation)
        float theta = Mathf.Atan2(targetPoint3D.x, targetPoint3D.z) * Mathf.Rad2Deg;
        float y = targetPoint3D.y;
        
        // Joint 2 Angle
        float gamma = CalculateY2(H, y);
        
        //Joint 3 angle
        float a2 = Mathf.Pow(H,2) + Mathf.Pow(y - L1,2);
        float cosAlpha = (Mathf.Pow(L2, 2) + Mathf.Pow(L3, 2)- a2) / (2 * L2 * L3);
        cosAlpha = Mathf.Clamp(cosAlpha, -1f, 1f);
        float alpha = Mathf.Acos(cosAlpha) * Mathf.Rad2Deg;

        return new Vector3(theta, 180 - gamma ,180- alpha);
    }

    //checked and fully matches slides
    public float CalculateY2(float x, float y)
    {
        float gamma21 = 90f;

        float gamma22 = Mathf.Atan2(y - L1, x) * Mathf.Rad2Deg;

        float a = Mathf.Sqrt(Mathf.Pow(x, 2) + Mathf.Pow(y - L1, 2));

        float cosGamma23 = (Mathf.Pow(a,2) + Mathf.Pow(L2, 2)  - Mathf.Pow(L3, 2)) / (2 * a * L2);
        cosGamma23 = Mathf.Clamp(cosGamma23, -1f, 1f);
        float gamma23 = Mathf.Acos(cosGamma23) * Mathf.Rad2Deg;

        float gamma2 = gamma21 + gamma22 + gamma23;

        return gamma2;
    }
}