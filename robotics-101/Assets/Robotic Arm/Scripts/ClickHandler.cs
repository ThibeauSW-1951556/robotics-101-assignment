using UnityEngine;

public class CubeClickHandler : MonoBehaviour
{
    public GameObject targetSphere; // Assign in the Inspector
    public float heightOffset = 9f;

    private void OnMouseDown()
    {
        if (targetSphere != null)
        {
            // Calculate the object's height using its collider
            float objectHeight = GetComponent<Collider>().bounds.size.y;

            // Position the sphere 9 units above the top of the cube
            Vector3 cubeTopPosition = transform.position + Vector3.up * (objectHeight / 2);

            Vector3 targetPosition = cubeTopPosition + Vector3.up * heightOffset;

            targetSphere.transform.position = targetPosition;
        }
    }
}
