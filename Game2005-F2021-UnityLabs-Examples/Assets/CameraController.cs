using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [Header("Controls")]
    public float sensitivity = 10.0f;

    [Header("Movement")]
    public float maxSpeed = 10.0f;

    public Vector3 velocity;

    private float XAxisRotation = 0.0f;
    private float YAxisRotation = 0.0f;  

    private Vector2 mouse;

    // Start is called before the first frame update
    void Start()
    {
        Cursor.lockState = CursorLockMode.Locked;
    }

    // Update is called once per frame
    void Update()
    {
        MouseLook();
        Move();
    }

    private void MouseLook()
    {
        // get input from mouse
        mouse.x = Input.GetAxis("Mouse X") * sensitivity;
        mouse.y = Input.GetAxis("Mouse Y") * sensitivity;

        // Look up and down 
        XAxisRotation -= mouse.y;
        XAxisRotation = Mathf.Clamp(XAxisRotation, -90.0f, 90.0f);

        // Look left and right rotation
        YAxisRotation += mouse.x;
        //YAxisRotation = Mathf.Clamp(YAxisRotation, -90.0f, 90.0f);

        // rotate
        transform.localRotation = Quaternion.Euler(XAxisRotation, YAxisRotation, 0.0f);
    }

    private void Move()
    {
        float x = Input.GetAxis("Horizontal");
        float y = Input.GetAxis("Vertical");

        Vector3 moveForward = Vector3.MoveTowards(Vector3.zero, transform.forward  * maxSpeed, y * maxSpeed * Time.deltaTime);
        Vector3 moveSideways = Vector3.MoveTowards(Vector3.zero, transform.right * maxSpeed, x * maxSpeed * Time.deltaTime);
        transform.position += moveForward + moveSideways;
    }
}
