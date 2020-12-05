using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionSensor : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    public float SumImpulseMagnitude = 0;
    public float SumImpulseMagnitudeForAction = 0;

    void OnCollisionStay(Collision collision)
    {
        SumImpulseMagnitude += collision.impulse.magnitude;
        SumImpulseMagnitudeForAction += collision.impulse.magnitude;
    }
    void OnCollisionEnter(Collision collision)
    {
        OnCollisionStay(collision);
    }
}
