using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class CarAI : MonoBehaviour
{
    [Header("Car Wheels (Wheel Collider)")]
    public WheelCollider frontLeft;
    public WheelCollider frontRight;
    public WheelCollider backLeft;
    public WheelCollider backRight;

    [Header("Car Wheels (Transform)")]
    public Transform wheelFL;
    public Transform wheelFR;
    public Transform wheelBL;
    public Transform wheelBR;

    [Header("Car Front (Transform)")]
    public Transform carFront;

    [Header("General Parameters")]
    public List<string> NavMeshLayers;
    public int MaxSteeringAngle = 45;
    public int MaxRPM = 150;
    public float motorTorqueMultiplier = 2000f; // Increased from 400

    [Header("Waypoint System")]
    public List<Transform> waypoints = new List<Transform>();
    public bool loopMode = true;
    public float waypointReachDistance = 3f;
    
    [Header("Path Generation")]
    [Tooltip("Number of additional points to generate between each waypoint pair (higher = more dense path)")]
    public int pointsBetweenWaypoints = 5;
    [Tooltip("Smoothing iterations (higher = smoother path, 0 = no smoothing)")]
    [Range(0, 10)]
    public int pathSmoothingIterations = 3;
    [Tooltip("Smoothing strength (0 = no smoothing, 1 = maximum smoothing)")]
    [Range(0f, 1f)]
    public float pathSmoothingStrength = 0.5f;

    [Header("Debug")]
    public bool ShowGizmos;
    public bool Debugger;

    [HideInInspector] public bool move = true;
    
    /// <summary>
    /// Gets the current move state of the car
    /// </summary>
    public bool IsMoving => move;
    
    /// <summary>
    /// Sets the move state of the car
    /// </summary>
    public void SetMoveState(bool shouldMove)
    {
        move = shouldMove;
        if (Debugger)
        {
            Debug($"Move state changed to: {move}", false);
        }
    }
    
    /// <summary>
    /// Starts the car moving
    /// </summary>
    public void StartMoving()
    {
        move = true;
        if (Debugger)
        {
            Debug("Car started moving", false);
        }
    }
    
    /// <summary>
    /// Stops the car from moving
    /// </summary>
    public void StopMoving()
    {
        move = false;
        if (Debugger)
        {
            Debug("Car stopped moving", false);
        }
    }

    private List<Vector3> generatedPath = new List<Vector3>();
    private int currentPathIndex = 0;
    private Vector3 currentTarget = Vector3.zero;
    private float LocalMaxSpeed;
    private int NavMeshLayerBite;
    private Rigidbody rb;

    void Start()
    {
        // Ensure move is always true for normal operation
        move = true;
        
        rb = GetComponent<Rigidbody>();
        
        // Configure rigidbody for better car physics
        rb.mass = 1500f; // Typical car mass
        rb.drag = 0.05f; // Low air resistance
        rb.angularDrag = 0.5f; // Some rotational damping
        rb.centerOfMass = new Vector3(0, -0.5f, 0); // Lower center of mass for stability
        
        // Remove any constraints that might prevent movement
        rb.constraints = RigidbodyConstraints.None;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        
        // Configure wheel colliders
        ConfigureWheelColliders();
        
        CalculateNavMeshLayerBite();
        GenerateFullPath();
        
        if (Debugger)
        {
            Debug($"Rigidbody mass: {rb.mass}, drag: {rb.drag}, centerOfMass: {rb.centerOfMass}, constraints: {rb.constraints}", false);
            Debug($"Start: move={move}, generatedPath.Count={generatedPath.Count}", false);
        }
        
        // Ensure the car starts moving
        StartMoving();
    }
    
    private void ConfigureWheelColliders()
    {
        WheelCollider[] wheels = { frontLeft, frontRight, backLeft, backRight };
        
        foreach (WheelCollider wheel in wheels)
        {
            if (wheel != null)
            {
                // Configure wheel collider physics
                wheel.mass = 40f; // Increased from 20
                wheel.wheelDampingRate = 1f; // Increased damping
                wheel.suspensionDistance = 0.2f; // Slightly reduced
                wheel.forceAppPointDistance = 0f; // Better force application
                wheel.radius = 0.4f; // Ensure proper wheel radius
                
                // Configure suspension - stiffer for better response
                JointSpring spring = wheel.suspensionSpring;
                spring.spring = 50000f; // Increased stiffness
                spring.damper = 5000f; // Increased damping
                spring.targetPosition = 0.5f;
                wheel.suspensionSpring = spring;
                
                // Configure wheel friction - better grip
                WheelFrictionCurve forwardFriction = wheel.forwardFriction;
                forwardFriction.extremumSlip = 0.4f;
                forwardFriction.extremumValue = 1.5f; // Increased grip
                forwardFriction.asymptoteSlip = 0.8f;
                forwardFriction.asymptoteValue = 1f; // Increased
                forwardFriction.stiffness = 2f; // Increased stiffness
                wheel.forwardFriction = forwardFriction;
                
                WheelFrictionCurve sidewaysFriction = wheel.sidewaysFriction;
                sidewaysFriction.extremumSlip = 0.3f;
                sidewaysFriction.extremumValue = 1.5f; // Increased grip
                sidewaysFriction.asymptoteSlip = 0.5f;
                sidewaysFriction.asymptoteValue = 1f; // Increased
                sidewaysFriction.stiffness = 2f; // Increased stiffness
                wheel.sidewaysFriction = sidewaysFriction;
                
                if (Debugger)
                {
                    Debug($"Configured wheel: {wheel.name}", false);
                }
            }
        }
    }

    void FixedUpdate()
    {
        if (!move || generatedPath.Count == 0)
        {
            // Just update wheel visuals and stop the car
            UpdateWheels();
            StopCar();
            return;
        }

        // Update steering based on current target
        ApplySteering();
        
        // Apply movement
        Movement();
        
        // Update wheel visual positions/rotations
        UpdateWheels();
        
        // Check and update path following
        UpdatePathFollowing();
    }

    private void CalculateNavMeshLayerBite()
    {
        if (NavMeshLayers == null || NavMeshLayers.Count == 0 || NavMeshLayers[0] == "AllAreas")
            NavMeshLayerBite = NavMesh.AllAreas;
        else if (NavMeshLayers.Count == 1)
            NavMeshLayerBite = 1 << NavMesh.GetAreaFromName(NavMeshLayers[0]);
        else
        {
            foreach (string layer in NavMeshLayers)
            {
                NavMeshLayerBite += 1 << NavMesh.GetAreaFromName(layer);
            }
        }
    }

    /// <summary>
    /// Generates a full path through all waypoints using NavMesh
    /// </summary>
    public void GenerateFullPath()
    {
        if (waypoints == null || waypoints.Count == 0)
        {
            Debug("No waypoints assigned", true);
            return;
        }

        generatedPath.Clear();
        List<Vector3> rawPath = new List<Vector3>();

        // Start from car's current position
        Vector3 currentPosition = carFront.position;
        
        // Ensure starting position is on NavMesh
        if (NavMesh.SamplePosition(currentPosition, out NavMeshHit startHit, 10f, NavMeshLayerBite))
        {
            currentPosition = startHit.position;
            rawPath.Add(currentPosition);
        }

        // Generate path through all waypoints
        for (int i = 0; i < waypoints.Count; i++)
        {
            if (waypoints[i] == null) continue;

            Vector3 targetPosition = waypoints[i].position;
            
            // Ensure target waypoint is on NavMesh
            if (!NavMesh.SamplePosition(targetPosition, out NavMeshHit hit, 10f, NavMeshLayerBite))
            {
                Debug($"Waypoint {i} is not on NavMesh, skipping", false);
                continue;
            }
            
            targetPosition = hit.position;

            // Calculate NavMesh path from current position to target
            NavMeshPath path = new NavMeshPath();
            if (NavMesh.CalculatePath(currentPosition, targetPosition, NavMeshLayerBite, path))
            {
                if (path.status == NavMeshPathStatus.PathComplete)
                {
                    // Add all corners except the first (already in path)
                    for (int j = 1; j < path.corners.Length; j++)
                    {
                        rawPath.Add(path.corners[j]);
                    }
                    currentPosition = targetPosition;
                }
                else
                {
                    Debug($"Incomplete path to waypoint {i}", false);
                }
            }
            else
            {
                Debug($"Failed to calculate path to waypoint {i}", false);
            }
        }

        // If loop mode, add path back to first waypoint
        if (loopMode && waypoints.Count > 0 && waypoints[0] != null)
        {
            Vector3 firstWaypointPos = waypoints[0].position;
            if (NavMesh.SamplePosition(firstWaypointPos, out NavMeshHit hit, 10f, NavMeshLayerBite))
            {
                NavMeshPath loopPath = new NavMeshPath();
                if (NavMesh.CalculatePath(currentPosition, hit.position, NavMeshLayerBite, loopPath))
                {
                    for (int j = 1; j < loopPath.corners.Length; j++)
                    {
                        rawPath.Add(loopPath.corners[j]);
                    }
                }
            }
        }

        // Densify path (add more points between waypoints)
        generatedPath = DensifyPath(rawPath, pointsBetweenWaypoints);

        // Smooth the path
        if (pathSmoothingIterations > 0)
        {
            generatedPath = SmoothPath(generatedPath, pathSmoothingIterations, pathSmoothingStrength);
        }

        // Ensure all generated points are on NavMesh
        for (int i = 0; i < generatedPath.Count; i++)
        {
            if (NavMesh.SamplePosition(generatedPath[i], out NavMeshHit hit, 5f, NavMeshLayerBite))
            {
                generatedPath[i] = hit.position;
            }
        }

        currentPathIndex = 0;
        if (generatedPath.Count > 0)
        {
            currentTarget = generatedPath[0];
            Debug($"Generated full path with {generatedPath.Count} points", false);
        }
        else
        {
            Debug("WARNING: Generated path is empty!", true);
        }
    }

    /// <summary>
    /// Densifies path by adding intermediate points between each pair of waypoints
    /// </summary>
    private List<Vector3> DensifyPath(List<Vector3> path, int pointsPerSegment)
    {
        if (path.Count < 2 || pointsPerSegment <= 0)
            return path;

        List<Vector3> densePath = new List<Vector3>();
        
        for (int i = 0; i < path.Count - 1; i++)
        {
            densePath.Add(path[i]);
            
            Vector3 start = path[i];
            Vector3 end = path[i + 1];
            
            // Add intermediate points
            for (int j = 1; j <= pointsPerSegment; j++)
            {
                float t = j / (float)(pointsPerSegment + 1);
                Vector3 interpolatedPoint = Vector3.Lerp(start, end, t);
                densePath.Add(interpolatedPoint);
            }
        }
        
        densePath.Add(path[path.Count - 1]); // Add last point
        return densePath;
    }

    /// <summary>
    /// Smooths the path using averaging algorithm
    /// </summary>
    private List<Vector3> SmoothPath(List<Vector3> path, int iterations, float strength)
    {
        if (path.Count < 3 || iterations <= 0 || strength <= 0)
            return path;

        List<Vector3> smoothed = new List<Vector3>(path);

        for (int iter = 0; iter < iterations; iter++)
        {
            for (int i = 1; i < smoothed.Count - 1; i++)
            {
                Vector3 previous = smoothed[i - 1];
                Vector3 current = smoothed[i];
                Vector3 next = smoothed[i + 1];

                Vector3 average = (previous + current + next) / 3f;
                smoothed[i] = Vector3.Lerp(current, average, strength);
            }
        }

        return smoothed;
    }

    /// <summary>
    /// Updates path following logic
    /// </summary>
    private void UpdatePathFollowing()
    {
        if (generatedPath.Count == 0 || currentPathIndex >= generatedPath.Count)
            return;

        // Set current target from generated path
        currentTarget = generatedPath[currentPathIndex];
    
        // Check if reached current target
        float distanceToTarget = Vector3.Distance(carFront.position, currentTarget);
        
        if (Debugger)
        {
            Debug($"Distance to target: {distanceToTarget:F2}m, Index: {currentPathIndex}/{generatedPath.Count}", false);
        }
        
        if (distanceToTarget < waypointReachDistance)
        {
            currentPathIndex++;
            
            if (currentPathIndex >= generatedPath.Count)
            {
                if (loopMode)
                {
                    // Loop back to start
                    currentPathIndex = 0;
                    Debug("Looping back to start", false);
                }
                else
                {
                    // Stop at end
                    Debug("Reached end of path", false);
                    move = false;
                }
            }
            else
            {
                currentTarget = generatedPath[currentPathIndex];
            }
        }
    }

    private void UpdateWheels()
    {
        ApplyRotationAndPosition(frontLeft, wheelFL);
        ApplyRotationAndPosition(frontRight, wheelFR);
        ApplyRotationAndPosition(backLeft, wheelBL);
        ApplyRotationAndPosition(backRight, wheelBR);
    }

    private void ApplyRotationAndPosition(WheelCollider targetWheel, Transform wheel)
    {
        if (targetWheel == null || wheel == null) return;
        
        targetWheel.ConfigureVehicleSubsteps(5, 12, 15);
        targetWheel.GetWorldPose(out Vector3 pos, out Quaternion rot);
        wheel.position = pos;
        wheel.rotation = rot;
        
        // Debug wheel collider state
        if (Debugger)
        {
            Debug($"Wheel {wheel.name}: RPM={targetWheel.rpm:F1}, MotorTorque={targetWheel.motorTorque:F1}, BrakeTorque={targetWheel.brakeTorque:F1}", false);
        }
    }

    private void ApplySteering()
    {
        if (generatedPath.Count == 0 || currentPathIndex >= generatedPath.Count)
            return;
            
        // Calculate steering based on target position
        Vector3 relativeVector = transform.InverseTransformPoint(currentTarget);
        float steeringAngle = (relativeVector.x / relativeVector.magnitude) * MaxSteeringAngle;
        
        // Clamp steering angle
        steeringAngle = Mathf.Clamp(steeringAngle, -MaxSteeringAngle, MaxSteeringAngle);
        
        // Adjust max speed based on steering angle (slow down for sharp turns)
        float absSteeringAngle = Mathf.Abs(steeringAngle);
        if (absSteeringAngle > 20) 
            LocalMaxSpeed = MaxRPM * 0.5f; // 50% speed for sharp turns
        else if (absSteeringAngle > 10) 
            LocalMaxSpeed = MaxRPM * 0.7f; // 70% speed for medium turns
        else 
            LocalMaxSpeed = MaxRPM; // Full speed for straight

        frontLeft.steerAngle = steeringAngle;
        frontRight.steerAngle = steeringAngle;
        
        if (Debugger)
        {
            Debug($"Steering: {steeringAngle:F1}°, MaxSpeed: {LocalMaxSpeed:F0}, Distance: {Vector3.Distance(transform.position, currentTarget):F1}m", false);
        }
    }

    private void Movement()
    {
        if (!move || generatedPath.Count == 0)
        {
            StopCar();
            return;
        }

        // Calculate average wheel speed (RPM)
        float avgRPM = (frontLeft.rpm + frontRight.rpm + backLeft.rpm + backRight.rpm) / 4f;
        float currentSpeed = Mathf.Abs(avgRPM);

        // Release brakes first
        frontLeft.brakeTorque = 0;
        frontRight.brakeTorque = 0;
        backLeft.brakeTorque = 0;
        backRight.brakeTorque = 0;

        if (currentSpeed < LocalMaxSpeed)
        {
            // Apply motor torque - using all wheels for better traction
            float torque = motorTorqueMultiplier;
            
            // Reduce torque on front wheels for better steering
            frontLeft.motorTorque = torque * 0.7f;
            frontRight.motorTorque = torque * 0.7f;
            
            // Full torque on rear wheels (rear-wheel drive bias)
            backLeft.motorTorque = torque;
            backRight.motorTorque = torque;
            
            if (Debugger)
            {
                Debug($"Accelerating: Torque={torque:F0}, Speed={currentSpeed:F0}/{LocalMaxSpeed:F0} RPM", false);
            }
        }
        else if (currentSpeed < LocalMaxSpeed * 1.15f) // Small tolerance for coasting
        {
            // Coast - maintain current speed
            frontLeft.motorTorque = 0;
            frontRight.motorTorque = 0;
            backLeft.motorTorque = 0;
            backRight.motorTorque = 0;
            
            if (Debugger)
            {
                Debug($"Coasting: Speed={currentSpeed:F0} RPM", false);
            }
        }
        else
        {
            // Going too fast - apply light braking
            float brakeTorque = 2000f;
            frontLeft.brakeTorque = brakeTorque;
            frontRight.brakeTorque = brakeTorque;
            backLeft.brakeTorque = brakeTorque;
            backRight.brakeTorque = brakeTorque;
            
            // Stop motor
            frontLeft.motorTorque = 0;
            frontRight.motorTorque = 0;
            backLeft.motorTorque = 0;
            backRight.motorTorque = 0;
            
            if (Debugger)
            {
                Debug($"Braking: Speed={currentSpeed:F0} (over {LocalMaxSpeed:F0}) RPM", false);
            }
        }
    }

    private void StopCar()
    {
        // Apply moderate braking
        float brakeTorque = 3000f;
        frontLeft.brakeTorque = brakeTorque;
        frontRight.brakeTorque = brakeTorque;
        backLeft.brakeTorque = brakeTorque;
        backRight.brakeTorque = brakeTorque;
        
        // Stop all motor torque
        frontLeft.motorTorque = 0;
        frontRight.motorTorque = 0;
        backLeft.motorTorque = 0;
        backRight.motorTorque = 0;
    }

    private void Debug(string text, bool isCritical)
    {
        if (Debugger)
        {
            if (isCritical)
                UnityEngine.Debug.LogError(text);
            else
                UnityEngine.Debug.Log(text);
        }
    }

    private void OnDrawGizmos()
    {
        if (!ShowGizmos) return;

        // Draw original waypoints (Transform targets)
        if (waypoints != null)
        {
            for (int i = 0; i < waypoints.Count; i++)
            {
                if (waypoints[i] != null)
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawWireSphere(waypoints[i].position, 2f);
                    
                    #if UNITY_EDITOR
                    UnityEditor.Handles.Label(waypoints[i].position + Vector3.up * 3, $"WP {i}");
                    #endif
                    
                    // Draw line to next waypoint
                    if (i < waypoints.Count - 1 && waypoints[i + 1] != null)
                    {
                        Gizmos.color = new Color(0, 1, 1, 0.3f);
                        Gizmos.DrawLine(waypoints[i].position, waypoints[i + 1].position);
                    }
                    
                    // Draw line back to first if loop mode
                    if (loopMode && i == waypoints.Count - 1 && waypoints[0] != null)
                    {
                        Gizmos.color = new Color(1, 1, 0, 0.3f);
                        Gizmos.DrawLine(waypoints[i].position, waypoints[0].position);
                    }
                }
            }
        }

        // Draw generated path
        if (generatedPath != null && generatedPath.Count > 0)
        {
            for (int i = 0; i < generatedPath.Count; i++)
            {
                // Color based on position in path
                if (i < currentPathIndex)
                    Gizmos.color = Color.green; // Passed points
                else if (i == currentPathIndex)
                    Gizmos.color = Color.yellow; // Current target
                else
                    Gizmos.color = Color.red; // Upcoming points

                Gizmos.DrawWireSphere(generatedPath[i], 0.5f);

                // Draw path lines
                if (i < generatedPath.Count - 1)
                {
                    Gizmos.color = (i < currentPathIndex) ? Color.green : Color.red;
                    Gizmos.DrawLine(generatedPath[i], generatedPath[i + 1]);
                }
            }

            // Draw current target marker
            if (currentPathIndex < generatedPath.Count)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(currentTarget, 1.5f);
                
                // Draw line from car to current target
                if (carFront != null)
                {
                    Gizmos.color = Color.magenta;
                    Gizmos.DrawLine(carFront.position, currentTarget);
                }
            }
        }
        
        // Draw car's forward direction
        if (carFront != null)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(carFront.position, carFront.forward * 5f);
        }
    }
}
