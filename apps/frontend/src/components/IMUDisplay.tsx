import { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

interface IMUDisplayProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

interface IMUData {
  orientation: { x: number; y: number; z: number; w: number };
  angularVelocity: { x: number; y: number; z: number };
  linearAcceleration: { x: number; y: number; z: number };
}

export function IMUDisplay({ ros, connectionStatus }: IMUDisplayProps) {
  const [imuData, setIMUData] = useState<IMUData | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);

  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    const imuTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/chassis/imu",
      messageType: "sensor_msgs/msg/Imu",
    });

    imuTopic.subscribe((msg: any) => {
      try {
        const imu: IMUData = {
          orientation: {
            x: msg.orientation.x || 0,
            y: msg.orientation.y || 0,
            z: msg.orientation.z || 0,
            w: msg.orientation.w || 0,
          },
          angularVelocity: {
            x: msg.angular_velocity.x || 0,
            y: msg.angular_velocity.y || 0,
            z: msg.angular_velocity.z || 0,
          },
          linearAcceleration: {
            x: msg.linear_acceleration.x || 0,
            y: msg.linear_acceleration.y || 0,
            z: msg.linear_acceleration.z || 0,
          },
        };
        setIMUData(imu);
      } catch (error) {
        console.error("[IMU] Error processing message:", error);
      }
    });

    setIsSubscribed(true);

    return () => {
      imuTopic.unsubscribe();
      setIsSubscribed(false);
    };
  }, [ros, connectionStatus]);

  return (
    <>
      <h2>🧭 IMU Data</h2>
      {connectionStatus !== "Connected" ? (
        <p style={{ color: "#666" }}>Connect to ROS to view IMU data</p>
      ) : !imuData ? (
        <p style={{ color: "#999" }}>Waiting for IMU data...</p>
      ) : (
        <div>
          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Orientation (Quaternion)
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X:</td>
                  <td>{imuData.orientation.x.toFixed(4)}</td>
                </tr>
                <tr>
                  <td>Y:</td>
                  <td>{imuData.orientation.y.toFixed(4)}</td>
                </tr>
                <tr>
                  <td>Z:</td>
                  <td>{imuData.orientation.z.toFixed(4)}</td>
                </tr>
                <tr>
                  <td>W:</td>
                  <td>{imuData.orientation.w.toFixed(4)}</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Angular Velocity
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X:</td>
                  <td>{imuData.angularVelocity.x.toFixed(4)} rad/s</td>
                </tr>
                <tr>
                  <td>Y:</td>
                  <td>{imuData.angularVelocity.y.toFixed(4)} rad/s</td>
                </tr>
                <tr>
                  <td>Z:</td>
                  <td>{imuData.angularVelocity.z.toFixed(4)} rad/s</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Linear Acceleration
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X:</td>
                  <td>{imuData.linearAcceleration.x.toFixed(4)} m/s²</td>
                </tr>
                <tr>
                  <td>Y:</td>
                  <td>{imuData.linearAcceleration.y.toFixed(4)} m/s²</td>
                </tr>
                <tr>
                  <td>Z:</td>
                  <td>{imuData.linearAcceleration.z.toFixed(4)} m/s²</td>
                </tr>
              </tbody>
            </table>
          </div>

          {isSubscribed && (
            <div
              style={{
                marginTop: "1rem",
                padding: "0.5rem",
                backgroundColor: "#e8f5e9",
                borderRadius: "4px",
                fontSize: "0.85rem",
                color: "#2e7d32",
              }}
            >
              ✓ Subscribed to /chassis/imu
            </div>
          )}
        </div>
      )}
    </>
  );
}
