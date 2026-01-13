import { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

interface OdometryDisplayProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

interface OdometryData {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
  linearVelocity: { x: number; y: number; z: number };
  angularVelocity: { x: number; y: number; z: number };
}

export function OdometryDisplay({
  ros,
  connectionStatus,
}: OdometryDisplayProps) {
  const [odometryData, setOdometryData] = useState<OdometryData | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);

  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    const odometryTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/chassis/odom",
      messageType: "nav_msgs/msg/Odometry",
    });

    odometryTopic.subscribe((msg: any) => {
      try {
        const odomData: OdometryData = {
          position: {
            x: msg.pose.pose.position.x || 0,
            y: msg.pose.pose.position.y || 0,
            z: msg.pose.pose.position.z || 0,
          },
          orientation: {
            x: msg.pose.pose.orientation.x || 0,
            y: msg.pose.pose.orientation.y || 0,
            z: msg.pose.pose.orientation.z || 0,
            w: msg.pose.pose.orientation.w || 0,
          },
          linearVelocity: {
            x: msg.twist.twist.linear.x || 0,
            y: msg.twist.twist.linear.y || 0,
            z: msg.twist.twist.linear.z || 0,
          },
          angularVelocity: {
            x: msg.twist.twist.angular.x || 0,
            y: msg.twist.twist.angular.y || 0,
            z: msg.twist.twist.angular.z || 0,
          },
        };
        setOdometryData(odomData);
      } catch (error) {
        console.error("[Odometry] Error processing message:", error);
      }
    });

    setIsSubscribed(true);

    return () => {
      odometryTopic.unsubscribe();
      setIsSubscribed(false);
    };
  }, [ros, connectionStatus]);

  return (
    <>
      <h2>📍 Odometry</h2>
      {connectionStatus !== "Connected" ? (
        <p style={{ color: "#666" }}>Connect to ROS to view odometry data</p>
      ) : !odometryData ? (
        <p style={{ color: "#999" }}>Waiting for odometry data...</p>
      ) : (
        <div>
          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>Position</h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X:</td>
                  <td>{odometryData.position.x.toFixed(4)} m</td>
                </tr>
                <tr>
                  <td>Y:</td>
                  <td>{odometryData.position.y.toFixed(4)} m</td>
                </tr>
                <tr>
                  <td>Z:</td>
                  <td>{odometryData.position.z.toFixed(4)} m</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Orientation (Quaternion)
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X:</td>
                  <td>{odometryData.orientation.x.toFixed(4)}</td>
                </tr>
                <tr>
                  <td>Y:</td>
                  <td>{odometryData.orientation.y.toFixed(4)}</td>
                </tr>
                <tr>
                  <td>Z:</td>
                  <td>{odometryData.orientation.z.toFixed(4)}</td>
                </tr>
                <tr>
                  <td>W:</td>
                  <td>{odometryData.orientation.w.toFixed(4)}</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Linear Velocity
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X:</td>
                  <td>{odometryData.linearVelocity.x.toFixed(4)} m/s</td>
                </tr>
                <tr>
                  <td>Y:</td>
                  <td>{odometryData.linearVelocity.y.toFixed(4)} m/s</td>
                </tr>
                <tr>
                  <td>Z:</td>
                  <td>{odometryData.linearVelocity.z.toFixed(4)} m/s</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Angular Velocity
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X:</td>
                  <td>{odometryData.angularVelocity.x.toFixed(4)} rad/s</td>
                </tr>
                <tr>
                  <td>Y:</td>
                  <td>{odometryData.angularVelocity.y.toFixed(4)} rad/s</td>
                </tr>
                <tr>
                  <td>Z:</td>
                  <td>{odometryData.angularVelocity.z.toFixed(4)} rad/s</td>
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
              ✓ Subscribed to /chassis/odom
            </div>
          )}
        </div>
      )}
    </>
  );
}
