import { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

interface LidarDisplayProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

interface LidarStats {
  pointCount: number;
  fields: string[];
  height: number;
  width: number;
  pointStep: number;
  rowStep: number;
  isBigEndian: boolean;
  isDense: boolean;
}

export function LidarDisplay({ ros, connectionStatus }: LidarDisplayProps) {
  const [lidarStats, setLidarStats] = useState<LidarStats | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [messagesReceived, setMessagesReceived] = useState(0);

  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    const lidarTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/front_3d_lidar/lidar_points",
      messageType: "sensor_msgs/msg/PointCloud2",
    });

    lidarTopic.subscribe((msg: any) => {
      try {
        const stats: LidarStats = {
          pointCount: msg.width * msg.height || 0,
          fields: msg.fields?.map((f: any) => f.name) || [],
          height: msg.height || 0,
          width: msg.width || 0,
          pointStep: msg.point_step || 0,
          rowStep: msg.row_step || 0,
          isBigEndian: msg.is_bigendian || false,
          isDense: msg.is_dense || false,
        };
        setLidarStats(stats);
        setMessagesReceived((prev) => prev + 1);
      } catch (error) {
        console.error("[LIDAR] Error processing message:", error);
      }
    });

    setIsSubscribed(true);

    return () => {
      lidarTopic.unsubscribe();
      setIsSubscribed(false);
    };
  }, [ros, connectionStatus]);

  return (
    <>
      <h2>📡 3D LIDAR</h2>
      {connectionStatus !== "Connected" ? (
        <p style={{ color: "#666" }}>Connect to ROS to view LIDAR data</p>
      ) : !lidarStats ? (
        <p style={{ color: "#999" }}>Waiting for LIDAR data...</p>
      ) : (
        <div>
          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Point Cloud Statistics
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>Total Points:</td>
                  <td>{lidarStats.pointCount.toLocaleString()}</td>
                </tr>
                <tr>
                  <td>Width:</td>
                  <td>{lidarStats.width}</td>
                </tr>
                <tr>
                  <td>Height:</td>
                  <td>{lidarStats.height}</td>
                </tr>
                <tr>
                  <td>Point Step:</td>
                  <td>{lidarStats.pointStep} bytes</td>
                </tr>
                <tr>
                  <td>Row Step:</td>
                  <td>{lidarStats.rowStep} bytes</td>
                </tr>
                <tr>
                  <td>Data Size:</td>
                  <td>
                    {(
                      (lidarStats.rowStep * lidarStats.height) /
                      (1024 * 1024)
                    ).toFixed(2)}{" "}
                    MB
                  </td>
                </tr>
              </tbody>
            </table>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Point Fields
            </h3>
            <div
              style={{
                backgroundColor: "#f5f5f5",
                padding: "0.75rem",
                borderRadius: "4px",
                fontSize: "0.9rem",
              }}
            >
              {lidarStats.fields.length > 0 ? (
                <div
                  style={{ display: "flex", flexWrap: "wrap", gap: "0.5rem" }}
                >
                  {lidarStats.fields.map((field, idx) => (
                    <span
                      key={idx}
                      style={{
                        backgroundColor: "#e3f2fd",
                        padding: "0.25rem 0.5rem",
                        borderRadius: "3px",
                        color: "#1976d2",
                      }}
                    >
                      {field}
                    </span>
                  ))}
                </div>
              ) : (
                <span style={{ color: "#999" }}>No fields available</span>
              )}
            </div>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>Format</h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>Byte Order:</td>
                  <td>
                    {lidarStats.isBigEndian ? "Big Endian" : "Little Endian"}
                  </td>
                </tr>
                <tr>
                  <td>Dense:</td>
                  <td>
                    {lidarStats.isDense
                      ? "Yes (no NaN points)"
                      : "No (may contain NaN)"}
                  </td>
                </tr>
              </tbody>
            </table>
          </div>

          <div
            style={{
              padding: "0.75rem",
              backgroundColor: "#f0f0f0",
              borderRadius: "4px",
              fontSize: "0.85rem",
              color: "#666",
            }}
          >
            <strong>Messages received:</strong> {messagesReceived}
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
              ✓ Subscribed to /front_3d_lidar/lidar_points
            </div>
          )}
        </div>
      )}
    </>
  );
}
