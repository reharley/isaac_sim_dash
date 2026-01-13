import { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

interface DifferentialDriveStatusProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

interface DriveCommand {
  linearX: number;
  linearY: number;
  linearZ: number;
  angularX: number;
  angularY: number;
  angularZ: number;
}

export function DifferentialDriveStatus({
  ros,
  connectionStatus,
}: DifferentialDriveStatusProps) {
  const [driveCommand, setDriveCommand] = useState<DriveCommand | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [messagesReceived, setMessagesReceived] = useState(0);

  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    const cmdVelTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/msg/Twist",
    });

    cmdVelTopic.subscribe((msg: any) => {
      try {
        const command: DriveCommand = {
          linearX: msg.linear.x || 0,
          linearY: msg.linear.y || 0,
          linearZ: msg.linear.z || 0,
          angularX: msg.angular.x || 0,
          angularY: msg.angular.y || 0,
          angularZ: msg.angular.z || 0,
        };
        setDriveCommand(command);
        setMessagesReceived((prev) => prev + 1);
      } catch (error) {
        console.error("[Drive] Error processing message:", error);
      }
    });

    setIsSubscribed(true);

    return () => {
      cmdVelTopic.unsubscribe();
      setIsSubscribed(false);
    };
  }, [ros, connectionStatus]);

  const getMotionIndicator = (linearX: number, angularZ: number) => {
    if (Math.abs(linearX) < 0.01 && Math.abs(angularZ) < 0.01) {
      return { status: "STOPPED", color: "#f44336", icon: "⏹️" };
    } else if (linearX > 0.01) {
      return { status: "FORWARD", color: "#4CAF50", icon: "↑" };
    } else if (linearX < -0.01) {
      return { status: "BACKWARD", color: "#4CAF50", icon: "↓" };
    } else if (angularZ > 0.01) {
      return { status: "TURNING LEFT", color: "#2196F3", icon: "↺" };
    } else if (angularZ < -0.01) {
      return { status: "TURNING RIGHT", color: "#2196F3", icon: "↻" };
    }
    return { status: "IDLE", color: "#999", icon: "●" };
  };

  const motion = driveCommand
    ? getMotionIndicator(driveCommand.linearX, driveCommand.angularZ)
    : { status: "N/A", color: "#999", icon: "?" };

  return (
    <>
      <h2>🎮 Differential Drive Status</h2>
      {connectionStatus !== "Connected" ? (
        <p style={{ color: "#666" }}>Connect to ROS to view drive status</p>
      ) : !driveCommand ? (
        <p style={{ color: "#999" }}>Waiting for drive commands...</p>
      ) : (
        <div>
          {/* Motion Status Indicator */}
          <div
            style={{
              marginBottom: "1.5rem",
              padding: "1rem",
              backgroundColor: motion.color,
              color: "white",
              borderRadius: "4px",
              textAlign: "center",
            }}
          >
            <div style={{ fontSize: "2rem", marginBottom: "0.5rem" }}>
              {motion.icon}
            </div>
            <div style={{ fontSize: "1.3rem", fontWeight: "bold" }}>
              {motion.status}
            </div>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Linear Velocity
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>X (Forward/Backward):</td>
                  <td
                    style={{
                      color: driveCommand.linearX !== 0 ? "#4CAF50" : "#999",
                    }}
                  >
                    {driveCommand.linearX.toFixed(3)} m/s
                  </td>
                </tr>
                <tr>
                  <td>Y (Strafe):</td>
                  <td
                    style={{
                      color: driveCommand.linearY !== 0 ? "#FF9800" : "#999",
                    }}
                  >
                    {driveCommand.linearY.toFixed(3)} m/s
                  </td>
                </tr>
                <tr>
                  <td>Z (Vertical):</td>
                  <td
                    style={{
                      color: driveCommand.linearZ !== 0 ? "#2196F3" : "#999",
                    }}
                  >
                    {driveCommand.linearZ.toFixed(3)} m/s
                  </td>
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
                  <td>X (Roll):</td>
                  <td
                    style={{
                      color: driveCommand.angularX !== 0 ? "#FF9800" : "#999",
                    }}
                  >
                    {driveCommand.angularX.toFixed(3)} rad/s
                  </td>
                </tr>
                <tr>
                  <td>Y (Pitch):</td>
                  <td
                    style={{
                      color: driveCommand.angularY !== 0 ? "#FF9800" : "#999",
                    }}
                  >
                    {driveCommand.angularY.toFixed(3)} rad/s
                  </td>
                </tr>
                <tr>
                  <td>Z (Yaw - Rotation):</td>
                  <td
                    style={{
                      color: driveCommand.angularZ !== 0 ? "#2196F3" : "#999",
                    }}
                  >
                    {driveCommand.angularZ.toFixed(3)} rad/s
                  </td>
                </tr>
              </tbody>
            </table>
          </div>

          {/* Combined Motion Magnitude */}
          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Motion Magnitude
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>Linear Speed:</td>
                  <td>
                    {Math.sqrt(
                      driveCommand.linearX ** 2 +
                        driveCommand.linearY ** 2 +
                        driveCommand.linearZ ** 2
                    ).toFixed(3)}{" "}
                    m/s
                  </td>
                </tr>
                <tr>
                  <td>Angular Speed:</td>
                  <td>
                    {Math.sqrt(
                      driveCommand.angularX ** 2 +
                        driveCommand.angularY ** 2 +
                        driveCommand.angularZ ** 2
                    ).toFixed(3)}{" "}
                    rad/s
                  </td>
                </tr>
              </tbody>
            </table>
          </div>

          <div
            style={{
              padding: "0.75rem",
              borderRadius: "4px",
              fontSize: "0.85rem",
              color: "#666",
            }}
          >
            <strong>Commands received:</strong> {messagesReceived}
          </div>

          {isSubscribed && (
            <div
              style={{
                marginTop: "1rem",
                padding: "0.5rem",
                borderRadius: "4px",
                fontSize: "0.85rem",
                color: "#2e7d32",
              }}
            >
              ✓ Subscribed to /cmd_vel
            </div>
          )}
        </div>
      )}
    </>
  );
}
