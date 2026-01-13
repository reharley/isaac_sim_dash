import { useEffect, useRef } from "react";
import * as ROSLIB from "roslib";

interface RobotControlProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

export function RobotControl({ ros, connectionStatus }: RobotControlProps) {
  const linearRef = useRef<number>(0);
  const angularRef = useRef<number>(0);
  const topicRef = useRef<ROSLIB.Topic | null>(null);

  // Initialize topic once ROS is connected
  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    topicRef.current = new ROSLIB.Topic({
      ros: ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/msg/Twist",
    });

    return () => {
      // Publish stop command on unmount
      if (topicRef.current) {
        const stopMsg = new ROSLIB.Message({
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
        topicRef.current.publish(stopMsg);
      }
    };
  }, [ros, connectionStatus]);

  const publishCmdVel = (linear: number, angular: number) => {
    if (!topicRef.current) return;

    const twistMsg = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    });

    topicRef.current.publish(twistMsg);
    linearRef.current = linear;
    angularRef.current = angular;
  };

  const handleButtonClick = (action: string) => {
    switch (action) {
      case "forward":
        publishCmdVel(0.5, 0);
        break;
      case "backward":
        publishCmdVel(-0.5, 0);
        break;
      case "left":
        publishCmdVel(0, 0.5);
        break;
      case "right":
        publishCmdVel(0, -0.5);
        break;
      case "stop":
        publishCmdVel(0, 0);
        break;
    }
  };

  const handleLinearSliderChange = (value: number) => {
    publishCmdVel(value, angularRef.current);
  };

  const handleAngularSliderChange = (value: number) => {
    publishCmdVel(linearRef.current, value);
  };

  const isConnected = connectionStatus === "Connected";

  return (
    <>
      <h2>🎮 Nova Carter Control</h2>
      {!isConnected && (
        <div
          style={{
            padding: "1rem",
            borderRadius: "4px",
            marginBottom: "1rem",
            color: "#856404",
          }}
        >
          Connect to ROS to control the robot
        </div>
      )}

      <div
        style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "1rem" }}
      >
        {/* Direction Buttons */}
        <div>
          <h3 style={{ marginTop: 0 }}>Direction Control</h3>
          <div
            style={{
              display: "grid",
              gridTemplateColumns: "repeat(3, 1fr)",
              gap: "0.5rem",
              marginBottom: "1rem",
            }}
          >
            <div></div>
            <button
              onClick={() => handleButtonClick("forward")}
              disabled={!isConnected}
              style={{
                padding: "0.75rem",
                backgroundColor: isConnected ? "#4CAF50" : "#ccc",
                color: "white",
                border: "none",
                borderRadius: "4px",
                cursor: isConnected ? "pointer" : "not-allowed",
                fontSize: "1.2rem",
              }}
            >
              ↑
            </button>
            <div></div>

            <button
              onClick={() => handleButtonClick("left")}
              disabled={!isConnected}
              style={{
                padding: "0.75rem",
                backgroundColor: isConnected ? "#2196F3" : "#ccc",
                color: "white",
                border: "none",
                borderRadius: "4px",
                cursor: isConnected ? "pointer" : "not-allowed",
                fontSize: "1.2rem",
              }}
            >
              ←
            </button>
            <button
              onClick={() => handleButtonClick("stop")}
              disabled={!isConnected}
              style={{
                padding: "0.75rem",
                backgroundColor: isConnected ? "#f44336" : "#ccc",
                color: "white",
                border: "none",
                borderRadius: "4px",
                cursor: isConnected ? "pointer" : "not-allowed",
                fontSize: "1rem",
                fontWeight: "bold",
              }}
            >
              STOP
            </button>
            <button
              onClick={() => handleButtonClick("right")}
              disabled={!isConnected}
              style={{
                padding: "0.75rem",
                backgroundColor: isConnected ? "#2196F3" : "#ccc",
                color: "white",
                border: "none",
                borderRadius: "4px",
                cursor: isConnected ? "pointer" : "not-allowed",
                fontSize: "1.2rem",
              }}
            >
              →
            </button>

            <div></div>
            <button
              onClick={() => handleButtonClick("backward")}
              disabled={!isConnected}
              style={{
                padding: "0.75rem",
                backgroundColor: isConnected ? "#4CAF50" : "#ccc",
                color: "white",
                border: "none",
                borderRadius: "4px",
                cursor: isConnected ? "pointer" : "not-allowed",
                fontSize: "1.2rem",
              }}
            >
              ↓
            </button>
            <div></div>
          </div>
        </div>

        {/* Sliders */}
        <div>
          <h3 style={{ marginTop: 0 }}>Fine Control</h3>
          <div style={{ marginBottom: "1rem" }}>
            <label style={{ display: "block", marginBottom: "0.5rem" }}>
              <strong>Linear Velocity:</strong> {linearRef.current.toFixed(2)}{" "}
              m/s
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              defaultValue="0"
              onChange={(e) =>
                handleLinearSliderChange(parseFloat(e.target.value))
              }
              disabled={!isConnected}
              style={{
                width: "100%",
                cursor: isConnected ? "pointer" : "not-allowed",
              }}
            />
          </div>
          <div>
            <label style={{ display: "block", marginBottom: "0.5rem" }}>
              <strong>Angular Velocity:</strong> {angularRef.current.toFixed(2)}{" "}
              rad/s
            </label>
            <input
              type="range"
              min="-1"
              max="1"
              step="0.1"
              defaultValue="0"
              onChange={(e) =>
                handleAngularSliderChange(parseFloat(e.target.value))
              }
              disabled={!isConnected}
              style={{
                width: "100%",
                cursor: isConnected ? "pointer" : "not-allowed",
              }}
            />
          </div>
        </div>
      </div>

      <div
        style={{
          marginTop: "1rem",
          padding: "0.75rem",
          borderRadius: "4px",
          fontSize: "0.9rem",
        }}
      >
        <strong>Publishing to:</strong> /cmd_vel (geometry_msgs/msg/Twist)
      </div>
    </>
  );
}
