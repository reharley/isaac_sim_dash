import { useEffect, useState } from "react";
import * as ROSLIB from "roslib";
import "./App.css";
import { NovaCarterCamera } from "./components/NovaCarterCamera";
import { RobotControl } from "./components/RobotControl";
import { OdometryDisplay } from "./components/OdometryDisplay";
import { IMUDisplay } from "./components/IMUDisplay";
import { BatteryStatus } from "./components/BatteryStatus";
import { TransformTree } from "./components/TransformTree";
import { LidarDisplay } from "./components/LidarDisplay";
import { DifferentialDriveStatus } from "./components/DifferentialDriveStatus";

interface JointState {
  header: any;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

interface TopicInfo {
  name: string;
  type: string;
}

function App() {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<
    "Disconnected" | "Connecting" | "Connected" | "Error"
  >("Disconnected");
  const [jointStates, setJointStates] = useState<JointState | null>(null);
  const [error, setError] = useState<string>("");
  const [topics, setTopics] = useState<TopicInfo[]>([]);
  const [selectedTopic, setSelectedTopic] = useState<string>("/joint_states");
  const [topicMessage, setTopicMessage] = useState<any>(null);
  const [loadingTopics, setLoadingTopics] = useState(false);

  // Connect to rosbridge_suite WebSocket
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: "ws://localhost:9090", // Change to your Isaac Sim machine IP:9090 if remote
    });

    rosInstance.on("connection", () => {
      console.log("✓ Connected to rosbridge");
      setConnectionStatus("Connected");
      setError("");
      setRos(rosInstance);
    });

    rosInstance.on("error", (error: Error) => {
      console.error("✗ ROS connection error:", error);
      setConnectionStatus("Error");
      setError(error.message || "Connection error");
    });

    rosInstance.on("close", () => {
      console.log("✗ ROS connection closed");
      setConnectionStatus("Disconnected");
      setRos(null);
    });

    setConnectionStatus("Connecting");

    return () => {
      rosInstance.close();
    };
  }, []);

  // Fetch available topics when connected
  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    setLoadingTopics(true);

    const topicsClient = new ROSLIB.Service({
      ros: ros,
      name: "/rosapi/topics",
      serviceType: "rosapi/Topics",
    });

    const request = new ROSLIB.ServiceRequest({});

    topicsClient.callService(request, (result: any) => {
      const topicList: TopicInfo[] = result.topics.map(
        (topic: string, index: number) => ({
          name: topic,
          type: result.types[index] || "unknown",
        })
      );
      setTopics(topicList);
      setLoadingTopics(false);
      console.log("✓ Found topics:", topicList);
    });
  }, [ros, connectionStatus]);

  // Subscribe to selected topic
  useEffect(() => {
    if (!ros || connectionStatus !== "Connected" || !selectedTopic) return;

    const selectedTopicInfo = topics.find((t) => t.name === selectedTopic);
    if (!selectedTopicInfo) return;

    console.log(
      `Subscribing to ${selectedTopic} (type: ${selectedTopicInfo.type})`
    );

    const topicSub = new ROSLIB.Topic({
      ros: ros,
      name: selectedTopic,
      messageType: selectedTopicInfo.type,
    });

    topicSub.subscribe((message: ROSLIB.Message) => {
      setTopicMessage(message);

      // Also update jointStates if it's a joint_states topic
      if (selectedTopic === "/joint_states") {
        const jointState = message as unknown as JointState;
        setJointStates(jointState);
      }
    });

    return () => {
      topicSub.unsubscribe();
    };
  }, [ros, connectionStatus, selectedTopic, topics.length]);

  return (
    <div className="container">
      <header className="header">
        <h1>🤖 Isaac Sim ROS 2 Dashboard</h1>
        <div className={`status-badge ${connectionStatus.toLowerCase()}`}>
          <span className="status-dot"></span>
          {connectionStatus}
        </div>
      </header>

      {error && (
        <div className="error-box">
          <strong>Connection Error:</strong> {error}
          <p>Make sure rosbridge_suite is running on ws://localhost:9090</p>
        </div>
      )}

      <main className="content">
        <section className="card">
          <h2>Connection Status</h2>
          <ul className="status-list">
            <li>
              Status: <strong>{connectionStatus}</strong>
            </li>
            <li>
              WebSocket URL: <code>ws://localhost:9090</code>
            </li>
            <li>ROS Instance: {ros ? "✓ Active" : "✗ Inactive"}</li>
          </ul>
        </section>

        {connectionStatus === "Connected" && (
          <>
            <div className="card card-featured">
              <NovaCarterCamera ros={ros} connectionStatus={connectionStatus} />
            </div>

            <section className="card">
              <RobotControl ros={ros} connectionStatus={connectionStatus} />
            </section>

            <section className="card">
              <OdometryDisplay ros={ros} connectionStatus={connectionStatus} />
            </section>

            <section className="card">
              <IMUDisplay ros={ros} connectionStatus={connectionStatus} />
            </section>

            <section className="card">
              <BatteryStatus ros={ros} connectionStatus={connectionStatus} />
            </section>

            <section className="card">
              <DifferentialDriveStatus
                ros={ros}
                connectionStatus={connectionStatus}
              />
            </section>

            <section className="card">
              <LidarDisplay ros={ros} connectionStatus={connectionStatus} />
            </section>

            <section className="card card-tall">
              <TransformTree ros={ros} connectionStatus={connectionStatus} />
            </section>
          </>
        )}
      </main>
    </div>
  );
}

export default App;
