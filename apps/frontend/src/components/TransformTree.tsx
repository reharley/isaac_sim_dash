import { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

interface TransformTreeProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

interface TransformInfo {
  child_frame_id: string;
  parent_frame_id: string;
  translation: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number; w: number };
}

export function TransformTree({ ros, connectionStatus }: TransformTreeProps) {
  const [transforms, setTransforms] = useState<TransformInfo[]>([]);
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [expandedTransforms, setExpandedTransforms] = useState<Set<number>>(
    new Set()
  );

  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    const tfTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/tf",
      messageType: "tf2_msgs/msg/TFMessage",
    });

    tfTopic.subscribe((msg: any) => {
      try {
        if (msg.transforms && Array.isArray(msg.transforms)) {
          const tfList: TransformInfo[] = msg.transforms.map(
            (transform: any) => ({
              child_frame_id: transform.child_frame_id || "",
              parent_frame_id: transform.header.frame_id || "",
              translation: {
                x: transform.transform.translation.x || 0,
                y: transform.transform.translation.y || 0,
                z: transform.transform.translation.z || 0,
              },
              rotation: {
                x: transform.transform.rotation.x || 0,
                y: transform.transform.rotation.y || 0,
                z: transform.transform.rotation.z || 0,
                w: transform.transform.rotation.w || 0,
              },
            })
          );
          setTransforms(tfList);
        }
      } catch (error) {
        console.error("[TF] Error processing message:", error);
      }
    });

    setIsSubscribed(true);

    return () => {
      tfTopic.unsubscribe();
      setIsSubscribed(false);
    };
  }, [ros, connectionStatus]);

  const toggleExpanded = (index: number) => {
    const newSet = new Set(expandedTransforms);
    if (newSet.has(index)) {
      newSet.delete(index);
    } else {
      newSet.add(index);
    }
    setExpandedTransforms(newSet);
  };

  return (
    <>
      <h2>🌳 Transform Tree (TF)</h2>
      {connectionStatus !== "Connected" ? (
        <p style={{ color: "#666" }}>Connect to ROS to view transform data</p>
      ) : transforms.length === 0 ? (
        <p style={{ color: "#999" }}>Waiting for transform data...</p>
      ) : (
        <div>
          <p style={{ marginTop: 0, color: "#666" }}>
            Found {transforms.length} active transform
            {transforms.length !== 1 ? "s" : ""}
          </p>

          <div
            style={{
              maxHeight: "400px",
              overflowY: "auto",
              backgroundColor: "#f5f5f5",
              borderRadius: "4px",
              padding: "0.5rem 0",
            }}
          >
            {transforms.map((tf, index) => (
              <div
                key={index}
                style={{
                  borderBottom: "1px solid #ddd",
                  padding: "0.75rem",
                  backgroundColor: expandedTransforms.has(index)
                    ? "#fff"
                    : "transparent",
                }}
              >
                <div
                  style={{
                    display: "flex",
                    justifyContent: "space-between",
                    alignItems: "center",
                    cursor: "pointer",
                    userSelect: "none",
                  }}
                  onClick={() => toggleExpanded(index)}
                >
                  <div>
                    <strong>{tf.child_frame_id}</strong>
                    <span style={{ color: "#999", fontSize: "0.9rem" }}>
                      {" "}
                      ← {tf.parent_frame_id}
                    </span>
                  </div>
                  <span style={{ color: "#666" }}>
                    {expandedTransforms.has(index) ? "▼" : "▶"}
                  </span>
                </div>

                {expandedTransforms.has(index) && (
                  <div
                    style={{
                      marginTop: "0.75rem",
                      paddingLeft: "1rem",
                      borderLeft: "2px solid #2196F3",
                      fontSize: "0.85rem",
                    }}
                  >
                    <div style={{ marginBottom: "0.5rem" }}>
                      <strong>Translation:</strong>
                      <div style={{ color: "#666", marginLeft: "0.5rem" }}>
                        X: {tf.translation.x.toFixed(4)} m<br />
                        Y: {tf.translation.y.toFixed(4)} m<br />
                        Z: {tf.translation.z.toFixed(4)} m
                      </div>
                    </div>
                    <div>
                      <strong>Rotation (Quaternion):</strong>
                      <div style={{ color: "#666", marginLeft: "0.5rem" }}>
                        X: {tf.rotation.x.toFixed(4)}
                        <br />
                        Y: {tf.rotation.y.toFixed(4)}
                        <br />
                        Z: {tf.rotation.z.toFixed(4)}
                        <br />
                        W: {tf.rotation.w.toFixed(4)}
                      </div>
                    </div>
                  </div>
                )}
              </div>
            ))}
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
              ✓ Subscribed to /tf
            </div>
          )}
        </div>
      )}
    </>
  );
}
