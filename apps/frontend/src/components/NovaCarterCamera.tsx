import { useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";

interface NovaCarterCameraProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

// Reusable canvas for faster rendering
let cachedCanvas: HTMLCanvasElement | null = null;

// Helper function to convert raw image data to base64 (optimized)
function rawImageToBase64(imageData: any): string {
  if (!imageData) return "";

  const { width, height, encoding, data } = imageData;

  // Reuse canvas if dimensions match, otherwise create new one
  let canvas: HTMLCanvasElement;
  if (
    cachedCanvas &&
    cachedCanvas.width === width &&
    cachedCanvas.height === height
  ) {
    canvas = cachedCanvas;
  } else {
    canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    cachedCanvas = canvas;
  }

  const ctx = canvas.getContext("2d", { willReadFrequently: true });
  if (!ctx) {
    console.error("[Camera] Failed to get canvas context");
    return "";
  }

  // Create image data
  const imgData = ctx.createImageData(width, height);
  const pixelData = imgData.data;

  // Convert data to Uint8Array
  let bytes: Uint8Array;

  if (typeof data === "string") {
    // If it's a string, it's likely already base64 encoded from rosbridge
    try {
      const binaryString = atob(data);
      bytes = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }
    } catch (e) {
      console.error("[Camera] Failed to decode base64:", e);
      return "";
    }
  } else if (data instanceof Uint8Array) {
    bytes = data;
  } else if (Array.isArray(data)) {
    bytes = new Uint8Array(data);
  } else {
    console.error("[Camera] Unknown data type:", typeof data);
    return "";
  }

  // Use TypedArray set for faster copying
  if (encoding === "rgb8") {
    // RGB to RGBA - optimized loop
    for (let i = 0, j = 0; i < bytes.length; i += 3, j += 4) {
      pixelData[j] = bytes[i];
      pixelData[j + 1] = bytes[i + 1];
      pixelData[j + 2] = bytes[i + 2];
      pixelData[j + 3] = 255;
    }
  } else if (encoding === "bgr8") {
    // BGR to RGBA - optimized loop
    for (let i = 0, j = 0; i < bytes.length; i += 3, j += 4) {
      pixelData[j] = bytes[i + 2];
      pixelData[j + 1] = bytes[i + 1];
      pixelData[j + 2] = bytes[i];
      pixelData[j + 3] = 255;
    }
  } else if (encoding === "rgba8") {
    pixelData.set(bytes);
  } else {
    console.warn("[Camera] Unsupported encoding:", encoding);
    return "";
  }

  try {
    ctx.putImageData(imgData, 0, 0);
    // Use JPEG instead of PNG for much faster encoding
    return canvas.toDataURL("image/jpeg", 0.8);
  } catch (e) {
    console.error("[Camera] Failed to convert image:", e);
    return "";
  }
}

export function NovaCarterCamera({
  ros,
  connectionStatus,
}: NovaCarterCameraProps) {
  const imgRef = useRef<HTMLImageElement>(null);
  const [cameraStatus, setCameraStatus] = useState<string>(
    "Waiting for frames..."
  );
  const [debugInfo, setDebugInfo] = useState<{
    topicSubscribed: boolean;
    messagesReceived: number;
    lastFrameTime: string;
    dataType: string;
  }>({
    topicSubscribed: false,
    messagesReceived: 0,
    lastFrameTime: "N/A",
    dataType: "N/A",
  });

  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") {
      return;
    }

    console.log(
      "[Camera] Subscribed to /front_stereo_camera/left/image_rect_color"
    );
    setDebugInfo((prev) => ({ ...prev, topicSubscribed: true }));

    // Subscribe to raw image feed
    const imageTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/front_stereo_camera/left/image_rect_color",
      messageType: "sensor_msgs/msg/Image",
    });

    imageTopic.subscribe((msg: any) => {
      try {
        if (!msg.data || !msg.width || !msg.height) {
          console.warn("[Camera] Message missing required fields");
          setCameraStatus("Error: Invalid image data");
          return;
        }

        if (!imgRef.current) {
          console.warn("[Camera] Image ref not ready");
          return;
        }

        // Convert raw image to base64
        const base64String = rawImageToBase64(msg);

        if (!base64String) {
          setCameraStatus("Error: Failed to convert image");
          return;
        }

        imgRef.current.src = base64String;

        const now = new Date().toLocaleTimeString();
        setDebugInfo((prev) => ({
          ...prev,
          messagesReceived: prev.messagesReceived + 1,
          lastFrameTime: now,
          dataType: typeof msg.data,
        }));
        setCameraStatus("✓ Streaming");
      } catch (error) {
        console.error("[Camera] Error processing frame:", error);
        setCameraStatus(
          `Error: ${error instanceof Error ? error.message : "Unknown error"}`
        );
        setDebugInfo((prev) => ({
          ...prev,
          dataType: `Error: ${
            error instanceof Error ? error.message : "Unknown"
          }`,
        }));
      }
    });

    return () => {
      console.log("[Camera] Unsubscribing");
      imageTopic.unsubscribe();
    };
  }, [ros, connectionStatus]);

  return (
    <>
      <h2>📹 Nova Carter Camera Feed</h2>
      {connectionStatus === "Connected" ? (
        <div
          style={{
            backgroundColor: "#000",
            borderRadius: "4px",
            overflow: "hidden",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            minHeight: "300px",
          }}
        >
          <img
            ref={imgRef}
            alt="Nova Carter Front Camera"
            style={{
              maxWidth: "100%",
              maxHeight: "500px",
              objectFit: "contain",
            }}
          />
        </div>
      ) : (
        <div
          style={{
            padding: "2rem",
            textAlign: "center",
            color: "#666",
            borderRadius: "4px",
          }}
        >
          <p>
            Connect to ROS to view camera feed from
            /front_stereo_camera/left/image_rect_color/compressed
          </p>
        </div>
      )}

      {/* Debug Info Panel */}
      <details
        style={{
          marginTop: "1rem",
          padding: "0.75rem",
          border: "1px solid #ddd",
          borderRadius: "4px",
          fontSize: "0.85rem",
        }}
      >
        <summary style={{ cursor: "pointer", fontWeight: "bold" }}>
          Debug Info
        </summary>
        <div style={{ marginTop: "0.75rem", fontFamily: "monospace" }}>
          <p>
            <strong>Status:</strong> {cameraStatus}
          </p>
          <p>
            <strong>Topic Subscribed:</strong>{" "}
            {debugInfo.topicSubscribed ? "✓ Yes" : "✗ No"}
          </p>
          <p>
            <strong>Messages Received:</strong> {debugInfo.messagesReceived}
          </p>
          <p>
            <strong>Last Frame:</strong> {debugInfo.lastFrameTime}
          </p>
          <p>
            <strong>Data Type:</strong> {debugInfo.dataType}
          </p>
          <p style={{ color: "#999", fontSize: "0.8rem", marginTop: "0.5rem" }}>
            Check browser console (F12) for detailed logs starting with
            "[Camera]"
          </p>
        </div>
      </details>
    </>
  );
}
