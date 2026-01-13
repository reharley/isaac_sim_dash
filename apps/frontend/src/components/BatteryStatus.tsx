import { useEffect, useState } from "react";
import * as ROSLIB from "roslib";

interface BatteryStatusProps {
  ros: ROSLIB.Ros | null;
  connectionStatus: "Disconnected" | "Connecting" | "Connected" | "Error";
}

interface BatteryData {
  voltage: number;
  current: number;
  charge: number;
  capacity: number;
  design_capacity: number;
  percentage: number;
  power_supply_status: number;
  power_supply_health: number;
}

const POWER_SUPPLY_STATUS: { [key: number]: string } = {
  0: "Unknown",
  1: "Charging",
  2: "Discharging",
  3: "Not charging",
  4: "Full",
};

const POWER_SUPPLY_HEALTH: { [key: number]: string } = {
  0: "Unknown",
  1: "Good",
  2: "Overheat",
  3: "Dead",
  4: "Over voltage",
  5: "Unspecified failure",
  6: "Cold",
  7: "Watchdog timer expire",
  8: "Safety timer expire",
};

export function BatteryStatus({ ros, connectionStatus }: BatteryStatusProps) {
  const [batteryData, setBatteryData] = useState<BatteryData | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);

  useEffect(() => {
    if (!ros || connectionStatus !== "Connected") return;

    const batteryTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/battery_state",
      messageType: "sensor_msgs/msg/BatteryState",
    });

    batteryTopic.subscribe((msg: any) => {
      try {
        const battery: BatteryData = {
          voltage: msg.voltage || 0,
          current: msg.current || 0,
          charge: msg.charge || 0,
          capacity: msg.capacity || 0,
          design_capacity: msg.design_capacity || 0,
          percentage: msg.percentage || 0,
          power_supply_status: msg.power_supply_status || 0,
          power_supply_health: msg.power_supply_health || 0,
        };
        setBatteryData(battery);
      } catch (error) {
        console.error("[Battery] Error processing message:", error);
      }
    });

    setIsSubscribed(true);

    return () => {
      batteryTopic.unsubscribe();
      setIsSubscribed(false);
    };
  }, [ros, connectionStatus]);

  const getStatusColor = (percentage: number) => {
    if (percentage > 75) return "#4CAF50"; // Green
    if (percentage > 50) return "#FFC107"; // Amber
    if (percentage > 25) return "#FF9800"; // Orange
    return "#f44336"; // Red
  };

  return (
    <>
      <h2>🔋 Battery Status</h2>
      {connectionStatus !== "Connected" ? (
        <p style={{ color: "#666" }}>Connect to ROS to view battery data</p>
      ) : !batteryData ? (
        <p style={{ color: "#999" }}>Waiting for battery data...</p>
      ) : (
        <div>
          {/* Battery Percentage Bar */}
          <div style={{ marginBottom: "1.5rem" }}>
            <div
              style={{
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
                marginBottom: "0.5rem",
              }}
            >
              <strong>Charge Level</strong>
              <span
                style={{
                  fontSize: "1.2rem",
                  fontWeight: "bold",
                  color: getStatusColor(batteryData.percentage),
                }}
              >
                {batteryData.percentage.toFixed(1)}%
              </span>
            </div>
            <div
              style={{
                width: "100%",
                height: "24px",
                borderRadius: "12px",
                overflow: "hidden",
              }}
            >
              <div
                style={{
                  width: `${batteryData.percentage}%`,
                  height: "100%",
                  transition: "width 0.3s ease",
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  color: "white",
                  fontSize: "0.8rem",
                  fontWeight: "bold",
                }}
              >
                {batteryData.percentage > 10 &&
                  `${batteryData.percentage.toFixed(0)}%`}
              </div>
            </div>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>
              Electrical Properties
            </h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>Voltage:</td>
                  <td>{batteryData.voltage.toFixed(2)} V</td>
                </tr>
                <tr>
                  <td>Current:</td>
                  <td>{batteryData.current.toFixed(2)} A</td>
                </tr>
                <tr>
                  <td>Power:</td>
                  <td>
                    {(batteryData.voltage * batteryData.current).toFixed(2)} W
                  </td>
                </tr>
              </tbody>
            </table>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>Capacity</h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>Current Charge:</td>
                  <td>{batteryData.charge.toFixed(2)} Ah</td>
                </tr>
                <tr>
                  <td>Capacity:</td>
                  <td>{batteryData.capacity.toFixed(2)} Ah</td>
                </tr>
                <tr>
                  <td>Design Capacity:</td>
                  <td>{batteryData.design_capacity.toFixed(2)} Ah</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div style={{ marginBottom: "1.5rem" }}>
            <h3 style={{ marginTop: 0, marginBottom: "0.75rem" }}>Status</h3>
            <table className="data-table">
              <tbody>
                <tr>
                  <td>Power Supply Status:</td>
                  <td>
                    {POWER_SUPPLY_STATUS[batteryData.power_supply_status] ||
                      "Unknown"}
                  </td>
                </tr>
                <tr>
                  <td>Health:</td>
                  <td>
                    {POWER_SUPPLY_HEALTH[batteryData.power_supply_health] ||
                      "Unknown"}
                  </td>
                </tr>
              </tbody>
            </table>
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
              ✓ Subscribed to /battery_state
            </div>
          )}
        </div>
      )}
    </>
  );
}
