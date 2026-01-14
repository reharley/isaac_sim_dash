# Isaac Sim ROS 2 Web Dashboard

A modern monorepo-based web dashboard for real-time visualization and control of Isaac Sim simulations via ROS 2, using **Vite + React + TypeScript** frontend and **Express + TypeScript** backend.

## 📋 Overview

| Component      | Technology             | Port      | Purpose                                             |
| -------------- | ---------------------- | --------- | --------------------------------------------------- |
| **Frontend**   | Vite + React + TS      | 5173      | Real-time dashboard UI with ROS topic visualization |
| **Backend**    | Express + TS           | 3001      | API proxy and helper services (optional)            |
| **ROS Bridge** | rosbridge_suite        | 9090 (WS) | Exposes ROS 2 topics to browser via WebSocket       |
| **Isaac Sim**  | ROS 2 Bridge Extension | -         | Publishes simulation data to ROS 2 topics           |

## 🚀 Quick Start

### Prerequisites

- **Node.js** 18+ and **pnpm** 8+
- **ROS 2** (Humble or later) on your system or remote machine
- **rosbridge_suite** package
- **Isaac Sim** with ROS 2 Bridge extension enabled

### 1. Install Dependencies

```bash
pnpm install
```

### 2. Start the Frontend

```bash
pnpm dev:frontend
# or:
cd apps/frontend && pnpm dev
```

The dashboard opens at **http://localhost:5173**

### 3. Start the Backend (Optional)

```bash
pnpm dev:backend
# or:
cd apps/backend && pnpm dev
```

Backend runs on **http://localhost:3001**

### 4. Launch rosbridge_suite

On your ROS 2 machine (or in WSL/VM):

```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Launch rosbridge_websocket
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

rosbridge will be available at **ws://localhost:9090**

```bash
pnpm dev
```

This runs frontend + backend concurrently (requires terminal splits or background processes).

## 📡 Available ROS 2 Topics

Out of the box, Isaac Sim publishes:

- `/joint_states` - Joint positions, velocities, efforts
- `/odom` - Odometry data (pose + twist)
- `/clock` - Simulation clock
- `/imu` - IMU sensor data (if configured)
- `/camera/image_raw` - Camera RGB frames (if cameras added)

**Note:** Topic availability depends on your Isaac Sim scene configuration.

## 🛠️ Development

### Frontend Architecture

- **React Hooks** for ROS connection & state management
- **roslibjs** client for WebSocket → ROS 2 bridge
- **CSS Modules** for styling (dark/light theme support)
- **TypeScript** for type safety

### Backend Architecture

- **Express** for REST API (future: launch rosbridge, manage ROS services)
- **CORS** enabled for cross-origin requests
- **Error handling** middleware
- **TypeScript** strict mode

### Shared Types

Located in `packages/shared/src/index.ts`:

```typescript
export interface JointState {
  header: Header;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}
```

Use these in both frontend and backend for consistency.

## 🔧 Configuration

### Frontend: Connect to Remote rosbridge

In `apps/frontend/src/App.tsx`:

```typescript
const rosInstance = new ROSLIB.Ros({
  url: 'ws://192.168.1.100:9090', // Change IP as needed
});
```

Or via environment variable:

```typescript
const rosUrl = process.env.REACT_APP_ROS_URL || 'ws://localhost:9090';
```

### Backend: Add Custom Routes

In `apps/backend/src/server.ts`:

```typescript
app.post('/api/launch-service', (req, res) => {
  // Add custom logic here
  res.json({ success: true });
});
```

## 🧪 Build for Production

### Frontend

```bash
cd apps/frontend
pnpm build
pnpm preview
```

### Backend

```bash
cd apps/backend
pnpm build
pnpm start
```


