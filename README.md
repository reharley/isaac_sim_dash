# Isaac Sim ROS 2 Web Dashboard

A modern monorepo-based web dashboard for real-time visualization and control of Isaac Sim simulations via ROS 2, using **Vite + React + TypeScript** frontend and **Express + TypeScript** backend.

## 📋 Overview

| Component      | Technology             | Port      | Purpose                                             |
| -------------- | ---------------------- | --------- | --------------------------------------------------- |
| **Frontend**   | Vite + React + TS      | 5173      | Real-time dashboard UI with ROS topic visualization |
| **Backend**    | Express + TS           | 3001      | API proxy and helper services (optional)            |
| **ROS Bridge** | rosbridge_suite        | 9090 (WS) | Exposes ROS 2 topics to browser via WebSocket       |
| **Isaac Sim**  | ROS 2 Bridge Extension | -         | Publishes simulation data to ROS 2 topics           |

## 🏗️ Project Structure

```
isaac-sim-web-dashboard/
├── apps/
│   ├── frontend/                    # Vite React dashboard
│   │   ├── src/
│   │   │   ├── App.tsx            # Main component with ROS connection
│   │   │   ├── main.tsx           # Entry point
│   │   │   └── *.css              # Styling
│   │   ├── index.html
│   │   ├── package.json
│   │   ├── vite.config.ts
│   │   └── tsconfig.json
│   └── backend/                     # Express server
│       ├── src/
│       │   └── server.ts           # Express app
│       ├── package.json
│       └── tsconfig.json
├── packages/
│   └── shared/                      # Shared types & utils
│       ├── src/
│       │   └── index.ts            # ROS message types
│       ├── package.json
│       └── tsconfig.json
├── pnpm-workspace.yaml              # pnpm workspace config
├── package.json                     # Root scripts
├── tsconfig.json                    # Root TS config
└── README.md
```

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

If Isaac Sim is on a different machine, use its IP:

```bash
# In app browser or vite.config.ts:
ws://YOUR_ISAAC_SIM_IP:9090
```

### 5. Start Isaac Sim Simulation

- Open Isaac Sim
- Load or create your scene
- Enable **ROS 2 Bridge** extension
- Click **Play** to start the simulation
- Topics like `/joint_states`, `/odom`, `/clock` will be published

### Run Everything Together

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

Serve frontend static files from Express in production:

```typescript
app.use(express.static('../frontend/dist'));
```

## 📦 Dependencies

### Frontend

- **react** - UI library
- **roslibjs** - ROS 2 WebSocket client
- **@mui/material** - Material Design components (optional)
- **recharts** - Charting library (optional)
- **react-joystick-component** - Virtual joystick (optional)
- **vite** - Build tool
- **@vitejs/plugin-react** - React plugin for Vite

### Backend

- **express** - Web framework
- **cors** - CORS middleware
- **tsx** - TypeScript runner (dev)

### Shared

- **typescript** - Type checking

## 🐛 Troubleshooting

### Dashboard shows "Disconnected"

1. **Check rosbridge is running:**

   ```bash
   ros2 topic list
   ```

   If empty, rosbridge isn't running.

2. **Verify WebSocket URL:**
   Check browser console (F12) for connection errors.
   If Isaac Sim is remote: `ws://YOUR_IP:9090`

3. **Firewall issues:**
   Ensure port 9090 (TCP/UDP) is open between your machine and Isaac Sim host.

### No joint states appearing

1. Check Isaac Sim is in **PLAY** mode
2. Verify robot has joints and ROS 2 Bridge is enabled
3. In browser console, check for topic subscription errors

### TypeScript errors

```bash
pnpm type-check
```

### Port conflicts

- Frontend (5173): `vite --port 5174`
- Backend (3001): Change `PORT` in `apps/backend/src/server.ts`
- rosbridge (9090): Change in `rosbridge_websocket_launch.xml`

## 🚀 Next Steps / Ideas

1. **Add 3D Visualization:**

   - Integrate `react-three-fiber` + `ros3djs`
   - Display robot model from URDF

2. **Real-time Charts:**

   - Use `recharts` for joint position/velocity plots
   - Add IMU acceleration graphs

3. **Joystick Teleop:**

   - Use `react-joystick-component`
   - Send `Twist` commands to `/cmd_vel`

4. **Multiple Robots:**

   - Add topic filtering UI
   - Support multiple namespaced robots

5. **State Management:**

   - Use **Zustand** or **Jotai** for complex state
   - Persist dashboard settings (theme, subscribed topics)

6. **ROS Service Calls:**

   - Expose ROS services (e.g., grasp, reset)
   - Add service UI builders

7. **Turborepo Integration:**

   - Add `turbo.json` for optimized builds
   - Enable incremental builds & caching

8. **Docker Deployment:**
   - Containerize frontend + backend
   - Easy deployment on any machine with ROS 2

## 📚 Resources

- [roslibjs Docs](http://wiki.ros.org/roslibjs)
- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
- [Isaac Sim ROS 2 Bridge](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html)
- [Vite Docs](https://vitejs.dev/)
- [React Docs](https://react.dev/)

## 📄 License

MIT

---

**Made for Isaac Sim ROS 2 integration with ❤️**
