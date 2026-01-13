# Isaac Sim Web Dashboard - Setup & Usage Guide

## ✅ Project Successfully Created!

Your monorepo is fully set up with all necessary files and configurations.

---

## 📁 Complete Project Structure

```
isaac-sim-web-dashboard/
├── .env.example                     # Environment variables template
├── .gitignore                       # Git ignore rules
├── .github/
│   └── workflows/
│       └── ci.yml                   # GitHub Actions CI pipeline
├── .vscode/
│   ├── extensions.json              # Recommended VS Code extensions
│   └── settings.json                # VS Code workspace settings
├── apps/
│   ├── backend/                     # Express.js + TypeScript backend
│   │   ├── src/
│   │   │   └── server.ts           # Express server entry point
│   │   ├── package.json
│   │   └── tsconfig.json
│   └── frontend/                    # Vite + React + TypeScript frontend
│       ├── src/
│       │   ├── App.tsx             # Main ROS connection component
│       │   ├── App.css             # Dashboard styling
│       │   ├── main.tsx            # React entry point
│       │   └── index.css           # Global styles
│       ├── index.html
│       ├── package.json
│       ├── vite.config.ts
│       ├── tsconfig.json
│       └── tsconfig.node.json
├── packages/
│   └── shared/                      # Shared types & utilities
│       ├── src/
│       │   └── index.ts            # ROS message type definitions
│       ├── package.json
│       └── tsconfig.json
├── pnpm-workspace.yaml              # pnpm monorepo configuration
├── package.json                     # Root package with workspace scripts
├── tsconfig.json                    # Root TypeScript configuration
├── Dockerfile                       # Docker image for production
├── docker-compose.yml               # Docker Compose for local development
├── nginx.conf                       # Nginx configuration for frontend
├── README.md                        # Full documentation
├── QUICKSTART.md                    # Quick start guide
└── SETUP_GUIDE.md                   # This file
```

---

## 🎯 Step-by-Step Setup

### Step 1: Install pnpm (if not already installed)

```bash
npm install -g pnpm
```

Verify:

```bash
pnpm --version
```

### Step 2: Install All Dependencies

Navigate to the root directory and install:

```bash
cd c:\Users\manny\Workspace\isaacsim_web
pnpm install
```

This installs dependencies for:

- Root workspace
- Frontend app
- Backend app
- Shared packages

### Step 3: Set Up Environment Variables

```bash
# Copy the example to .env
cp .env.example .env
```

Edit `.env` if needed (default values work for local development):

```
VITE_ROS_BRIDGE_URL=ws://localhost:9090
VITE_API_URL=http://localhost:3001
PORT=3001
NODE_ENV=development
```

---

## 🚀 Running the Application

### Option A: Run Everything Separately (Recommended for Development)

**Terminal 1 - Frontend:**

```bash
cd apps/frontend
pnpm dev
# Runs on http://localhost:5173
```

**Terminal 2 - Backend:**

```bash
cd apps/backend
pnpm dev
# Runs on http://localhost:3001
```

**Terminal 3 - ROS Bridge (on ROS 2 machine or WSL):**

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# WebSocket on ws://localhost:9090
```

### Option B: Run Frontend & Backend Together

```bash
pnpm dev
```

(You still need to launch rosbridge separately)

---

## 📱 Using the Dashboard

1. **Open** http://localhost:5173 in your browser
2. **Status Badge** shows connection to rosbridge:
   - 🟢 Green = Connected
   - 🟡 Yellow = Connecting
   - 🔴 Red = Error or Disconnected
3. **Joint States** table appears when Isaac Sim is playing
4. **Raw JSON** view available via "Raw JSON" dropdown

### What the Dashboard Does:

- ✅ Connects to rosbridge_suite WebSocket
- ✅ Subscribes to `/joint_states` topic
- ✅ Displays real-time joint positions, velocities, efforts
- ✅ Shows connection status and diagnostics
- ✅ Dark/Light theme support

---

## 🔧 Configuration

### Change ROS Bridge URL

**For Local Development (default):**

```typescript
// apps/frontend/src/App.tsx, line ~20
url: 'ws://localhost:9090';
```

**For Remote Isaac Sim (e.g., different machine):**

```typescript
url: 'ws://192.168.1.100:9090'; // Use your Isaac Sim machine IP
```

Or use environment variable:

```typescript
url: process.env.VITE_ROS_BRIDGE_URL || 'ws://localhost:9090';
```

### Change Frontend Port

Edit `apps/frontend/vite.config.ts`:

```typescript
export default defineConfig({
  server: {
    port: 5174, // Change from 5173
  },
});
```

### Change Backend Port

Edit `apps/backend/src/server.ts`:

```typescript
const PORT = 3002; // Change from 3001
```

---

## 🐛 Troubleshooting

### "Connection: Disconnected" in Dashboard

**Check 1:** Is rosbridge running?

```bash
ros2 topic list
```

If empty, rosbridge isn't running. Launch it:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Check 2:** Browser console errors (F12 → Console)

- `WebSocket is closed before the connection is established` → rosbridge not running
- `Connection refused` → Wrong IP or port

**Check 3:** Firewall

- Windows Firewall might block port 9090
- Add exception for Node/Vite if needed

### No Joint States Appearing

**Check 1:** Is Isaac Sim playing?

- Click the **Play** button in Isaac Sim
- Simulation must be running to publish topics

**Check 2:** Does the robot have joints?

- Verify `/joint_states` topic exists:
  ```bash
  ros2 topic list | grep joint_states
  ```

**Check 3:** ROS 2 Bridge Extension

- In Isaac Sim, enable the **ROS 2 Bridge** extension
- Check in **Extensions** → Search "ros"

### Port Already in Use

**Kill process on port:**

```bash
# Find process on port 5173
netstat -ano | findstr :5173
# Kill it (replace PID with actual process ID)
taskkill /PID <PID> /F
```

### TypeScript Errors

```bash
# Check all type errors in monorepo
pnpm type-check
```

### Node Modules Issue

```bash
# Clear cache and reinstall
pnpm store prune
rm -r node_modules apps/*/node_modules packages/*/node_modules
pnpm install
```

---

## 📦 Building for Production

### Build Frontend

```bash
cd apps/frontend
pnpm build
# Output in: apps/frontend/dist/
```

### Build Backend

```bash
cd apps/backend
pnpm build
# Output in: apps/backend/dist/
```

### Build Everything

```bash
# From root
pnpm build
```

---

## 🐳 Docker Deployment

### Build Docker Image

```bash
docker build -t isaac-sim-dashboard .
```

### Run with Docker Compose

```bash
docker-compose up
```

Then:

- Frontend: http://localhost:5173
- Backend API: http://localhost:3001
- ROS Bridge: Still needs to run on your ROS 2 machine

---

## 📝 Common Tasks

### Add New npm Package to Frontend

```bash
cd apps/frontend
pnpm add package-name
```

### Add New npm Package to Backend

```bash
cd apps/backend
pnpm add package-name
```

### Add Dev Dependency to Root

```bash
pnpm add -D package-name -w
```

### Type Check Everything

```bash
pnpm type-check
```

### Git Workflow

```bash
git add .
git commit -m "Initial commit: Isaac Sim web dashboard monorepo"
git branch -M main
git push -u origin main
```

---

## 🎨 Next Steps - Enhancements

### 1. Add Charts for Joint Data

```bash
cd apps/frontend
pnpm add recharts
```

Then update `App.tsx` to render charts with historical data.

### 2. Add 3D Visualization

```bash
pnpm add @react-three/fiber @react-three/drei
```

Render robot model + workspace.

### 3. Add Joystick Control

```bash
pnpm add react-joystick-component
```

Send `/cmd_vel` Twist messages.

### 4. Add Service UI

Call ROS 2 services from dashboard (e.g., reset, grasp).

### 5. Switch to Zustand for State Management

```bash
pnpm add zustand
```

For complex multi-topic subscriptions.

---

## 🔗 Useful Links

- **roslibjs:** http://wiki.ros.org/roslibjs
- **rosbridge_suite:** http://wiki.ros.org/rosbridge_suite
- **Isaac Sim Docs:** https://docs.omniverse.nvidia.com/app_isaacsim
- **ROS 2 Bridge Extension:** https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html
- **Vite:** https://vitejs.dev/
- **React:** https://react.dev/
- **Express:** https://expressjs.com/

---

## 📧 Support

For issues:

1. Check **README.md** troubleshooting section
2. Review browser console (F12)
3. Check ROS topics: `ros2 topic list`
4. Check ROS bridge logs

---

**Your Isaac Sim web dashboard monorepo is ready! 🚀**

Start by running:

```bash
pnpm install
pnpm dev:frontend
```

Then open http://localhost:5173 in your browser!
