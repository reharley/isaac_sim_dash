# Isaac Sim ROS 2 Web Dashboard

Monorepo for real-time visualization and control of Isaac Sim simulations via ROS 2.

## Quick Start

```bash
# Install dependencies
pnpm install

# Start frontend (http://localhost:5173)
pnpm dev:frontend

# In another terminal, start backend (http://localhost:3001)
pnpm dev:backend

# On ROS 2 machine, launch rosbridge
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then start your Isaac Sim simulation!

## Structure

- `apps/frontend/` - Vite + React + TypeScript dashboard
- `apps/backend/` - Express + TypeScript API server
- `packages/shared/` - Shared types & utilities

See `README.md` for full documentation.
