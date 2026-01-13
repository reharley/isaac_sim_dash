import cors from 'cors';
import express from 'express';

const app = express();
const PORT = 3001;

// Middleware
app.use(cors());
app.use(express.json());

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({
    status: 'ok',
    timestamp: new Date().toISOString(),
    service: 'Isaac Sim ROS 2 Backend',
  });
});

// API endpoints for future use
app.get('/api/status', (req, res) => {
  res.json({
    service: 'isaac-sim-backend',
    version: '1.0.0',
    rosbridge: {
      url: 'ws://localhost:9090',
      description: 'rosbridge_suite WebSocket connection',
    },
  });
});

// Error handling middleware
app.use(
  (
    err: any,
    req: express.Request,
    res: express.Response,
    next: express.NextFunction
  ) => {
    console.error('Error:', err);
    res.status(err.status || 500).json({
      error: err.message || 'Internal Server Error',
    });
  }
);

// 404 handler
app.use((req, res) => {
  res.status(404).json({
    error: 'Not Found',
    path: req.path,
  });
});

// Start server
app.listen(PORT, () => {
  console.log(`\n🚀 Backend server running at http://localhost:${PORT}`);
  console.log(`📡 ROS Bridge WebSocket: ws://localhost:9090\n`);
});
