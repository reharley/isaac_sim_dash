/**
 * ROS 2 Message Types
 * Definitions for common ROS 2 message types used with roslibjs
 */

export interface Header {
  stamp: {
    sec: number;
    nsec: number;
  };
  frame_id: string;
}

export interface JointState {
  header: Header;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

export interface Odometry {
  header: Header;
  child_frame_id: string;
  pose: {
    pose: {
      position: {
        x: number;
        y: number;
        z: number;
      };
      orientation: {
        x: number;
        y: number;
        z: number;
        w: number;
      };
    };
    covariance: number[];
  };
  twist: {
    twist: {
      linear: {
        x: number;
        y: number;
        z: number;
      };
      angular: {
        x: number;
        y: number;
        z: number;
      };
    };
    covariance: number[];
  };
}

export interface Clock {
  clock: {
    sec: number;
    nsec: number;
  };
}

export interface Twist {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

export interface Point {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

/**
 * Dashboard Configuration
 */
export interface DashboardConfig {
  rosbridge_url: string;
  topics: TopicSubscription[];
}

export interface TopicSubscription {
  name: string;
  message_type: string;
  enabled: boolean;
}

/**
 * API Response Types
 */
export interface ApiResponse<T> {
  status: 'ok' | 'error';
  data?: T;
  error?: string;
  timestamp: string;
}
