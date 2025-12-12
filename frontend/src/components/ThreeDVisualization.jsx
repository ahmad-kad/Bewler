import React, { useRef, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Stats } from '@react-three/drei';
import * as THREE from 'three';

/**
<<<<<<< Current (Your changes)
 * Simple 3D Robot Model Component
 *
 * Renders a basic representation of the URC rover with animated rotation.
 * Includes base, head, arms, and wheels for visual representation.
 *
 * @component
 * @returns {JSX.Element} The 3D robot model
=======
 * Robot Model Component - URC 2026 Rover 3D Representation
 *
 * Renders a 3D model of the Mars rover using Three.js geometries. This component
 * provides a simplified visual representation of the rover's physical structure
 * for monitoring and control purposes.
 *
 * The model includes:
 * - Main body chassis
 * - Robotic arms for manipulation
 * - Wheels for mobility
 * - Head/camera assembly
 *
 * Features continuous rotation animation to indicate active status and provides
 * visual feedback during operation.
 *
 * @component
 * @returns {JSX.Element} 3D robot model with continuous rotation
 *
 * @example
 * ```jsx
 * <RobotModel />
 * ```
>>>>>>> Incoming (Background Agent changes)
 */
function RobotModel() {
  const meshRef = useRef();

  // Simple rotation animation
  useFrame((state, delta) => {
    if (meshRef.current) {
      meshRef.current.rotation.y += delta * 0.5;
    }
  });

  return (
    <group ref={meshRef} position={[0, 0, 0]}>
      {/* Robot Base */}
      <mesh position={[0, 0.5, 0]}>
        <boxGeometry args={[0.8, 1, 0.8]} />
        <meshStandardMaterial color="#4a5568" />
      </mesh>

      {/* Robot Head */}
      <mesh position={[0, 1.2, 0]}>
        <sphereGeometry args={[0.3]} />
        <meshStandardMaterial color="#2d3748" />
      </mesh>

      {/* Left Arm */}
      <mesh position={[-0.5, 0.8, 0]}>
        <cylinderGeometry args={[0.08, 0.08, 0.6]} />
        <meshStandardMaterial color="#1a202c" />
      </mesh>

      {/* Right Arm */}
      <mesh position={[0.5, 0.8, 0]}>
        <cylinderGeometry args={[0.08, 0.08, 0.6]} />
        <meshStandardMaterial color="#1a202c" />
      </mesh>

      {/* Wheels */}
      <mesh position={[-0.4, 0, -0.4]}>
        <cylinderGeometry args={[0.15, 0.15, 0.1]} />
        <meshStandardMaterial color="#1a202c" />
      </mesh>
      <mesh position={[0.4, 0, -0.4]}>
        <cylinderGeometry args={[0.15, 0.15, 0.1]} />
        <meshStandardMaterial color="#1a202c" />
      </mesh>
      <mesh position={[-0.4, 0, 0.4]}>
        <cylinderGeometry args={[0.15, 0.15, 0.1]} />
        <meshStandardMaterial color="#1a202c" />
      </mesh>
      <mesh position={[0.4, 0, 0.4]}>
        <cylinderGeometry args={[0.15, 0.15, 0.1]} />
        <meshStandardMaterial color="#1a202c" />
      </mesh>
    </group>
  );
}

// Environment Component
function Environment() {
  return (
    <>
      {/* Ground plane */}
      <mesh position={[0, -0.05, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[20, 20]} />
        <meshStandardMaterial color="#e2e8f0" />
      </mesh>

      {/* Grid helper */}
      <Grid args={[20, 20]} position={[0, 0, 0]} />
    </>
  );
}

// Lighting setup
function Lighting() {
  return (
    <>
      {/* Ambient light */}
      <ambientLight intensity={0.4} />

      {/* Directional light (sun) */}
      <directionalLight
        position={[10, 10, 5]}
        intensity={1}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
        shadow-camera-far={50}
        shadow-camera-left={-10}
        shadow-camera-right={10}
        shadow-camera-top={10}
        shadow-camera-bottom={-10}
      />

      {/* Point light for accent */}
      <pointLight position={[-10, 5, -10]} intensity={0.5} />
    </>
  );
}

/**
 * ThreeDVisualization Component
 *
 * Provides a 3D visualization interface for the URC rover using React Three Fiber.
 * Displays the robot model, environmental grid, and connection status indicators.
 * Supports interactive camera controls and real-time performance statistics.
 *
 * @component
 * @param {Object} props - Component properties
 * @param {boolean} props.isConnected - Whether the system is connected to ROS2
 * @returns {JSX.Element} The 3D visualization interface
 */
export default function ThreeDVisualization({ isConnected }) {
  const [showStats, setShowStats] = useState(false);

  return (
    <div className="scene-container">
      {/* 3D Controls Header */}
      <div style={{
        padding: '1rem',
        backgroundColor: '#1f2937',
        borderBottom: '1px solid #374151',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center'
      }}>
        <div>
          <h3 style={{
            color: '#60a5fa',
            margin: 0,
            fontSize: '1.125rem',
            fontWeight: '600'
          }}>
            3D Visualization
          </h3>
          <p style={{
            color: '#9ca3af',
            margin: '0.25rem 0 0 0',
            fontSize: '0.875rem'
          }}>
            {isConnected ? 'Connected to ROS - Real-time data' : 'Demo mode - Simulated environment'}
          </p>
        </div>

        <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
          <button
            onClick={() => setShowStats(!showStats)}
            style={{
              padding: '0.375rem 0.75rem',
              backgroundColor: showStats ? '#10b981' : '#374151',
              color: 'white',
              border: 'none',
              borderRadius: '0.25rem',
              fontSize: '0.75rem',
              cursor: 'pointer'
            }}
          >
            {showStats ? 'Hide Stats' : 'Show Stats'}
          </button>
        </div>
      </div>

      {/* 3D Canvas */}
      <div style={{ flex: 1, position: 'relative' }}>
        <Canvas
          camera={{ position: [5, 5, 5], fov: 60 }}
          shadows
          style={{ background: '#0f172a' }}
        >
          <Lighting />
          <Environment />
          <RobotModel />

          {/* Camera controls */}
          <OrbitControls
            enablePan={true}
            enableZoom={true}
            enableRotate={true}
            maxDistance={20}
            minDistance={2}
            maxPolarAngle={Math.PI / 2}
          />

          {/* Performance stats */}
          {showStats && <Stats />}
        </Canvas>

        {/* Overlay controls */}
        <div style={{
          position: 'absolute',
          top: '1rem',
          left: '1rem',
          backgroundColor: 'rgba(0, 0, 0, 0.7)',
          color: 'white',
          padding: '0.5rem',
          borderRadius: '0.375rem',
          fontSize: '0.75rem'
        }}>
          <div>Mouse: Orbit • Scroll: Zoom</div>
          <div>WASD: Move • Shift: Run</div>
          <div>Space: Jump</div>
        </div>
      </div>
    </div>
  );
}
