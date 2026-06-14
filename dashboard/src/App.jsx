import React, { useState, useEffect, useRef } from 'react';
import './App.css';

// Default configurations
const DEFAULT_API_URL = "http://localhost:5000";
const RESOLUTION = 0.1; // m/cell
const GRID_CELLS = 300; // 300x300 cells = 30m x 30m
const GRID_SIZE_M = 30.0;
const CANVAS_SIZE = 600; // 600x600 pixels

export default function App() {
  // Map and planner states
  const [mapLoaded, setMapLoaded] = useState(false);
  const [demoMode, setDemoMode] = useState(false);
  const [grid, setGrid] = useState(null);
  const [potentials, setPotentials] = useState(null);
  
  // Origin coordinates in world frame (default matching python side)
  const [originX, setOriginX] = useState(-4.3);
  const [originY, setOriginY] = useState(-10.8);

  // Interaction mode: 'start', 'goal', or 'none'
  const [interactionMode, setInteractionMode] = useState('start');
  
  // Pose states (in world coordinates)
  const [startX, setStartX] = useState(0.5);
  const [startY, setStartY] = useState(0.5);
  const [startTheta, setStartTheta] = useState(0.0);
  const [goalX, setGoalX] = useState(15.0);
  const [goalY, setGoalY] = useState(8.0);

  // Planner configurations
  const [allowDiagonal, setAllowDiagonal] = useState(false);
  const [inflationRadius, setInflationRadius] = useState(0.8);
  const [uWeight, setUWeight] = useState(15.0);
  const [viewMode, setViewMode] = useState('combined'); // 'raw', 'potential', 'combined'
  
  // Follower configurations
  const [kv, setKv] = useState(0.5);
  const [kw, setKw] = useState(1.5);
  const [lookahead, setLookahead] = useState(0.4);
  const [goalTol, setGoalTol] = useState(0.15);

  // Results states
  const [path, setPath] = useState(null);
  const [pathCells, setPathCells] = useState([]);
  const [expandedNodes, setExpandedNodes] = useState([]);
  const [planningMeta, setPlanningMeta] = useState(null);
  const [planningError, setPlanningError] = useState(null);

  // Simulation states
  const [simRunning, setSimRunning] = useState(false);
  const [simTrajectory, setSimTrajectory] = useState([]);
  const [simIndex, setSimIndex] = useState(0);
  const [currentPose, setCurrentPose] = useState({ x: 0.5, y: 0.5, theta: 0.0, v: 0.0, w: 0.0, err: 0.0 });
  const [simProgress, setSimProgress] = useState(0);

  // UI status
  const [apiConnected, setApiConnected] = useState(false);
  const [loading, setLoading] = useState(false);

  // Canvas and interaction references
  const canvasRef = useRef(null);
  const isDraggingStart = useRef(false);
  const dragStartPoint = useRef(null);
  const simIntervalRef = useRef(null);

  // Convert world to grid coords
  const worldToGrid = (x, y) => {
    const c = Math.floor((x - originX) / RESOLUTION);
    const r = Math.floor((y - originY) / RESOLUTION);
    return { r, c };
  };

  // Convert grid to world coords
  const gridToWorld = (r, c) => {
    const x = originX + (c + 0.5) * RESOLUTION;
    const y = originY + (r + 0.5) * RESOLUTION;
    return { x, y };
  };

  // Convert world to canvas pixel coordinates
  const worldToCanvas = (x, y) => {
    const { r, c } = worldToGrid(x, y);
    // Canvas y goes top-down, grid r goes bottom-up
    const px = (c / GRID_CELLS) * CANVAS_SIZE;
    const py = ((GRID_CELLS - 1 - r) / GRID_CELLS) * CANVAS_SIZE;
    return { px, py };
  };

  // Convert canvas pixel to world coordinates
  const canvasToWorld = (px, py) => {
    const c = Math.floor((px / CANVAS_SIZE) * GRID_CELLS);
    const r = GRID_CELLS - 1 - Math.floor((py / CANVAS_SIZE) * GRID_CELLS);
    return gridToWorld(r, c);
  };

  // Fetch map from API
  const fetchMap = async () => {
    setLoading(true);
    try {
      const res = await fetch(`${DEFAULT_API_URL}/api/map`);
      if (!res.ok) throw new Error("Could not connect to map API");
      const data = await res.json();
      
      setGrid(data.grid);
      setPotentials(data.potentials);
      setOriginX(data.origin_x);
      setOriginY(data.origin_y);
      setMapLoaded(true);
      setApiConnected(true);
      setDemoMode(false);
      
      // Auto-set start/goal to free cells
      findDefaultPositions(data.grid, data.origin_x, data.origin_y);
    } catch (e) {
      console.warn("Backend API offline. Starting Demo mode with a synthesized environment.");
      setApiConnected(false);
      setDemoMode(true);
      generateDemoMap();
    } finally {
      setLoading(false);
    }
  };

  // Set default poses based on free cells
  const findDefaultPositions = (mapGrid, origX, origY) => {
    let sR = 30, sC = 30; // Find first free spot
    let gR = 150, gC = 150;
    
    // Simple scan for start
    for (let r = 20; r < GRID_CELLS; r++) {
      let found = false;
      for (let c = 20; c < GRID_CELLS; c++) {
        if (mapGrid[r][c] === 0) {
          sR = r; sC = c;
          found = true;
          break;
        }
      }
      if (found) break;
    }

    // Simple scan for goal
    for (let r = GRID_CELLS - 30; r > 0; r--) {
      let found = false;
      for (let c = GRID_CELLS - 30; c > 0; c--) {
        if (mapGrid[r][c] === 0 && Math.abs(r - sR) > 50) {
          gR = r; gC = c;
          found = true;
          break;
        }
      }
      if (found) break;
    }

    const startPose = gridToWorldOffset(sR, sC, origX, origY);
    const goalPose = gridToWorldOffset(gR, gC, origX, origY);
    
    setStartX(startPose.x);
    setStartY(startPose.y);
    setStartTheta(0.0);
    setGoalX(goalPose.x);
    setGoalY(goalPose.y);
  };

  const gridToWorldOffset = (r, c, ox, oy) => {
    return {
      x: ox + (c + 0.5) * RESOLUTION,
      y: oy + (r + 0.5) * RESOLUTION
    };
  };

  // Generate synthetic map for offline demo mode
  const generateDemoMap = () => {
    const newGrid = Array(GRID_CELLS).fill(null).map(() => Array(GRID_CELLS).fill(0));
    
    // Add border walls
    for (let i = 0; i < GRID_CELLS; i++) {
      newGrid[0][i] = 1;
      newGrid[GRID_CELLS - 1][i] = 1;
      newGrid[i][0] = 1;
      newGrid[i][GRID_CELLS - 1] = 1;
    }

    // Add some random rooms/walls
    // Room 1: Vertical dividing wall in center with a door
    for (let r = 50; r < 250; r++) {
      if (r < 130 || r > 170) {
        newGrid[r][140] = 1;
      }
    }

    // Room 2: Horizontal dividing wall
    for (let c = 50; c < 200; c++) {
      if (c < 100 || c > 130) {
        newGrid[100][c] = 1;
      }
    }

    // Room 3: A circular obstacle
    const cx = 200, cy = 200, rad = 25;
    for (let r = 0; r < GRID_CELLS; r++) {
      for (let c = 0; c < GRID_CELLS; c++) {
        const d = Math.hypot(r - cy, c - cx);
        if (d < rad) {
          newGrid[r][c] = 1;
        }
      }
    }

    // Compute synthetic distance transform & potentials
    const dist = Array(GRID_CELLS).fill(null).map(() => Array(GRID_CELLS).fill(1e9));
    const q = [];
    
    for (let r = 0; r < GRID_CELLS; r++) {
      for (let c = 0; c < GRID_CELLS; c++) {
        if (newGrid[r][c] === 1) {
          dist[r][c] = 0;
          q.push([r, c]);
        }
      }
    }

    const dirs = [[0,1], [0,-1], [1,0], [-1,0], [1,1], [1,-1], [-1,1], [-1,-1]];
    let idx = 0;
    while (idx < q.length) {
      const [r, c] = q[idx++];
      const d = dist[r][c];
      
      for (const [dr, dc] of dirs) {
        const nr = r + dr;
        const nc = c + dc;
        const step = (dr !== 0 && dc !== 0) ? 1.414 : 1.0;
        
        if (nr >= 0 && nr < GRID_CELLS && nc >= 0 && nc < GRID_CELLS) {
          if (dist[nr][nc] > d + step) {
            dist[nr][nc] = d + step;
            q.push([nr, nc]);
          }
        }
      }
    }

    const inflCells = inflationRadius / RESOLUTION;
    const newPotentials = Array(GRID_CELLS).fill(null).map((_, r) => 
      Array(GRID_CELLS).fill(null).map((_, c) => {
        const cellDist = dist[r][c] * RESOLUTION;
        if (cellDist < inflationRadius && cellDist > 0) {
          return uWeight * Math.pow(1.0 - cellDist / inflationRadius, 2);
        }
        return 0;
      })
    );

    setGrid(newGrid);
    setPotentials(newPotentials);
    setOriginX(-15.0); // center demo map on (-15, -15) to (15, 15)
    setOriginY(-15.0);
    setMapLoaded(true);

    // Initial positions
    setStartX(-10.0);
    setStartY(-10.0);
    setStartTheta(0.7);
    setGoalX(10.0);
    setGoalY(10.0);
  };

  // Re-calculate potentials locally in Demo Mode
  const updateDemoPotentials = () => {
    if (!demoMode || !grid) return;
    
    // Simple re-run distance transform
    const dist = Array(GRID_CELLS).fill(null).map(() => Array(GRID_CELLS).fill(1e9));
    const q = [];
    for (let r = 0; r < GRID_CELLS; r++) {
      for (let c = 0; c < GRID_CELLS; c++) {
        if (grid[r][c] === 1) {
          dist[r][c] = 0;
          q.push([r, c]);
        }
      }
    }

    const dirs = [[0,1], [0,-1], [1,0], [-1,0], [1,1], [1,-1], [-1,1], [-1,-1]];
    let idx = 0;
    while (idx < q.length) {
      const [r, c] = q[idx++];
      const d = dist[r][c];
      for (const [dr, dc] of dirs) {
        const nr = r + dr;
        const nc = c + dc;
        const step = (dr !== 0 && dc !== 0) ? 1.414 : 1.0;
        if (nr >= 0 && nr < GRID_CELLS && nc >= 0 && nc < GRID_CELLS) {
          if (dist[nr][nc] > d + step) {
            dist[nr][nc] = d + step;
            q.push([nr, nc]);
          }
        }
      }
    }

    const newPotentials = Array(GRID_CELLS).fill(null).map((_, r) => 
      Array(GRID_CELLS).fill(null).map((_, c) => {
        const cellDist = dist[r][c] * RESOLUTION;
        if (cellDist < inflationRadius && cellDist > 0) {
          return uWeight * Math.pow(1.0 - cellDist / inflationRadius, 2);
        }
        return 0;
      })
    );
    setPotentials(newPotentials);
  };

  useEffect(() => {
    fetchMap();
  }, []);

  useEffect(() => {
    if (demoMode) {
      updateDemoPotentials();
    }
  }, [inflationRadius, uWeight]);

  // Handle Canvas Drawing
  useEffect(() => {
    if (!mapLoaded || !grid || !canvasRef.current) return;
    drawCanvas();
  }, [grid, potentials, startX, startY, startTheta, goalX, goalY, path, pathCells, expandedNodes, viewMode, currentPose, simRunning]);

  const drawCanvas = () => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    
    // Clear canvas
    ctx.clearRect(0, 0, CANVAS_SIZE, CANVAS_SIZE);
    
    // Draw cells
    const cellPixelSize = CANVAS_SIZE / GRID_CELLS;
    
    for (let r = 0; r < GRID_CELLS; r++) {
      // Invert row index for drawing (top-down)
      const py = (GRID_CELLS - 1 - r) * cellPixelSize;
      
      for (let c = 0; c < GRID_CELLS; c++) {
        const px = c * cellPixelSize;
        const cellType = grid[r][c];
        const cellPot = potentials ? potentials[r][c] : 0;
        
        if (cellType === 1) {
          // Obstacle
          ctx.fillStyle = '#0f111a';
          ctx.fillRect(px, py, cellPixelSize, cellPixelSize);
        } else {
          // Free cell background
          ctx.fillStyle = '#1b1d28';
          ctx.fillRect(px, py, cellPixelSize, cellPixelSize);
          
          // Render potential field heatmap if requested
          if (cellPot > 0.05 && (viewMode === 'potential' || viewMode === 'combined')) {
            const intensity = Math.min(cellPot / uWeight, 1.0);
            ctx.fillStyle = `rgba(239, 68, 68, ${intensity * 0.7})`; // red overlay
            ctx.fillRect(px, py, cellPixelSize, cellPixelSize);
          }
        }
      }
    }

    // Draw expanded nodes (Search frontier)
    if (expandedNodes && expandedNodes.length > 0 && !simRunning) {
      ctx.fillStyle = 'rgba(59, 130, 246, 0.25)'; // Light blue
      expandedNodes.forEach(([r, c]) => {
        const px = c * cellPixelSize;
        const py = (GRID_CELLS - 1 - r) * cellPixelSize;
        ctx.fillRect(px, py, cellPixelSize, cellPixelSize);
      });
    }

    // Draw planned path
    if (pathCells && pathCells.length > 0) {
      ctx.strokeStyle = '#10b981'; // green path
      ctx.lineWidth = 3;
      ctx.shadowColor = 'rgba(16, 185, 129, 0.6)';
      ctx.shadowBlur = 10;
      ctx.beginPath();
      
      pathCells.forEach(([r, c], idx) => {
        const px = (c + 0.5) * cellPixelSize;
        const py = (GRID_CELLS - 1 - r + 0.5) * cellPixelSize;
        if (idx === 0) {
          ctx.moveTo(px, py);
        } else {
          ctx.lineTo(px, py);
        }
      });
      ctx.stroke();
      ctx.shadowBlur = 0; // reset
    }

    // Draw Goal marker
    const gCanvas = worldToCanvas(goalX, goalY);
    ctx.fillStyle = '#ef4444'; // Red Goal
    ctx.shadowColor = 'rgba(239, 68, 68, 0.8)';
    ctx.shadowBlur = 12;
    ctx.beginPath();
    ctx.arc(gCanvas.px, gCanvas.py, 8, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;
    // Target inner circle
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(gCanvas.px, gCanvas.py, 4, 0, 2 * Math.PI);
    ctx.stroke();

    // Draw Start / Robot
    const rPose = simRunning ? currentPose : { x: startX, y: startY, theta: startTheta };
    const rCanvas = worldToCanvas(rPose.x, rPose.y);
    
    // Draw robot physical envelope (0.4m radius = 4 cells = 8px radius)
    const rRadiusPx = (0.4 / RESOLUTION) * cellPixelSize;
    
    // Robot body (glowing blue-green)
    ctx.fillStyle = simRunning ? 'rgba(59, 130, 246, 0.75)' : 'rgba(16, 185, 129, 0.75)';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.shadowColor = simRunning ? 'rgba(59, 130, 246, 0.8)' : 'rgba(16, 185, 129, 0.8)';
    ctx.shadowBlur = 15;
    ctx.beginPath();
    ctx.arc(rCanvas.px, rCanvas.py, rRadiusPx, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Draw nose arrow showing orientation
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(rCanvas.px, rCanvas.py);
    ctx.lineTo(
      rCanvas.px + rRadiusPx * 1.5 * Math.cos(rPose.theta),
      rCanvas.px - rRadiusPx * 1.5 * Math.sin(rPose.theta) // Canvas y-axis inverted
    );
    ctx.stroke();

    // Draw a simulated 8-beam range finder scan
    if (simRunning) {
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.45)';
      ctx.lineWidth = 1;
      const numBeams = 8;
      const maxScanDist = 4.0; // meters
      
      for (let i = 0; i < numBeams; i++) {
        const angle = rPose.theta + (i - numBeams/2) * (Math.PI / 4);
        // Cast a simple ray on the grid
        let beamLen = 0;
        const step = 0.05; // 5cm step
        
        while (beamLen < maxScanDist) {
          const bx = rPose.x + beamLen * Math.cos(angle);
          const by = rPose.y + beamLen * Math.sin(angle);
          const { r, c } = worldToGrid(bx, by);
          
          if (r < 0 || r >= GRID_CELLS || c < 0 || c >= GRID_CELLS || grid[r][c] === 1) {
            break;
          }
          beamLen += step;
        }
        
        const scanEndCanvas = worldToCanvas(
          rPose.x + beamLen * Math.cos(angle),
          rPose.y + beamLen * Math.sin(angle)
        );
        
        ctx.beginPath();
        ctx.moveTo(rCanvas.px, rCanvas.py);
        ctx.lineTo(scanEndCanvas.px, scanEndCanvas.py);
        ctx.stroke();
        
        // Dot at end
        ctx.fillStyle = '#ef4444';
        ctx.beginPath();
        ctx.arc(scanEndCanvas.px, scanEndCanvas.py, 3, 0, 2*Math.PI);
        ctx.fill();
      }
    }
  };

  // Drag to set start pose and heading
  const handleCanvasMouseDown = (e) => {
    if (simRunning) return;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const worldPos = canvasToWorld(px, py);

    if (interactionMode === 'start') {
      isDraggingStart.current = true;
      dragStartPoint.current = worldPos;
      setStartX(worldPos.x);
      setStartY(worldPos.y);
      setPath(null);
    } else if (interactionMode === 'goal') {
      setGoalX(worldPos.x);
      setGoalY(worldPos.y);
      setPath(null);
    }
  };

  const handleCanvasMouseMove = (e) => {
    if (!isDraggingStart.current || interactionMode !== 'start') return;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const worldPos = canvasToWorld(px, py);
    
    // Compute angle from start point to current mouse position
    const dx = worldPos.x - dragStartPoint.current.x;
    const dy = worldPos.y - dragStartPoint.current.y;
    const angle = Math.atan2(dy, dx);
    setStartTheta(angle);
  };

  const handleCanvasMouseUp = () => {
    isDraggingStart.current = false;
  };

  // Plan Path request
  const planPath = async () => {
    if (simRunning) stopSimulation();
    
    setLoading(true);
    setPlanningError(null);
    setPath(null);
    setExpandedNodes([]);
    
    const startPose = [startX, startY];
    const goalPose = [goalX, goalY];
    
    if (demoMode) {
      // Local A* fallback implementation
      runDemoPlanner();
      setLoading(false);
      return;
    }

    try {
      const res = await fetch(`${DEFAULT_API_URL}/api/plan`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          start: startPose,
          goal: goalPose,
          allow_diagonal: allowDiagonal,
          inflation_radius: inflationRadius,
          u_weight: uWeight
        })
      });
      const data = await res.json();
      
      if (data.success) {
        setPath(data.path);
        setPathCells(data.metadata.path_cells);
        setExpandedNodes(data.metadata.expanded_nodes);
        setPlanningMeta(data.metadata);
      } else {
        setPlanningError(data.error);
      }
    } catch (e) {
      setPlanningError("Network error contacting path-planning backend: " + e.message);
    } finally {
      setLoading(false);
    }
  };

  // Local planner for Demo Mode
  const runDemoPlanner = () => {
    const startTime = performance.now();
    const startGrid = worldToGrid(startX, startY);
    const goalGrid = worldToGrid(goalX, goalY);

    if (grid[startGrid.r][startGrid.c] === 1 || grid[goalGrid.r][goalGrid.c] === 1) {
      setPlanningError("Start or Goal is inside an obstacle!");
      return;
    }

    // A* implementation in JS
    const openSet = [];
    const openSetMap = new Map();
    const closedSet = new Set();
    const expandedList = [];
    
    const key = (r, c) => `${r},${c}`;
    const startKey = key(startGrid.r, startGrid.c);
    
    const gScore = new Map();
    gScore.set(startKey, 0);
    
    const parent = new Map();
    
    const h = (r, c) => Math.abs(r - goalGrid.r) + Math.abs(c - goalGrid.c);
    
    // Priority queue insert
    const push = (item) => {
      openSet.push(item);
      openSet.sort((a, b) => a.f - b.f);
      openSetMap.set(key(item.r, item.c), item.f);
    };

    push({ r: startGrid.r, c: startGrid.c, f: h(startGrid.r, startGrid.c) });

    const dirs = allowDiagonal 
      ? [[0,1,1.0],[0,-1,1.0],[1,0,1.0],[-1,0,1.0],[1,1,1.414],[1,-1,1.414],[-1,1,1.414],[-1,-1,1.414]]
      : [[0,1,1.0],[0,-1,1.0],[1,0,1.0],[-1,0,1.0]];

    let found = false;
    
    while (openSet.length > 0) {
      const current = openSet.shift();
      const currKey = key(current.r, current.c);
      openSetMap.delete(currKey);

      if (current.r === goalGrid.r && current.c === goalGrid.c) {
        found = true;
        break;
      }

      closedSet.add(currKey);
      expandedList.push([current.r, current.c]);

      const currG = gScore.get(currKey);

      for (const [dr, dc, stepCost] of dirs) {
        const nr = current.r + dr;
        const nc = current.c + dc;
        
        if (nr >= 0 && nr < GRID_CELLS && nc >= 0 && nc < GRID_CELLS) {
          if (grid[nr][nc] === 1) continue;
          
          const neighKey = key(nr, nc);
          if (closedSet.has(neighKey)) continue;

          const potVal = potentials ? potentials[nr][nc] : 0;
          const tentativeG = currG + stepCost + potVal;

          if (!gScore.has(neighKey) || tentativeG < gScore.get(neighKey)) {
            gScore.set(neighKey, tentativeG);
            parent.set(neighKey, { r: current.r, c: current.c });
            
            const f = tentativeG + h(nr, nc);
            if (!openSetMap.has(neighKey)) {
              push({ r: nr, c: nc, f });
            }
          }
        }
      }
    }

    const duration = performance.now() - startTime;

    if (!found) {
      setPlanningError("No path found (A* failed).");
      return;
    }

    // Reconstruct
    const pCells = [];
    let curr = goalGrid;
    while (curr) {
      pCells.push([curr.r, curr.c]);
      const next = parent.get(key(curr.r, curr.c));
      curr = next;
    }
    pCells.reverse();

    const pWorld = pCells.map(([r, c]) => {
      const w = gridToWorld(r, c);
      return [w.x, w.y];
    });

    setPath(pWorld);
    setPathCells(pCells);
    setExpandedNodes(expandedList);
    setPlanningMeta({
      time_ms: duration,
      path_length_m: pWorld.length * RESOLUTION,
      expanded_count: closedSet.size
    });
  };

  // Run Path Following Simulation
  const startSimulation = async () => {
    if (!path || path.length === 0) return;
    
    if (demoMode) {
      runDemoSimulation();
      return;
    }

    setLoading(true);
    try {
      const res = await fetch(`${DEFAULT_API_URL}/api/simulate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          path: path,
          start_pose: [startX, startY, startTheta],
          kv: kv,
          kw: kw,
          lookahead: lookahead,
          goal_tol: goalTol
        })
      });
      const data = await res.json();
      
      if (data.success && data.trajectory.length > 0) {
        setSimTrajectory(data.trajectory);
        setSimIndex(0);
        setSimRunning(true);
        setCurrentPose(data.trajectory[0]);
      } else {
        alert("Failed to compute simulation trajectory");
      }
    } catch (e) {
      alert("Network error: " + e.message + ". Starting local demo simulation instead.");
      runDemoSimulation();
    } finally {
      setLoading(false);
    }
  };

  // Simple client-side simulation fallback
  const runDemoSimulation = () => {
    // Basic differential-drive kinematics simulation
    const trajectory = [];
    let rx = startX, ry = startY, rtheta = startTheta;
    let targetIdx = 0;
    const dt = 0.05;
    const maxSteps = 1000;
    
    trajectory.push({ x: rx, y: ry, theta: rtheta, v: 0, w: 0, err: 0 });

    for (let step = 0; step < maxSteps; step++) {
      const goal_x = path[path.length - 1][0];
      const goal_y = path[path.length - 1][1];
      const dist_to_goal = Math.hypot(goal_x - rx, goal_y - ry);
      
      if (dist_to_goal < goalTol) {
        break;
      }
      
      let target_pt = path[path.length - 1];
      for (let idx = targetIdx; idx < path.length; idx++) {
        const wx = path[idx][0];
        const wy = path[idx][1];
        const d = Math.hypot(wx - rx, wy - ry);
        if (d >= lookahead) {
          targetIdx = idx;
          target_pt = [wx, wy];
          break;
        }
      }
      
      const tx = target_pt[0];
      const ty = target_pt[1];
      const target_angle = Math.atan2(ty - ry, tx - rx);
      const angle_err = Math.atan2(Math.sin(target_angle - rtheta), Math.cos(target_angle - rtheta));
      
      let v = 0.0, w = 0.0;
      if (Math.abs(angle_err) > 0.5) {
        v = 0.0;
        w = kw * Math.sign(angle_err) * 0.6;
      } else {
        v = kv * Math.min(dist_to_goal, 0.5);
        v = v * Math.cos(angle_err);
        w = kw * angle_err;
      }
      
      v = Math.max(-0.1, Math.min(v, 0.4));
      w = Math.max(-1.0, Math.min(w, 1.0));
      
      rx = rx + v * Math.cos(rtheta) * dt;
      ry = ry + v * Math.sin(rtheta) * dt;
      rtheta = rtheta + w * dt;
      // Normalize
      rtheta = Math.atan2(Math.sin(rtheta), Math.cos(rtheta));
      
      trajectory.push({
        x: rx, y: ry, theta: rtheta,
        v, w, tx, ty,
        err: Math.hypot(tx - rx, ty - ry)
      });
    }

    setSimTrajectory(trajectory);
    setSimIndex(0);
    setSimRunning(true);
    setCurrentPose(trajectory[0]);
  };

  // Playback loop for simulation
  useEffect(() => {
    if (!simRunning || simTrajectory.length === 0) return;
    
    simIntervalRef.current = setInterval(() => {
      setSimIndex((prevIdx) => {
        const nextIdx = prevIdx + 1;
        if (nextIdx >= simTrajectory.length) {
          clearInterval(simIntervalRef.current);
          setSimRunning(false);
          return prevIdx;
        }
        
        const nextPose = simTrajectory[nextIdx];
        setCurrentPose(nextPose);
        setSimProgress(Math.round((nextIdx / (simTrajectory.length - 1)) * 100));
        return nextIdx;
      });
    }, 50); // 50ms (matches simulation dt)

    return () => clearInterval(simIntervalRef.current);
  }, [simRunning, simTrajectory]);

  const stopSimulation = () => {
    clearInterval(simIntervalRef.current);
    setSimRunning(false);
    setSimIndex(0);
    setSimProgress(0);
  };

  const clearPlanner = () => {
    stopSimulation();
    setPath(null);
    setPathCells([]);
    setExpandedNodes([]);
    setPlanningMeta(null);
    setPlanningError(null);
  };

  return (
    <div className="app-container">
      {/* Top Header */}
      <header className="app-header">
        <div className="header-title-section">
          <h1>Pioneer 3AT Path Planning</h1>
          <p>
            {demoMode 
              ? "DEMO MODE (Synthesized 30m x 30m grid)" 
              : `CONNECTED TO LOCAL SERVER (${GRID_SIZE_M}m x ${GRID_SIZE_M}m Window)`
            }
          </p>
        </div>
        <div className="connection-badge">
          <div className={`badge-dot ${apiConnected ? 'connected' : ''}`}></div>
          {apiConnected ? 'ROS API Server Connected' : 'Offline / Standalone Simulator'}
        </div>
      </header>

      {/* Main Content Area */}
      <div className="app-content">
        {/* Left Control Panel */}
        <aside className="sidebar">
          
          {/* A* Settings Card */}
          <div className="glass-card">
            <h2 className="card-title">Planner Configuration</h2>
            
            <div className="control-group">
              <label className="control-label">
                Heuristic <span>Manhattan Distance</span>
              </label>
              <div className="toggle-group">
                <button 
                  className={`toggle-btn ${!allowDiagonal ? 'active' : ''}`}
                  onClick={() => setAllowDiagonal(false)}
                >
                  4-Way Connected
                </button>
                <button 
                  className={`toggle-btn ${allowDiagonal ? 'active' : ''}`}
                  onClick={() => setAllowDiagonal(true)}
                >
                  8-Way Connected (Diag)
                </button>
              </div>
            </div>

            <div className="control-group">
              <label className="control-label">
                Inflation Radius 
                <span className="control-val">{inflationRadius.toFixed(1)}m</span>
              </label>
              <input 
                type="range" 
                min="0.3" 
                max="2.5" 
                step="0.1" 
                value={inflationRadius} 
                onChange={(e) => setInflationRadius(parseFloat(e.target.value))}
              />
            </div>

            <div className="control-group">
              <label className="control-label">
                Repulsion Weight 
                <span className="control-val">{uWeight.toFixed(0)}</span>
              </label>
              <input 
                type="range" 
                min="0" 
                max="100" 
                step="5" 
                value={uWeight} 
                onChange={(e) => setUWeight(parseFloat(e.target.value))}
              />
            </div>
          </div>

          {/* Controller Settings Card */}
          <div className="glass-card">
            <h2 className="card-title">Path Follower</h2>
            
            <div className="control-group">
              <label className="control-label">
                Linear Gain (Kp v) 
                <span className="control-val">{kv.toFixed(1)}</span>
              </label>
              <input 
                type="range" 
                min="0.1" 
                max="1.5" 
                step="0.1" 
                value={kv} 
                onChange={(e) => setKv(parseFloat(e.target.value))}
              />
            </div>

            <div className="control-group">
              <label className="control-label">
                Angular Gain (Kp w) 
                <span className="control-val">{kw.toFixed(1)}</span>
              </label>
              <input 
                type="range" 
                min="0.5" 
                max="3.0" 
                step="0.1" 
                value={kw} 
                onChange={(e) => setKw(parseFloat(e.target.value))}
              />
            </div>

            <div className="control-group">
              <label className="control-label">
                Lookahead Distance 
                <span className="control-val">{lookahead.toFixed(2)}m</span>
              </label>
              <input 
                type="range" 
                min="0.2" 
                max="1.5" 
                step="0.05" 
                value={lookahead} 
                onChange={(e) => setLookahead(parseFloat(e.target.value))}
              />
            </div>
          </div>

          {/* Initial/Goal manual coordinates */}
          <div className="glass-card">
            <h2 className="card-title">Grid Coordinates</h2>
            
            <div className="control-group">
              <label className="control-label">Start Position (X, Y, Heading θ)</label>
              <div className="coord-grid">
                <div className="coord-input-wrapper">
                  <span className="coord-prefix">X</span>
                  <input 
                    type="number" step="0.1" className="coord-input" 
                    value={startX.toFixed(1)} 
                    onChange={(e) => { setStartX(parseFloat(e.target.value)); setPath(null); }}
                  />
                </div>
                <div className="coord-input-wrapper">
                  <span className="coord-prefix">Y</span>
                  <input 
                    type="number" step="0.1" className="coord-input" 
                    value={startY.toFixed(1)} 
                    onChange={(e) => { setStartY(parseFloat(e.target.value)); setPath(null); }}
                  />
                </div>
                <div className="coord-input-wrapper">
                  <span className="coord-prefix">θ</span>
                  <input 
                    type="number" step="0.1" className="coord-input" 
                    value={startTheta.toFixed(2)} 
                    onChange={(e) => setStartTheta(parseFloat(e.target.value))}
                  />
                </div>
              </div>
            </div>

            <div className="control-group">
              <label className="control-label">Goal Position (X, Y)</label>
              <div className="coord-grid">
                <div className="coord-input-wrapper">
                  <span className="coord-prefix">X</span>
                  <input 
                    type="number" step="0.1" className="coord-input" 
                    value={goalX.toFixed(1)} 
                    onChange={(e) => { setGoalX(parseFloat(e.target.value)); setPath(null); }}
                  />
                </div>
                <div className="coord-input-wrapper">
                  <span className="coord-prefix">Y</span>
                  <input 
                    type="number" step="0.1" className="coord-input" 
                    value={goalY.toFixed(1)} 
                    onChange={(e) => { setGoalY(parseFloat(e.target.value)); setPath(null); }}
                  />
                </div>
              </div>
            </div>
          </div>
        </aside>

        {/* Center Visualizer */}
        <main className="visualizer-area">
          {/* Interaction Mode Buttons */}
          <div className="mode-selectors">
            <button 
              className={`mode-btn ${interactionMode === 'start' ? 'active-start' : ''}`}
              onClick={() => setInteractionMode('start')}
            >
              📍 Set Start Pose
            </button>
            <button 
              className={`mode-btn ${interactionMode === 'goal' ? 'active-goal' : ''}`}
              onClick={() => setInteractionMode('goal')}
            >
              🏁 Set Goal Point
            </button>
          </div>

          {/* Layer Overlay Selector */}
          <div className="layer-selector">
            <button 
              className={`layer-btn ${viewMode === 'raw' ? 'active' : ''}`}
              onClick={() => setViewMode('raw')}
            >
              Raw Map
            </button>
            <button 
              className={`layer-btn ${viewMode === 'potential' ? 'active' : ''}`}
              onClick={() => setViewMode('potential')}
            >
              Potentials
            </button>
            <button 
              className={`layer-btn ${viewMode === 'combined' ? 'active' : ''}`}
              onClick={() => setViewMode('combined')}
            >
              Combined
            </button>
          </div>

          {/* Grid Canvas */}
          <div className="canvas-container">
            {loading && <div className="loader-overlay">Planning path...</div>}
            
            {mapLoaded ? (
              <canvas
                ref={canvasRef}
                width={CANVAS_SIZE}
                height={CANVAS_SIZE}
                className="planner-canvas"
                onMouseDown={handleCanvasMouseDown}
                onMouseMove={handleCanvasMouseMove}
                onMouseUp={handleCanvasMouseUp}
                onMouseLeave={handleCanvasMouseUp}
              />
            ) : (
              <div className="loading-map">Loading map data...</div>
            )}
          </div>

          {/* Control overlay */}
          <div className="visualizer-overlay">
            <button className="btn btn-primary" onClick={planPath} disabled={loading}>
              ⚡ Plan Path
            </button>
            
            {!simRunning ? (
              <button className="btn btn-success" onClick={startSimulation} disabled={!path}>
                ▶ Run Simulation
              </button>
            ) : (
              <button className="btn btn-danger" onClick={stopSimulation}>
                ■ Stop Simulation
              </button>
            )}
            
            <button className="btn" onClick={clearPlanner}>
              ↺ Reset
            </button>
          </div>
        </main>

        {/* Right Info Sidebar */}
        <aside className="sidebar">
          {/* Metrics Card */}
          <div className="glass-card">
            <h2 className="card-title">Planning Statistics</h2>
            
            <div className="metrics-row">
              <div className="metric-card">
                <div className="metric-info">
                  <span className="metric-label">Search Status</span>
                  <div style={{ marginTop: '4px' }}>
                    {planningError ? (
                      <span className="status-badge error">FAILED</span>
                    ) : path ? (
                      <span className="status-badge success">OPTIMAL</span>
                    ) : (
                      <span className="status-badge none">STANDBY</span>
                    )}
                  </div>
                </div>
                <div className="metric-icon">🏁</div>
              </div>

              <div className="metric-card">
                <div className="metric-info">
                  <span className="metric-label">Execution Time</span>
                  <span className="metric-val">
                    {planningMeta ? `${planningMeta.time_ms.toFixed(2)} ms` : '--'}
                  </span>
                </div>
                <div className="metric-icon">⏱</div>
              </div>

              <div className="metric-card">
                <div className="metric-info">
                  <span className="metric-label">Path Length</span>
                  <span className="metric-val">
                    {planningMeta ? `${planningMeta.path_length_m.toFixed(2)} m` : '--'}
                  </span>
                </div>
                <div className="metric-icon">📏</div>
              </div>

              <div className="metric-card">
                <div className="metric-info">
                  <span className="metric-label">Nodes Expanded</span>
                  <span className="metric-val">
                    {planningMeta ? planningMeta.expanded_count : '--'}
                  </span>
                </div>
                <div className="metric-icon">🔍</div>
              </div>
            </div>

            {planningError && (
              <div className="toast-msg error">
                <span>⚠️ {planningError}</span>
              </div>
            )}
          </div>

          {/* Telemetry Card */}
          <div className="glass-card">
            <h2 className="card-title">Robot Telemetry</h2>
            
            <div className="control-group">
              <span className="control-label">
                Current Pose (x, y, θ)
              </span>
              <span className="metric-val" style={{ fontSize: '1rem', fontFamily: 'var(--font-mono)' }}>
                {simRunning 
                  ? `[${currentPose.x.toFixed(2)}, ${currentPose.y.toFixed(2)}, ${currentPose.theta.toFixed(2)}]`
                  : `[${startX.toFixed(2)}, ${startY.toFixed(2)}, ${startTheta.toFixed(2)}]`
                }
              </span>
            </div>

            {/* Gauges */}
            <div className="gauge-container">
              <div className="gauge-header">
                <span>Linear Velocity (v)</span>
                <span className="control-val">{currentPose.v.toFixed(2)} m/s</span>
              </div>
              <div className="gauge-bar-outer">
                <div 
                  className="gauge-bar-inner" 
                  style={{ width: `${(Math.max(0, currentPose.v) / 0.4) * 100}%` }}
                ></div>
              </div>
            </div>

            <div className="gauge-container">
              <div className="gauge-header">
                <span>Angular Velocity (w)</span>
                <span className="control-val">{currentPose.w.toFixed(2)} rad/s</span>
              </div>
              <div className="gauge-bar-outer">
                <div 
                  className="gauge-bar-inner bidirectional" 
                  style={{ 
                    width: `${((currentPose.w + 1.0) / 2.0) * 100}%`,
                  }}
                ></div>
              </div>
            </div>

            <div className="gauge-container">
              <div className="gauge-header">
                <span>Cross-track Error</span>
                <span className="control-val">{(currentPose.err || 0.0).toFixed(2)} m</span>
              </div>
              <div className="gauge-bar-outer">
                <div 
                  className="gauge-bar-inner" 
                  style={{ 
                    width: `${Math.min(100, ((currentPose.err || 0.0) / 1.0) * 100)}%`,
                    backgroundColor: 'var(--warning-color)'
                  }}
                ></div>
              </div>
            </div>
            
            {simRunning && (
              <div className="control-group">
                <span className="control-label">Simulation Progress</span>
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                  <div className="gauge-bar-outer" style={{ flex: 1 }}>
                    <div className="gauge-bar-inner" style={{ width: `${simProgress}%`, backgroundColor: 'var(--success-color)' }}></div>
                  </div>
                  <span className="control-val">{simProgress}%</span>
                </div>
              </div>
            )}
          </div>

          {/* Instructions card */}
          <div className="ros-guide-card">
            <p>💡 <b>Quick Guide:</b></p>
            <p style={{ marginTop: '4px' }}>1. Drag mouse on grid to set <b>start position</b> & orientation.</p>
            <p style={{ marginTop: '2px' }}>2. Click to set <b>goal</b> position.</p>
            <p style={{ marginTop: '2px' }}>3. Adjust sliders and click <b>⚡ Plan Path</b>.</p>
            <p style={{ marginTop: '2px' }}>4. Run <b>▶ Run Simulation</b> to test path following.</p>
          </div>
        </aside>
      </div>
    </div>
  );
}
