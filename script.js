// Initialize variables
const GRID_ROWS = 10;
const GRID_COLS = 15;
let currentMode = 'obstacle';
let startCell = null;
let endCell = null;
let pickupPoints = [];
let obstacles = [];
let path = [];
let optimizedPath = [];

// DOM elements
const grid = document.getElementById('warehouse-grid');
const robot = document.getElementById('robot');
const findRouteBtn = document.getElementById('find-route');
const simulateBtn = document.getElementById('simulate');
const resetBtn = document.getElementById('reset');
const modeButtons = document.querySelectorAll('.mode-btn');
const timeConstraintInput = document.getElementById('time-constraint');
const robotSpeedInput = document.getElementById('robot-speed');
const totalDistanceSpan = document.getElementById('total-distance');
const estimatedTimeSpan = document.getElementById('estimated-time');
const constraintMetSpan = document.getElementById('constraint-met');

// Hide robot initially
robot.style.display = 'none';

// Create the grid
function createGrid() {
    grid.innerHTML = '';
    
    // Create cells
    for (let row = 0; row < GRID_ROWS; row++) {
        for (let col = 0; col < GRID_COLS; col++) {
            const cell = document.createElement('div');
            cell.className = 'cell';
            cell.dataset.row = row;
            cell.dataset.col = col;
            
            cell.addEventListener('click', () => handleCellClick(cell, row, col));
            
            grid.appendChild(cell);
        }
    }
    
    // Set default start position (robot charging station)
    const defaultStartCell = getCellAt(0, 0);
    setStartCell(defaultStartCell, 0, 0);
}

// Handle cell clicks based on current mode
function handleCellClick(cell, row, col) {
    // Check if cell is already the start or end point
    if ((startCell && startCell.dataset.row == row && startCell.dataset.col == col) ||
        (endCell && endCell.dataset.row == row && endCell.dataset.col == col)) {
        return;
    }
    
    // Handle different modes
    switch (currentMode) {
        case 'obstacle':
            toggleObstacle(cell, row, col);
            break;
            
        case 'pickup':
            togglePickupPoint(cell, row, col);
            break;
            
        case 'delivery':
            setEndCell(cell, row, col);
            break;
    }
}

// Toggle obstacle on cell
function toggleObstacle(cell, row, col) {
    // Check if cell is a pickup point
    const isPickup = pickupPoints.some(p => p.row === row && p.col === col);
    if (isPickup) return;
    
    const isObstacle = cell.classList.contains('obstacle');
    
    if (isObstacle) {
        cell.classList.remove('obstacle');
        obstacles = obstacles.filter(o => !(o.row === row && o.col === col));
    } else {
        cell.classList.add('obstacle');
        obstacles.push({ row, col });
    }
}

// Toggle pickup point on cell
function togglePickupPoint(cell, row, col) {
    // Check if cell is already an obstacle
    if (cell.classList.contains('obstacle')) return;
    
    const isPickup = pickupPoints.some(p => p.row === row && p.col === col);
    
    if (isPickup) {
        cell.classList.remove('pickup');
        const timeCritical = cell.querySelector('.time-critical');
        if (timeCritical) cell.removeChild(timeCritical);
        pickupPoints = pickupPoints.filter(p => !(p.row === row && p.col === col));
    } else {
        cell.classList.add('pickup');
        
        // Add time-critical indicator
        const timeCritical = document.createElement('div');
        timeCritical.className = 'time-critical';
        cell.appendChild(timeCritical);
        
        pickupPoints.push({ row, col, timeCritical: true });
    }
}

// Set starting cell (robot position)
function setStartCell(cell, row, col) {
    // Remove previous start cell styling
    if (startCell) {
        startCell.classList.remove('start');
    }
    
    // Set new start cell
    cell.classList.add('start');
    startCell = cell;
    
    // Position robot at start cell
    const cellRect = cell.getBoundingClientRect();
    const gridRect = grid.getBoundingClientRect();
    
    robot.style.left = `${cell.offsetLeft + 5}px`;
    robot.style.top = `${cell.offsetTop + 5}px`;
}

// Set delivery/end point
function setEndCell(cell, row, col) {
    // Remove previous end styling
    if (endCell) {
        endCell.classList.remove('end');
    }
    
    // Set new end cell
    cell.classList.add('end');
    endCell = cell;
}

// Get cell element at specific row,col
function getCellAt(row, col) {
    return document.querySelector(`.cell[data-row="${row}"][data-col="${col}"]`);
}

// Change the current editing mode
function changeMode(mode) {
    currentMode = mode;
    
    // Update UI
    modeButtons.forEach(btn => {
        if (btn.dataset.mode === mode) {
            btn.classList.add('active');
        } else {
            btn.classList.remove('active');
        }
    });
}

// Event listeners for mode buttons
modeButtons.forEach(btn => {
    btn.addEventListener('click', () => {
        changeMode(btn.dataset.mode);
    });
});

// Reset the grid
function resetGrid() {
    // Clear variables
    pickupPoints = [];
    obstacles = [];
    path = [];
    optimizedPath = [];
    
    // Reset UI
    document.querySelectorAll('.cell').forEach(cell => {
        cell.classList.remove('obstacle', 'pickup', 'end', 'path');
        const timeCritical = cell.querySelector('.time-critical');
        if (timeCritical) cell.removeChild(timeCritical);
    });
    
    // Reset stats
    updateStats(0, 0);
    
    // Reset start position
    const defaultStartCell = getCellAt(0, 0);
    setStartCell(defaultStartCell, 0, 0);
    endCell = null;
    
    // Hide robot and disable simulate button
    robot.style.display = 'none';
    simulateBtn.disabled = true;
}

// Event listener for reset button
resetBtn.addEventListener('click', resetGrid);

// A* Algorithm for pathfinding
function findPath(start, end, obstacles) {
    const openSet = [start];
    const closedSet = [];
    const cameFrom = {};
    
    // g score is distance from start to current node
    const gScore = {};
    gScore[`${start.row},${start.col}`] = 0;
    
    // f score is estimated distance from start to end through current node
    const fScore = {};
    fScore[`${start.row},${start.col}`] = heuristic(start, end);
    
    while (openSet.length > 0) {
        // Find node with lowest fScore in openSet
        let current = openSet[0];
        let lowestFScoreIndex = 0;
        
        for (let i = 1; i < openSet.length; i++) {
            const node = openSet[i];
            const nodeKey = `${node.row},${node.col}`;
            const currentKey = `${current.row},${current.col}`;
            
            if (fScore[nodeKey] < fScore[currentKey]) {
                current = node;
                lowestFScoreIndex = i;
            }
        }
        
        // Check if we've reached the goal
        if (current.row === end.row && current.col === end.col) {
            // Reconstruct the path
            const path = [];
            let temp = current;
            
            while (cameFrom[`${temp.row},${temp.col}`]) {
                path.push(temp);
                temp = cameFrom[`${temp.row},${temp.col}`];
            }
            
            path.push(start);
            return path.reverse();
        }
        
        // Remove current from openSet and add to closedSet
        openSet.splice(lowestFScoreIndex, 1);
        closedSet.push(current);
        
        // Check all neighbors
        const neighbors = getNeighbors(current, obstacles);
        
        for (const neighbor of neighbors) {
            const neighborKey = `${neighbor.row},${neighbor.col}`;
            
            // Skip if in closedSet
            if (closedSet.some(node => node.row === neighbor.row && node.col === neighbor.col)) {
                continue;
            }
            
            // Calculate tentative gScore
            const currentKey = `${current.row},${current.col}`;
            const tentativeGScore = gScore[currentKey] + 1; // Assuming uniform cost of 1
            
            // Add to openSet if not there
            if (!openSet.some(node => node.row === neighbor.row && node.col === neighbor.col)) {
                openSet.push(neighbor);
            } else if (tentativeGScore >= (gScore[neighborKey] || Infinity)) {
                // This is not a better path
                continue;
            }
            
            // This path is the best so far
            cameFrom[neighborKey] = current;
            gScore[neighborKey] = tentativeGScore;
            fScore[neighborKey] = gScore[neighborKey] + heuristic(neighbor, end);
        }
    }
    
    // No path found
    return [];
}

// Get valid neighbors for A* algorithm
function getNeighbors(node, obstacles) {
    const { row, col } = node;
    const neighbors = [];
    const directions = [
        { row: -1, col: 0 }, // Up
        { row: 1, col: 0 },  // Down
        { row: 0, col: -1 }, // Left
        { row: 0, col: 1 }   // Right
    ];
    
    for (const dir of directions) {
        const newRow = row + dir.row;
        const newCol = col + dir.col;
        
        // Check if within grid bounds
        if (newRow < 0 || newRow >= GRID_ROWS || newCol < 0 || newCol >= GRID_COLS) {
            continue;
        }
        
        // Check if obstacle
        if (obstacles.some(o => o.row === newRow && o.col === newCol)) {
            continue;
        }
        
        neighbors.push({ row: newRow, col: newCol });
    }
    
    return neighbors;
}

// Heuristic function for A* (Manhattan distance)
function heuristic(node, goal) {
    return Math.abs(node.row - goal.row) + Math.abs(node.col - goal.col);
}

// Nearest neighbor algorithm for TSP-like route optimization
function optimizeRoute(startPoint, endPoint, pickupPoints) {
    // If no pickup points, just go from start to end
    if (pickupPoints.length === 0) {
        return findPath(startPoint, endPoint, obstacles);
    }
    
    // Start with all pickup points unvisited
    const unvisited = [...pickupPoints];
    let currentPoint = startPoint;
    let fullPath = [];
    
    // Visit each pickup point in nearest-neighbor order
    while (unvisited.length > 0) {
        let nearestDist = Infinity;
        let nearestPoint = null;
        let nearestIndex = -1;
        
        // Find nearest unvisited pickup point
        for (let i = 0; i < unvisited.length; i++) {
            const point = unvisited[i];
            const dist = heuristic(currentPoint, point);
            
            if (dist < nearestDist) {
                nearestDist = dist;
                nearestPoint = point;
                nearestIndex = i;
            }
        }
        
        // Find path to nearest point
        const pathSegment = findPath(currentPoint, nearestPoint, obstacles);
        
        // Remove the duplicate point (current point appears at the end of previous segment
        // and beginning of new segment)
        if (fullPath.length > 0) {
            fullPath.pop();
        }
        
        fullPath = [...fullPath, ...pathSegment];
        
        // Update current point and remove from unvisited
        currentPoint = nearestPoint;
        unvisited.splice(nearestIndex, 1);
    }
    
    // Add path from last pickup point to end point
    const finalSegment = findPath(currentPoint, endPoint, obstacles);
    if (fullPath.length > 0) {
        fullPath.pop(); // Remove duplicate
    }
    fullPath = [...fullPath, ...finalSegment];
    
    return fullPath;
}

// Visualize the path on the grid
function visualizePath(path) {
    // Clear previous path
    document.querySelectorAll('.cell.path').forEach(cell => {
        cell.classList.remove('path');
    });
    
    // Skip start and end cells in visualization
    const pathCells = path.slice(1, path.length - 1);
    
    // Add path class to cells
    pathCells.forEach(point => {
        const cell = getCellAt(point.row, point.col);
        if (cell && !cell.classList.contains('pickup') && 
            !cell.classList.contains('start') && 
            !cell.classList.contains('end')) {
            cell.classList.add('path');
        }
    });
}

// Update statistics display
function updateStats(distance, time) {
    totalDistanceSpan.textContent = distance;
    estimatedTimeSpan.textContent = time.toFixed(1);
    
    const timeConstraint = parseFloat(timeConstraintInput.value);
    if (time <= timeConstraint) {
        constraintMetSpan.textContent = "Yes";
        constraintMetSpan.style.color = "green";
    } else {
        constraintMetSpan.textContent = "No";
        constraintMetSpan.style.color = "red";
    }
}

// Find and visualize the optimal route
function findOptimalRoute() {
    // Check if we have start and end points
    if (!startCell || !endCell) {
        alert("Please set both start and delivery points");
        return;
    }
    
    // Get start and end coordinates
    const start = {
        row: parseInt(startCell.dataset.row),
        col: parseInt(startCell.dataset.col)
    };
    
    const end = {
        row: parseInt(endCell.dataset.row),
        col: parseInt(endCell.dataset.col)
    };
    
    // Optimize route through all pickup points
    optimizedPath = optimizeRoute(start, end, pickupPoints);
    
    // Check if path was found
    if (optimizedPath.length === 0) {
        alert("No valid path found! Please check obstacles.");
        return;
    }
    
    // Visualize the path
    visualizePath(optimizedPath);
    
    // Calculate statistics
    const distance = optimizedPath.length - 1; // -1 because path includes start point
    const robotSpeed = parseFloat(robotSpeedInput.value);
    const time = distance / robotSpeed;
    
    // Update statistics display
    updateStats(distance, time);
    
    // Enable simulation button
    simulateBtn.disabled = false;
}

// Simulate robot movement along the path
function simulateRobot() {
    if (optimizedPath.length === 0) return;
    
    // Show robot
    robot.style.display = 'flex';
    
    // Disable buttons during simulation
    simulateBtn.disabled = true;
    findRouteBtn.disabled = true;
    resetBtn.disabled = true;
    
    let currentIndex = 0;
    
    function moveToNextPoint() {
        if (currentIndex >= optimizedPath.length) {
            // End of simulation
            simulateBtn.disabled = false;
            findRouteBtn.disabled = false;
            resetBtn.disabled = false;
            return;
        }
        
        const point = optimizedPath[currentIndex];
        const cell = getCellAt(point.row, point.col);
        
        // Move robot to new position
        robot.style.left = `${cell.offsetLeft + 5}px`;
        robot.style.top = `${cell.offsetTop + 5}px`;
        
        currentIndex++;
        
        // Continue movement with delay
        const robotSpeed = parseFloat(robotSpeedInput.value);
        const moveDelay = 1000 / robotSpeed; // Convert cells/second to milliseconds
        
        setTimeout(moveToNextPoint, moveDelay);
    }
    
    moveToNextPoint();
}

// Event listeners
findRouteBtn.addEventListener('click', findOptimalRoute);
simulateBtn.addEventListener('click', simulateRobot);

// Initialize grid on page load
createGrid();
