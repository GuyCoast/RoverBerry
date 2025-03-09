/* global SSB */
/* global DSNAV */
/* global IMU */
/* global rbfunc */


const { createCanvas } = require('canvas');

const gicp = require('./slam_addon/GICP/build/Release/GICP'); // Importa il modulo ICP
const loopClosure = require('./slam_addon/LC/build/Release/LoopClosure');





let scale = 15; //def 15
let slam_scale = 2;
const PI = Math.PI;
let start_angle = 0;
let rel_angle = 0;
let lastUpdateTime;

let starttime = 0;
let starttemp = 0;
let fps = 0;

let mode = 0;



exports.start = function() {
    start_angle = IMU.yaw;

    Z_flag = 0;
    W_flag = 0;
    I_flag = 0;
    R20 = 0;
    L20 = 0;
    R40 = 0;
    L40 = 0;
    MC_flag = 1;

    rbfunc.SetMotorParam(global.MotorAccExp, global.MotorDecExp);

    lastUpdateTime = Date.now();
};


exports.stop = function() {
    MC_flag = 0;
    rbfunc.SetMotorParam(global.MotorAcc, global.MotorDec);
};

exports.zp = function() {
    if (scale <= 5)
        scale = 5;
    else
        scale -= 5;

    slam_scale -= 0.5;
    if (slam_scale <= 0.5)
        slam_scale = 0.5;

    slam_scale_set(slam_scale);
};

exports.zm = function() {
    scale += 5;
    slam_scale += 0.5;

    slam_scale_set(slam_scale);
};

exports.vw = function() {
    global.config.viewmode++;
    if (global.config.viewmode > 1)
        global.config.viewmode = 0;

    set_viewmode();
};

exports.set_viewmode = function() {
    set_viewmode();
};

function set_viewmode() {

    switch (global.config.viewmode) {
        case 0:
            mode = 0;
            scale = 15;
            break;

        case 1:
            mode = 1;
            break;
    }
}

exports.run = function(idx) {
    starttime = new Date();
    fps = parseFloat(1 / (starttime - starttemp) * 1000).toFixed(1);
    //console.log(fps);
    starttemp = starttime;


    if (mode === 0) {
        let x, y;
        const width = 640;
        const height = 360;
        const testcanvas = createCanvas(width, height);
        const ctx = testcanvas.getContext('2d');

        let half_width;
        let half_height;
        half_width = width / 2;
        half_height = height / 2;

        ctx.fillStyle = 'black';
        ctx.fillRect(0, 0, width, height);


        for (let i = 0; i < idx; i++) {

            x = 1000 * SSB.ydlidar_dist[i] * Math.sin(SSB.ydlidar_angle[i]);
            y = 1000 * SSB.ydlidar_dist[i] * Math.cos(SSB.ydlidar_angle[i]);

            if (1000 * SSB.ydlidar_dist[i] > MIN_DIST) {
                ctx.beginPath();
                ctx.arc(half_width + x / scale, half_height - y / scale, 2, 0, 2 * Math.PI);
                ctx.fillStyle = 'rgb(0, 255, 0)';
                ctx.fill();
            }
        }

        ctx.beginPath();
        ctx.rect(half_width - 100 / scale, half_height - 100 / scale, 100 / scale, 100 / scale);
        ctx.strokeStyle = 'grey';
        ctx.lineWidth = 4;
        ctx.stroke();
        if (SSB.stair_alertR === 1) {
            ctx.strokeStyle = 'rgb(255, 0, 0)';
            ctx.beginPath();
            ctx.moveTo(half_width + 50 / scale, half_height - 400 / scale);
            ctx.lineTo(half_width + 600 / scale, half_height - 400 / scale);
            ctx.stroke();

        }
        if (SSB.stair_alertL === 1) {
            ctx.strokeStyle = 'rgb(255, 0, 0)';
            ctx.beginPath();
            ctx.moveTo(half_width - 50 / scale, half_height - 400 / scale);
            ctx.lineTo(half_width - 600 / scale, half_height - 400 / scale);
            ctx.stroke();
        }
        let space = 0;
        ctx.strokeStyle = 'rgb(100, 100, 100)';
        ctx.lineWidth = 1;
        for (let j = 0; j < scale; j++) {
            ctx.beginPath();
            ctx.moveTo((1000 + space) / scale, 0);
            ctx.lineTo((1000 + space) / scale, height);
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(0, (1000 + space) / scale);
            ctx.lineTo(width, (1000 + space) / scale);
            ctx.stroke();

            space += 1000;
        }

        ctx.fillStyle = 'rgb(200, 0, 0)';
        ctx.font = '12px sans-serif';

        ctx.fillText(`fps: ${fps}`, 6, 350);
        ctx.fillText(`scale 1: ${scale}`, 548, 350);
        ctx.fillText(`x: ${DSNAV.PosXmes.toFixed(4)}`, 100, 350);
        ctx.fillText(`y: ${DSNAV.PosYmes.toFixed(4)}`, 200, 350);
        ctx.fillText(`O: ${DSNAV.ThetaMes.toFixed(4)}`, 300, 350);
        ctx.fillText(`p: ${idx}`, 400, 350);

        const buffer = testcanvas.toBuffer('image/png');
        require('fs').writeFileSync('/root/RoverBerry/temp/out.png', buffer);

    }
    else if (mode === 1) { /*-----  MODE 1  -----*/
        const lidarDistances = [];
        const lidarAngles = [];
        const odometry = [];

        for (let i = 0; i < idx; i++) {
            lidarDistances[i] = SSB.ydlidar_dist[i]; // meters
            lidarAngles[i] = normalizeAngle(SSB.ydlidar_angle[i]); // radiants
        }

        odometry.x = ODO_OFFSET_X + DSNAV.PosXmes / 1000; // meters
        odometry.y = ODO_OFFSET_Y + DSNAV.PosYmes / 1000; // meters
        odometry.theta = normalizeAngle(DSNAV.ThetaMes);

        updateMap(lidarDistances, lidarAngles, odometry);

    }
};


function normalizeAngle(theta) {
    return Math.atan2(Math.sin(theta), Math.cos(theta));
}

// Parametri della mappa iniziali
let GRID_SIZE = 0.1; // Dimensione di ogni cella in metri
let MAP_WIDTH = 200; // Numero di celle in larghezza iniziali
let MAP_HEIGHT = 200; // Numero di celle in altezza iniziali
let DISPLAY_SIZE_X;
let DISPLAY_SIZE_Y;
let CELL_SIZE;
const ODO_OFFSET_X = MAP_WIDTH / 20;
const ODO_OFFSET_Y = MAP_HEIGHT / 20;

// Discrete rays param
const MAX_LIDAR_POINTS = 100; // Numero massimo di punti LIDAR da processare
const RAY_STEP = 0.1; // Passo dei raggi discreti in metri

let options = {
    maxIterations: 1500, // Incremento del numero massimo di iterazioni per una convergenza più precisa def 500
    transformationEpsilon: 1e-9, // Migliora la sensibilità al cambiamento delle trasformazioni
    euclideanFitnessEpsilon: 1e-7, // Riduzione per gestire piccole variazioni
    maxCorrespondenceDistance: 0.1, // Rendi il confronto più rigoroso con un raggio minore
};

const MAX_POSES = 500; // 🔹 Limite massimo di pose nel grafico
let lastPGOTime = 0;
const PGO_INTERVAL_MS = 3000; // 🔹 Tempo minimo tra due ottimizzazioni (def 5 secondi)
const MAX_POSES_TO_CHECK = 100; // 🔹 Controlliamo solo le ultime 100 pose
let lastLoopClosureTime = 0;
const LOOP_CLOSURE_INTERVAL_MS = 3000; // 🔹 Tempo minimo tra due rilevamenti di loop closure (def 5 secondi)

let fitnessHistory = []; // Storico dei fitness score

// Inizializza la mappa con valori neutri (0.5 = incerto)
let map = Array.from({ length: MAP_HEIGHT }, () => Array(MAP_WIDTH).fill(0.5));

// Lista per memorizzare la traiettoria del robot
let robotTrajectory = [];

// Lista per memorizzare le posizioni chiave per il confronto ICP
let keyPositions = [];

// Variabile globale per memorizzare la posizione odometrica precedente
let prevOdometry = { x: 0, y: 0, theta: 0 };

// Dati iniziali delle pose
let poses = [];

// Misure relative tra le pose
let measurements = []; // Sarà popolato dinamicamente

slam_scale_set(slam_scale);

InitialPose();

function slam_scale_set(slam_scale) {
    DISPLAY_SIZE_X = 36 * slam_scale; // Dimensione locale della mappa da visualizzare (in celle)
    DISPLAY_SIZE_Y = 20 * slam_scale;
    CELL_SIZE = 18 / slam_scale; // Dimensione della cella in pixel per la visualizzazione
}

function InitialPose() {
    poses.x = ODO_OFFSET_X + DSNAV.PosXmes / 1000;
    poses.y = ODO_OFFSET_Y + DSNAV.PosYmes / 1000;
    poses.theta = normalizeAngle(DSNAV.ThetaMes);
}


// Funzione per ridimensionare dinamicamente la mappa
function resizeMapIfNeeded(odometry) {
    const buffer = 10; // Celle di margine per ridimensionamento
    const requiredWidth = Math.ceil((odometry.x + buffer * GRID_SIZE) / GRID_SIZE);
    const requiredHeight = Math.ceil((odometry.y + buffer * GRID_SIZE) / GRID_SIZE);

    if (requiredWidth <= MAP_WIDTH && requiredHeight <= MAP_HEIGHT) {
        return; // Nessun ridimensionamento necessario
    }

    const newWidth = Math.max(MAP_WIDTH, requiredWidth);
    const newHeight = Math.max(MAP_HEIGHT, requiredHeight);

    // Inizializza la nuova mappa con valori neutri (0.5)
    let newMap = Array.from({ length: newHeight }, () => Array(newWidth).fill(0.5));

    // Copia i valori della vecchia mappa nella nuova mappa
    for (let y = 0; y < MAP_HEIGHT; y++) {
        for (let x = 0; x < MAP_WIDTH; x++) {
            if (y < newHeight && x < newWidth) {
                newMap[y][x] = map[y][x];
            }
        }
    }

    // Aggiorna i riferimenti alla mappa
    map = newMap;
    MAP_WIDTH = newWidth;
    MAP_HEIGHT = newHeight;
    console.log(`Mappa ridimensionata: ${MAP_WIDTH}x${MAP_HEIGHT}`);
}


function filterFitnessScore(fitnessScore, history, maxThreshold = 0.05, adaptationFactor = 0.01) {
    const averageFitness = history.reduce((sum, score) => sum + score, 0) / Math.max(history.length, 1);

    if (fitnessScore > averageFitness + adaptationFactor) {
        console.warn(`Fitness Score troppo alto (${fitnessScore}). Dati ignorati.`);
        return false;
    }

    if (history.length >= 10) history.shift(); // Mantieni solo gli ultimi 10 valori
    history.push(fitnessScore);

    return true;
}

function adjustGICPParameters(options, fitnessScore, hasConverged) {
    const adjustmentFactor = 0.1;
    const maxFitnessThreshold = 0.05;

    if (!hasConverged) {
        options.transformationEpsilon *= 10;
        options.maxCorrespondenceDistance *= 2;
        console.warn('GICP: Incrementati parametri per migliorare convergenza.');
    }
    else if (fitnessScore > maxFitnessThreshold) {
        options.maxCorrespondenceDistance *= 1 + adjustmentFactor;
        options.transformationEpsilon *= 1.5;
        console.log('GICP: Incrementati parametri per gestire alta complessità.');
    }
    else {
        options.maxCorrespondenceDistance *= 1 - adjustmentFactor;
        options.transformationEpsilon /= 1.5;
        console.log('GICP: Ridotti parametri per maggiore precisione.');
    }

    options.maxCorrespondenceDistance = Math.min(options.maxCorrespondenceDistance, 1.0);
    options.transformationEpsilon = Math.max(options.transformationEpsilon, 1e-9);

    return options;
}


function updateWithGICP(lidarDistances, lidarAngles, odometry) {
    if (keyPositions.length === 0) {
        keyPositions.push({
            x: odometry.x,
            y: odometry.y,
            theta: odometry.theta,
            points: generateLidarPoints(lidarDistances, lidarAngles, odometry),
        });
        return;
    }

    const currentPoints = generateLidarPoints(lidarDistances, lidarAngles, odometry);
    const lastKey = keyPositions[keyPositions.length - 1];
    const previousPoints = lastKey.points;

    let initialGuess = createInitialGuess(prevOdometry, odometry);

    try {
        const result = gicp.performGICP(currentPoints, previousPoints, { ...options, initialGuess });

        if (!result || !result.transformation) {
            console.error('GICP: Trasformazione non valida.');
            return;
        }

        if (!filterFitnessScore(result.fitnessScore, fitnessHistory)) {
            return; // Ignora risultati con fitness score troppo alto
        }

        const deltaX = result.transformation[3];
        const deltaY = result.transformation[7];
        const deltaTheta = Math.atan2(result.transformation[4], result.transformation[0]);

        odometry.x += deltaX;
        odometry.y += deltaY;
        odometry.theta = normalizeAngle(odometry.theta + deltaTheta);

        const distance = Math.sqrt(
            Math.pow(odometry.x - lastKey.x, 2) + Math.pow(odometry.y - lastKey.y, 2)
        );
        if (distance > 1.0 || Math.abs(deltaTheta) > 0.1) {
            keyPositions.push({
                x: odometry.x,
                y: odometry.y,
                theta: odometry.theta,
                points: currentPoints,
            });
        }
    }
    catch (error) {
        console.error('Errore durante l’esecuzione di GICP:', error.message);
        options = adjustGICPParameters(options, 1.0, false);
    }
}

function createInitialGuess(prev, current) {
    const deltaX = current.x - prev.x;
    const deltaY = current.y - prev.y;
    const deltaTheta = normalizeAngle(current.theta - prev.theta);

    return [
        Math.cos(deltaTheta), -Math.sin(deltaTheta), 0, deltaX,
        Math.sin(deltaTheta), Math.cos(deltaTheta), 0, deltaY,
        0, 0, 1, 0,
        0, 0, 0, 1
    ];
}

function generateLidarPoints(lidarDistances, lidarAngles, odometry) {
    const points = [];
    const STEP = 1;
    for (let i = 0; i < lidarDistances.length; i += STEP) {
        const distance = lidarDistances[i];
        if (!Number.isFinite(distance) || distance <= 0.16 || distance > 10.0) continue;

        const angle = lidarAngles[i];
        const x = odometry.x + distance * Math.cos(angle + odometry.theta);
        const y = odometry.y + distance * Math.sin(angle + odometry.theta);

        points.push({ x, y, z: 0 });
    }
    return points;
}

// Aggiorna le misurazioni relative basandosi su odometria e LIDAR
function updateMeasurements() {
    if (poses.length < 2) return; // Nessuna misura da aggiornare se c'è solo una pose

    const lastPose = poses[poses.length - 2];
    const currentPose = poses[poses.length - 1];

    const relativeX = currentPose.x - lastPose.x;
    const relativeY = currentPose.y - lastPose.y;
    const relativeTheta = currentPose.theta - lastPose.theta;

    measurements.push({
        id1: poses.length - 2,
        id2: poses.length - 1,
        relative: { x: relativeX, y: relativeY, theta: relativeTheta },
    });
}

// Aggiorna le pose basandosi su odometria
function updatePoses(odometry) {
    const newPose = { x: odometry.x, y: odometry.y, theta: odometry.theta };
    poses.push(newPose);

    if (poses.length > MAX_POSES) {
        poses.shift(); // 🔹 Rimuove la pose più vecchia per evitare sovraccarico
    }

    updateMeasurements();
}

// Rileva i loop closures
function detectLoopClosures() {
    const currentTime = Date.now();
    if (currentTime - lastLoopClosureTime < LOOP_CLOSURE_INTERVAL_MS) {
        //console.log("Loop Closure saltato per evitare carico eccessivo.");
        return [];
    }
    lastLoopClosureTime = currentTime;

    if (poses.length < 2) return [];
    const recentPoses = poses.slice(-MAX_POSES_TO_CHECK);
    return loopClosure.detectLoopClosures(recentPoses);
}

// Funzione per unire le pose ottimizzate nella mappa senza cancellare dati
function mergeOptimizedPosesIntoMap(optimizedPoses) {
    optimizedPoses.forEach((pose, index) => {
        if (!map[index]) {
            // Se la posizione non esiste nella mappa, aggiungila
            map[index] = {
                x: pose.x,
                y: pose.y,
                theta: pose.theta,
            };
        }
        else {
            // Se la posizione esiste, aggiorna solo i valori necessari
            map[index].x = pose.x;
            map[index].y = pose.y;
            map[index].theta = pose.theta;
        }
    });
}

// Funzione per aggiornare la traiettoria del robot
function updateRobotTrajectory(optimizedPoses) {
    robotTrajectory = optimizedPoses.map(pose => ({
        x: pose.x,
        y: pose.y,
        theta: pose.theta,
    }));
}

// Calcola il delta odometry
function calculateDeltaOdometry(currentOdometry) {
    const deltaX = currentOdometry.x - prevOdometry.x;
    const deltaY = currentOdometry.y - prevOdometry.y;
    const deltaTheta = normalizeAngle(currentOdometry.theta - prevOdometry.theta);

    // Aggiorna la posizione precedente per il prossimo ciclo
    prevOdometry = { ...currentOdometry };

    return { x: deltaX, y: deltaY, theta: deltaTheta };
}


/******************************************************************************/
/******************************************************************************/
// Funzione principale per aggiornare la mappa
function updateMap(lidarDistances, lidarAngles, odometry) {
    const deltaOdometry = calculateDeltaOdometry(odometry);

    if (!shouldUpdateMap(deltaOdometry)) {
        return; // Salta l'aggiornamento se i criteri non sono soddisfatti
    }


    //applyProbabilityDecay(0.01); //Applichiamo il decadimento della probabilità prima di aggiornare la mappa

    updateWithGICP(lidarDistances, lidarAngles, odometry);

    updatePoses(odometry); //Aggiorna le pose con nuovi dati

    const candidates = detectLoopClosures(); //Rileva i loop closures
    //console.log(`Loop closures trovati: ${candidates.length}`);

    if (candidates.length > 0) { // Se ci sono candidati validi, ottimizza il grafo
        //console.log("Dettagli loop closure:", candidates);
        /*for (const c of candidates) {
            const diffX = Math.abs(poses[c.id1].x - poses[c.id2].x);
            const diffY = Math.abs(poses[c.id1].y - poses[c.id2].y);
            const diffTheta = Math.abs(poses[c.id1].theta - poses[c.id2].theta);

            console.log(`Loop closure tra ${c.id1} e ${c.id2} - ΔX: ${diffX}, ΔY: ${diffY}, ΔTheta: ${diffTheta}`);
        }*/

        const currentTime = Date.now();
        if (currentTime - lastPGOTime < PGO_INTERVAL_MS) {
            //console.log("PGO saltata per evitare carico eccessivo.");
            return; // 🔹 Saltiamo l'ottimizzazione se troppo recente
        }
        lastPGOTime = currentTime;

        // 🔹 Eseguiamo la PGO in un thread separato
        setTimeout(() => {
            try {
                const result = loopClosure.optimizePoseGraph();
                mergeOptimizedPosesIntoMap(result.afterOptimization);
                updateRobotTrajectory(result.afterOptimization);
                console.log("PGO completata.");
            }
            catch (error) {
                console.error("Errore nell'ottimizzazione:", error);
            }
        }, 0);
    }
    /*else {
        console.log("Nessun candidato trovato. Mappa e traiettoria non modificate.");
    }*/

    resizeMapIfNeeded(odometry);
    updateMapWithDiscreteRays(lidarDistances, lidarAngles, odometry);
    displayLocalMap(map, odometry, robotTrajectory, lidarDistances);
}

/******************************************************************************/
/******************************************************************************/


function updateProbability(prior, likelihood) {
    // Calcolo probabilistico avanzato con Bayes
    return (likelihood * prior) / ((likelihood * prior) + ((1 - likelihood) * (1 - prior)));
}


const UPDATE_INTERVAL_MS = 500; // Intervallo di tempo minimo in millisecondi
const MIN_DISTANCE_UPDATE = 0.5; // Minima distanza in metri per aggiornare
const MIN_ANGLE_UPDATE = 0.1; // Minimo angolo in radianti per aggiornare

function shouldUpdateMap(deltaOdometry) {
    const currentTime = Date.now();
    const timeElapsed = currentTime - lastUpdateTime;

    const distanceMoved = Math.sqrt(deltaOdometry.x * deltaOdometry.x + deltaOdometry.y * deltaOdometry.y);
    const angleMoved = Math.abs(deltaOdometry.theta);

    if (timeElapsed >= UPDATE_INTERVAL_MS ||
        distanceMoved >= MIN_DISTANCE_UPDATE ||
        angleMoved >= MIN_ANGLE_UPDATE) {
        lastUpdateTime = currentTime; // Aggiorna il tempo
        return true;
    }
    return false;
}

function applyProbabilityDecay(decayRate = 0.01, threshold = 0.1) {
    for (let y = 0; y < MAP_HEIGHT; y++) {
        for (let x = 0; x < MAP_WIDTH; x++) {
            if (map[y][x] > 0.7) {
                // Se la cella è molto occupata, riduciamo più lentamente
                map[y][x] = Math.max(0.7, map[y][x] - decayRate / 2);
            }
            else if (map[y][x] > 0.5 + threshold) {
                // Decadimento normale per celle leggermente occupate
                map[y][x] = Math.max(0.5 + threshold, map[y][x] - decayRate);
            }
            else if (map[y][x] < 0.3) {
                // Se la cella è molto libera, aumentiamo più lentamente
                map[y][x] = Math.min(0.3, map[y][x] + decayRate / 2);
            }
            else if (map[y][x] < 0.5 - threshold) {
                // Aumento normale per celle leggermente libere
                map[y][x] = Math.min(0.5 - threshold, map[y][x] + decayRate);
            }
        }
    }
}



// Funzione per aggiornare la mappa con raggi discreti
function updateMapWithDiscreteRays(lidarDistances, lidarAngles, odometry) {
    const robotX = odometry.x;
    const robotY = odometry.y;

    for (let i = 0; i < Math.min(lidarDistances.length, MAX_LIDAR_POINTS); i++) {
        const distance = lidarDistances[i];
        const angle = lidarAngles[i];

        if (distance <= 0.16 || distance > 10.0) continue;

        const endX = robotX + distance * Math.cos(odometry.theta + angle);
        const endY = robotY + distance * Math.sin(odometry.theta + angle);

        const steps = Math.ceil(distance / RAY_STEP);
        const stepX = (endX - robotX) / steps;
        const stepY = (endY - robotY) / steps;

        for (let j = 0; j < steps; j++) {
            const currentX = robotX + stepX * j;
            const currentY = robotY + stepY * j;

            const cellX = Math.floor(currentX / GRID_SIZE);
            const cellY = Math.floor(currentY / GRID_SIZE);

            if (cellX >= 0 && cellX < MAP_WIDTH && cellY >= 0 && cellY < MAP_HEIGHT) {
                const prior = map[cellY][cellX];
                const newProbability = updateProbability(prior, 0.3); // Aggiorna con probabilità di attraversamento
                map[cellY][cellX] = Math.max(0, newProbability);
            }
        }

        const targetX = Math.floor(endX / GRID_SIZE);
        const targetY = Math.floor(endY / GRID_SIZE);

        if (targetX >= 0 && targetX < MAP_WIDTH && targetY >= 0 && targetY < MAP_HEIGHT) {
            const prior = map[targetY][targetX];
            const likelihood = 0.7;
            const newProbability = updateProbability(prior, likelihood); // Aggiorna con probabilità di occupazione
            map[targetY][targetX] = Math.min(1, newProbability);
        }
    }

    robotTrajectory.push({ x: odometry.x, y: odometry.y });
}

/**
 * Genera un'immagine PNG in Buffer con la finestra locale della mappa.
 * @param {number[][]} map - Matrice 2D con valori da 0..1 (occupancy).
 * @param {{x:number, y:number, theta:number}} odometry - Posizione e angolo del robot (in metri/radiani).
 * @param {{x:number, y:number}[]} robotTrajectory - Array di posizioni passate del robot (in metri).
 * @returns {Buffer} - Il buffer PNG con l'immagine.
 */

// Funzione per visualizzare una finestra locale della mappa
function displayLocalMap(map, odometry, robotTrajectory, lidarDistances) {
    // 1. Calcoliamo il centro (in celle) rispetto a odometry.x,y
    const centerX = Math.floor(odometry.x / GRID_SIZE);
    const centerY = Math.floor(odometry.y / GRID_SIZE);

    // 2. Limiti della finestra locale
    const startX = Math.max(centerX - Math.floor(DISPLAY_SIZE_X / 2), 0);
    const startY = Math.max(centerY - Math.floor(DISPLAY_SIZE_Y / 2), 0);
    const endX = Math.min(centerX + Math.floor(DISPLAY_SIZE_X / 2), MAP_WIDTH);
    const endY = Math.min(centerY + Math.floor(DISPLAY_SIZE_Y / 2), MAP_HEIGHT);

    // 3. Dimensioni (in pixel) della finestra locale
    const widthPx = (endX - startX) * CELL_SIZE;
    const heightPx = (endY - startY) * CELL_SIZE;

    // 4. Creiamo il canvas e il contesto 2D
    const canvas = createCanvas(widthPx, heightPx);
    const ctx = canvas.getContext('2d');

    // Sfondo bianco
    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, widthPx, heightPx);

    // 5. Disegno celle
    for (let y = startY; y < endY; y++) {
        for (let x = startX; x < endX; x++) {
            const value = map[y][x]; // Occupancy [0..1]
            let color;
            if (value > 0.7) color = 'black'; // Ostacolo
            else if (value < 0.3) color = 'white'; // Libero
            else color = 'gray'; // Incerto

            // Coordinate locali
            const localX = x - startX;
            const localY = y - startY;
            const px = localX * CELL_SIZE;
            const py = localY * CELL_SIZE;

            // Riempi cella
            ctx.fillStyle = color;
            ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);
        }
    }

    // 6. Disegno traiettoria robot
    ctx.strokeStyle = 'red';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 1; i < robotTrajectory.length; i++) {
        const prev = robotTrajectory[i - 1];
        const curr = robotTrajectory[i];

        // Converti da coordinate reali a pixel locali
        const prevX = ((prev.x - startX * GRID_SIZE) / GRID_SIZE) * CELL_SIZE;
        const prevY = ((prev.y - startY * GRID_SIZE) / GRID_SIZE) * CELL_SIZE;
        const currX = ((curr.x - startX * GRID_SIZE) / GRID_SIZE) * CELL_SIZE;
        const currY = ((curr.y - startY * GRID_SIZE) / GRID_SIZE) * CELL_SIZE;

        ctx.moveTo(prevX, prevY);
        ctx.lineTo(currX, currY);
    }
    ctx.stroke();

    // 7. Disegno robot come triangolo
    // Lo posizioniamo al centro della finestra locale
    const robotLocalX = (DISPLAY_SIZE_X / 2) * CELL_SIZE;
    const robotLocalY = (DISPLAY_SIZE_Y / 2) * CELL_SIZE;
    const triangleSize = CELL_SIZE; // “raggio” del triangolo
    const angle = odometry.theta;

    const tipX = robotLocalX + triangleSize * Math.cos(angle);
    const tipY = robotLocalY + triangleSize * Math.sin(angle);
    const leftX = robotLocalX + (triangleSize * 0.5) * Math.cos(angle + (2 * Math.PI) / 3);
    const leftY = robotLocalY + (triangleSize * 0.5) * Math.sin(angle + (2 * Math.PI) / 3);
    const rightX = robotLocalX + (triangleSize * 0.5) * Math.cos(angle - (2 * Math.PI) / 3);
    const rightY = robotLocalY + (triangleSize * 0.5) * Math.sin(angle - (2 * Math.PI) / 3);

    ctx.fillStyle = 'red';
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(leftX, leftY);
    ctx.lineTo(rightX, rightY);
    ctx.closePath();
    ctx.fill();

    // 8. Testo informativo su x, y, theta
    ctx.fillStyle = 'rgb(200, 0,0)';
    ctx.font = '14px sans-serif';
    ctx.fillText(`x: ${odometry.x.toFixed(4)}`, 50, 15);
    ctx.fillText(`y: ${odometry.y.toFixed(4)}`, 150, 15);
    ctx.fillText(`O: ${odometry.theta.toFixed(4)}`, 250, 15);
    ctx.fillText(`p: ${lidarDistances.length}`, 350, 15);
    ctx.fillText(`scale: ${slam_scale}`, 450, 15);

    // 9. Ritorniamo il buffer PNG
    const buffer = canvas.toBuffer('image/png');
    require('fs').writeFileSync('/root/RoverBerry/temp/out.png', buffer);
}









const MAX_DIST = 800; //1000
const MIN_DIST = 160; //170
const MIN_DIST_FRONT = 120; //120
const DIST_WARN = 400; //400

let SPEED_FW = 0;
const SPEED_MAX = 300; //300******
const SPEED_MIN = 150; //150******

let bws;
let dir;

const correction = 1; //1*******
const dir400 = 400 * correction;
const dir200 = 200 * correction;
const dir120 = 120 * correction;
const dir90 = 90 * correction;
const dir40 = 40 * correction;
const dir20 = 20 * correction;

let Z_flag = 0;
let W_flag = 0;
let I_flag = 0;
let MC_flag = 0;
let R20 = 0;
let L20 = 0;
let R40 = 0;
let L40 = 0;

const Current_Limit = 1500; //1100  //1500
const Current_Delay = 3; //8 //3
let Current_Count_fw = 0;
let Current_Count_bw = 0;



//******************************************************************************
//*********************** START MAIN NAVIGATION CYCLE **************************
//******************************************************************************

exports.navigation = function(idx) {

    rel_angle = IMU.yaw - start_angle;

    if (rel_angle < 0)
        rel_angle += 360;

    if (I_flag === 0) {
        SPEED_FW = SPEED_MAX;
        //dir = 0;
        let optiDir = searchPath();

        if (optiDir < 1)
            dir = 0;
        else
            dir = SPEED_FW * (180 - optiDir) / 500.0;
    }

    for (let i = 0; i < idx; i++) {

        Lidar_Dist_Warn(i);
        Lidar_Dist_Halt(i);
        Lidar_0(i);
        Lidar_R20(i);
        Lidar_L20(i);
        Lidar_R40(i);
        Lidar_L40(i);


        if (I_flag === 0) {
            if (Z_flag === 1) { // ********** FRONT OBSTACLE ***********
                //console.log('front');

                /*if (R20 === 1 && L20 === 1 && R40 === 1 && L40 === 1 && W_flag === 1) {
                    //console.log('STOP');
                    dir = 0;
                    SPEED_FW = 0;
                    continue;
                }*/
                if (R40 === 0 && L20 === 1 && R20 === 1) {
                    //console.log('turn right_b');
                    dir = dir90; //90**
                    SPEED_FW = SPEED_MIN;
                    if (W_flag === 1) {
                        dir = dir200; //200**
                        SPEED_FW = 0;
                    }
                    continue;
                }
                if (L40 === 0 && L20 === 1 && R20 === 1) {
                    //console.log('turn left_b');
                    dir = -dir90; //90**
                    SPEED_FW = SPEED_MIN;
                    if (W_flag === 1) {
                        dir = -dir200; //-200**
                        SPEED_FW = 0;
                    }
                    continue;
                }
                if (R20 === 0 && L20 === 1) {
                    //console.log('turn right_a');
                    dir = dir90; //90**
                    SPEED_FW = SPEED_MIN;
                    if (W_flag === 1) {
                        //console.log('Warning right_a');
                        dir = dir200; //200**
                        SPEED_FW = 0;
                    }
                    continue;
                }
                if (L20 === 0 && R20 === 1) {
                    //console.log('turn left_a');
                    dir = -dir90; //90**
                    SPEED_FW = SPEED_MIN;
                    if (W_flag === 1) {
                        //console.log('Warning left_a');
                        dir = -dir200; //-200**
                        SPEED_FW = 0;
                    }
                    continue;
                }
            }
            else { //***************************** NO FRONT OBSTACLES ************************************
                W_flag = 0;
                if (L20 === 0 && R20 === 1) {
                    //console.log('turn left');
                    dir = -dir40; //40**
                    SPEED_FW = SPEED_MAX;
                    continue;
                }
                if (R20 === 0 && L20 === 1) {
                    //console.log('turn right');
                    dir = dir40; //40**
                    SPEED_FW = SPEED_MAX;
                    continue;
                }
                if (R20 === 0 && L20 === 0 && R40 === 1) {
                    //console.log('turn soft left');
                    dir = -dir20; //20**
                    SPEED_FW = SPEED_MAX;
                    continue;
                }
                if (R20 === 0 && L20 === 0 && L40 === 1) {
                    //console.log('turn soft right');
                    dir = dir20; //20**
                    SPEED_FW = SPEED_MAX;
                    continue;
                }
            }
        }
    }

    if ((Current_ctrl_fw() === 1 /*|| stasis() === 1*/ || SSB.stair_alertL === 1 || SSB.stair_alertR === 1) && I_flag === 0) {
        I_flag = 1;
        SPEED_FW = -SPEED_MAX;
        dir = 0;
        //console.log('impact or stairs');
    }

    if (I_flag === 2) {
        SPEED_FW = 0;
        I_flag = 3;
        if (R20 === 1 || R40 === 1) //**************
            dir = -dir120; //-200**
        else if (L20 === 1 || L40 === 1) //*************
            dir = dir120; //200**
        else
            dir = dir200; //400**
    }

    //console.log(SPEED_FW, dir, IMU.yaw, rel_angle);
    //console.log(L40, L20, Z_flag, R20, R40);
    //console.log(I_flag);
    /*console.log(DSNAV.PosXmes, DSNAV.PosYmes);
    rbfunc.dsnav_ctrl(300, 300);*/
    //console.log(SSB.stair_alert);
    //console.log(stasis());


    if (MC_flag === 1)
        rbfunc.dsnav_ctrl(SPEED_FW + dir, SPEED_FW - dir);

    Z_flag = 0;
    R20 = 0;
    L20 = 0;
    R40 = 0;
    L40 = 0;

    if (I_flag === 1) {
        bws = setTimeout(function() { I_flag = 2; }, 700); // delay for back off def 700
        if (Current_ctrl_bw() === 1 /*|| stasis() === 1*/ ) {
            console.log('clearTimeout');
            clearTimeout(bws);
            I_flag = 2;
        }
    }

    if (I_flag === 3)
        setTimeout(function() { I_flag = 0; }, 2000); // delay for rotation def 2000
};
//******************************************************************************
//*********************** END MAIN NAVIGATION CYCLE ****************************
//******************************************************************************


const arcAngle = 40; //40
const minGap = 550; //550

function searchPath() {
    let searchAngle;
    let optiAngle;
    let minDist;
    let maxDist = 0;

    for (searchAngle = -90 + arcAngle / 2; searchAngle < 90 - arcAngle / 2; searchAngle++) {
        minDist = 10000;

        if (searchAngle < 0)
            searchAngle += 360;

        for (let i = searchAngle - arcAngle / 2; i < searchAngle + arcAngle / 2; i++) {
            //trova l'ostacolo piÃ¹ vicino in un arco arcAngle attorno a searchAngle
            if (MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < minDist) {
                minDist = MtToMm(SSB.ydlidar_dist[i]);
            }
        }

        if (minDist > maxDist) {
            maxDist = minDist;
            optiAngle = searchAngle;
        }
        if (searchAngle > 180)
            searchAngle -= 360
    }

    //console.log("optiAngle=" + String(optiAngle) + "\tmaxDist=" + String(maxDist));
    if (maxDist > minGap) //se il varco trovato Ã¨ sufficientemente profondo
        return (optiAngle);
    else
        return 0; //non ci sono varchi sufficientemente profondi
}

function RadToDeg(radians) {
    let degrees = radians * (180 / Math.PI);
    if (degrees < 0)
        degrees += 360;
    return degrees;

}

function MtToMm(meters) {
    return meters * 1000;
}


function Current_ctrl_fw() {
    if (global.ovrc_disable === 0) {
        if (DSNAV.currentfrontL > Current_Limit || DSNAV.currentfrontR > Current_Limit || DSNAV.currentrearL > Current_Limit + 400 || DSNAV.currentrearR > Current_Limit + 400) { //500
            Current_Count_fw++;
            if (Current_Count_fw > Current_Delay) {
                Current_Count_fw = 0;
                console.log('obstacle over current forward');
                return 1;
            }
            else
                return 0;
        }
        else {
            Current_Count_fw = 0;
            return 0;
        }
    }
    else
        return 0;
}

function Current_ctrl_bw() {
    if (global.ovrc_disable === 0) {
        if (DSNAV.currentfrontL < -Current_Limit || DSNAV.currentfrontR < -Current_Limit || DSNAV.currentrearL < -Current_Limit || DSNAV.currentrearR < -Current_Limit) {
            Current_Count_bw++;
            if (Current_Count_bw > Current_Delay) {
                Current_Count_bw = 0;
                console.log('obstacle over current backward');
                return 1;
            }
            else
                return 0;
        }
        else {
            Current_Count_bw = 0;
            return 0;
        }
    }
    else
        return 0;
}

function stasis() {
    if (((DSNAV.m_error >> 14) & 1) === 1) {
        console.log('stasis');
        return 1;
    }
    else
        return 0;
}

function Lidar_Dist_Halt(i) {
    if ((RadToDeg(SSB.ydlidar_angle[i]) > 320 || RadToDeg(SSB.ydlidar_angle[i]) < 40) && MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < 240) {
        I_flag = 1;
        SPEED_FW = -SPEED_MAX;
        dir = 0;
    }
}

function Lidar_Dist_Warn(i) {
    if ((RadToDeg(SSB.ydlidar_angle[i]) > 310 || RadToDeg(SSB.ydlidar_angle[i]) < 50) && MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < DIST_WARN)
        W_flag = 1;
}

function Lidar_0(i) {
    if ((RadToDeg(SSB.ydlidar_angle[i]) >= 350 || RadToDeg(SSB.ydlidar_angle[i]) <= 10) && MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < MAX_DIST)
        Z_flag = 1;
}

function Lidar_R20(i) {
    if (RadToDeg(SSB.ydlidar_angle[i]) > 10 && RadToDeg(SSB.ydlidar_angle[i]) <= 30 && MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < MAX_DIST)
        R20 = 1;
}

function Lidar_L20(i) {
    if (RadToDeg(SSB.ydlidar_angle[i]) >= 330 && RadToDeg(SSB.ydlidar_angle[i]) < 350 && MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < MAX_DIST)
        L20 = 1;
}

function Lidar_R40(i) {
    if (RadToDeg(SSB.ydlidar_angle[i]) > 30 && RadToDeg(SSB.ydlidar_angle[i]) <= 50 && MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < MAX_DIST)
        R40 = 1;
}

function Lidar_L40(i) {
    if (RadToDeg(SSB.ydlidar_angle[i]) >= 310 && RadToDeg(SSB.ydlidar_angle[i]) < 330 && MtToMm(SSB.ydlidar_dist[i]) > MIN_DIST_FRONT && MtToMm(SSB.ydlidar_dist[i]) < MAX_DIST)
        L40 = 1;
}
