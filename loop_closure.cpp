#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <napi.h>

// Struttura per il LoopClosureCandidate

struct LoopClosureCandidate {
    int id1;
    int id2;
    double score;
};

// Struttura per la Pose

struct Pose {
    double x, y, theta;
};

// Dichiarazioni globali
std::vector<Pose> poses; // Lista di pose ottimizzate
std::vector<std::pair<int, int>> edges; // Relazioni tra pose
std::unordered_map<int, Eigen::Vector3d> measurements; // Misurazioni tra le pose


// Definizione della struttura per la funzione di costo

struct PoseGraphError {

    PoseGraphError(const Eigen::Vector3d& relative_measurement)
    : relative_measurement_(relative_measurement) {
    }

    template <typename T>
    bool operator()(const T * const pose1, const T * const pose2, T* residual) const {
        T dx = pose2[0] - pose1[0];
        T dy = pose2[1] - pose1[1];
        T dtheta = pose2[2] - pose1[2];

        residual[0] = dx - T(relative_measurement_(0));
        residual[1] = dy - T(relative_measurement_(1));
        residual[2] = dtheta - T(relative_measurement_(2));

        return true;
    }

private:
    const Eigen::Vector3d relative_measurement_;
};



// Classe per il KD-Tree con PCL

class PoseKDTree {
public:
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    PoseKDTree() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    }

    void buildTree(const std::vector<Pose>& input_poses) {
        cloud->clear();
        std::set<std::pair<double, double>> unique_poses; // Per tracciare le pose uniche

        for (const auto& pose : input_poses) {
            // Controlla se la pose è già stata aggiunta
            if (unique_poses.find({pose.x, pose.y}) == unique_poses.end()) {
            cloud->push_back(pcl::PointXYZ(pose.x, pose.y, 0.0));
            unique_poses.insert({pose.x, pose.y});
        }
        }

        if (cloud->empty()) {
            std::cerr << "Error: Trying to build KD-Tree with an empty or duplicate-only point cloud!" << std::endl;
            return;
        }

        kdtree.setInputCloud(cloud);
    }

    std::vector<int> findNearbyPoses(const Pose& queryPose, double radius) {
        std::vector<int> neighbors;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointXYZ searchPoint(queryPose.x, queryPose.y, 0.0);

        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                neighbors.push_back(pointIdxRadiusSearch[i]);
            }
        }
        return neighbors;
    }
};


// Funzione per calcolare la similarità tra due pose

double calculateSimilarity(const Pose& p1, const Pose& p2, double angleWeight = 1.0) {
    double distance = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    double angleDiff = std::fabs(p1.theta - p2.theta);
    //double angleDiff = std::fabs(std::atan2(std::sin(p1.theta - p2.theta), std::cos(p1.theta - p2.theta)));
    return distance + angleWeight * angleDiff;
}

// Rileva i loop closure usando KD-Tree con PCL

std::vector<LoopClosureCandidate> detectLoopClosuresWithKDTree(const std::vector<Pose>& poses, double radius = 1.0) {
    PoseKDTree poseTree;

    // 🔹 Manteniamo solo le ultime 100 pose per ridurre il carico
    int startIdx = poses.size() > 100 ? poses.size() - 100 : 0;
    std::vector<Pose> recentPoses(poses.begin() + startIdx, poses.end());

    poseTree.buildTree(recentPoses);

    std::vector<LoopClosureCandidate> candidates;
    for (size_t i = 0; i < recentPoses.size(); ++i) {
        std::vector<int> neighbors = poseTree.findNearbyPoses(recentPoses[i], radius);
        for (int neighborIdx : neighbors) {
            if (i < static_cast<size_t> (neighborIdx)) {
                double score = calculateSimilarity(recentPoses[i], recentPoses[neighborIdx]);

                // 🔹 Filtro per evitare falsi positivi
                if (score < 1.0 && std::abs(recentPoses[i].theta - recentPoses[neighborIdx].theta) < 0.2) {
                    candidates.push_back({static_cast<int> (i + startIdx), neighborIdx + startIdx, score});
                }
            }
        }
    }
    return candidates;
}

void addNoiseToPoses(double noiseLevel = 1e-4) {
    for (auto& pose : poses) {
        pose.x += ((rand() % 100) / 100.0 - 0.5) * noiseLevel;
        pose.y += ((rand() % 100) / 100.0 - 0.5) * noiseLevel;
        pose.theta += ((rand() % 100) / 100.0 - 0.5) * noiseLevel;
    }
}

// Aggiungi i vincoli di odometria tra pose consecutive

void addOdometricConstraints() {
    for (size_t i = 1; i < poses.size(); ++i) {
        double dx = poses[i].x - poses[i - 1].x;
        double dy = poses[i].y - poses[i - 1].y;
        double dtheta = poses[i].theta - poses[i - 1].theta;

        measurements[i - 1] = Eigen::Vector3d(dx, dy, dtheta);
        edges.push_back({i - 1, i}); // Aggiungi il vincolo tra le pose consecutive
    }
}

// Funzione di errore per la Pose Graph Optimization

void optimizePoseGraphWithCeres() {
    if (poses.empty()) {
        std::cerr << "Errore: nessuna pose disponibile per l'ottimizzazione." << std::endl;
        return;
    }

    ceres::Problem problem;

    // 🔹 Aggiungiamo un piccolo rumore alle pose per evitare soluzioni banali
    addNoiseToPoses(1e-4);

    // 🔹 Dichiariamo il peso dell'odometria
    double odometry_weight = 0.5;  // Peso dell'odometria (puoi regolarlo)
    double loop_closure_weight = 100.0;  // Peso dei loop closure

    // Aggiungiamo le pose al problema
    for (size_t i = 0; i < poses.size(); ++i) {
        problem.AddParameterBlock(&poses[i].x, 3);
    }

    // Fissiamo la prima pose per stabilizzare la PGO
    problem.SetParameterBlockConstant(&poses[0].x);

    // Aggiungiamo i vincoli tra pose consecutive
    for (size_t i = 1; i < poses.size(); ++i) {
        double dx = poses[i].x - poses[i - 1].x;
        double dy = poses[i].y - poses[i - 1].y;
        double dtheta = poses[i].theta - poses[i - 1].theta;

        Eigen::Vector3d relative_measurement(dx, dy, dtheta);
        
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<PoseGraphError, 3, 3, 3>(
                new PoseGraphError(relative_measurement));

        // 🔹 Aggiungiamo il vincolo di odometria con il peso specificato
        ceres::LossFunction* loss_function = new ceres::ScaledLoss(new ceres::HuberLoss(1.0), odometry_weight, ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(cost_function, loss_function, &poses[i - 1].x, &poses[i].x);
    }

    // Aggiungiamo i loop closure come vincoli
    for (const auto& edge : edges) {
        if (measurements.find(edge.first) == measurements.end()) {
            continue;
        }

        const Eigen::Vector3d& relative_measurement = measurements[edge.first];

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<PoseGraphError, 3, 3, 3>(
                new PoseGraphError(relative_measurement));

        // 🔹 Aggiungiamo il vincolo di loop closure con il peso
        ceres::LossFunction* loss_function = new ceres::ScaledLoss(new ceres::HuberLoss(1.0), loop_closure_weight, ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(cost_function, loss_function, &poses[edge.first].x, &poses[edge.second].x);
    }

    // Configura il solver Ceres
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.num_threads = 2;
    //options.function_tolerance = 1e-20;  // 🔹 Permettiamo al solver di cercare soluzioni più fini
    //options.gradient_tolerance = 1e-20;
    //options.parameter_tolerance = 1e-20;
    options.max_num_iterations = 2000;   // 🔹 Maggiore numero di iterazioni
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout << summary.FullReport() << std::endl;
}


// Funzione da usare nel wrapping con Node.js

Napi::Value DetectLoopClosuresWrapped(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    // Parse input poses from JavaScript
    if (!info[0].IsArray()) {
        Napi::TypeError::New(env, "Input must be an array of poses").ThrowAsJavaScriptException();
        return env.Null();
    }

    for (const auto& pose : poses) {
        if (std::isnan(pose.x) || std::isnan(pose.y) || std::isnan(pose.theta)) {
            std::cerr << "Invalid pose: NaN values detected!" << std::endl;
            return
            {
            }; // Interrompi l'esecuzione se trovi valori invalidi
        }
        if (std::isinf(pose.x) || std::isinf(pose.y) || std::isinf(pose.theta)) {
            std::cerr << "Invalid pose: Inf values detected!" << std::endl;
            return
            {
            }; // Interrompi l'esecuzione se trovi valori infiniti
        }
    }


    auto inputArray = info[0].As<Napi::Array>();
    poses.clear();
    for (size_t i = 0; i < inputArray.Length(); ++i) {
        auto poseObj = inputArray.Get(i).As<Napi::Object>();
        Pose p = {poseObj.Get("x").As<Napi::Number>().DoubleValue(),
            poseObj.Get("y").As<Napi::Number>().DoubleValue(),
            poseObj.Get("theta").As<Napi::Number>().DoubleValue()};
        poses.push_back(p);
    }


    // Usa il nome corretto per la funzione
    auto candidates = detectLoopClosuresWithKDTree(poses);

    // Restituisci i risultati
    Napi::Array resultArray = Napi::Array::New(env, candidates.size());
    for (size_t i = 0; i < candidates.size(); ++i) {
        Napi::Object obj = Napi::Object::New(env);
        obj.Set("id1", candidates[i].id1);
        obj.Set("id2", candidates[i].id2);
        obj.Set("score", candidates[i].score);
        resultArray.Set(i, obj);
    }

    return resultArray;
}

// Funzione di wrapping per JavaScript

Napi::Value OptimizePoseGraphWrapped(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    try {
        // Aggiungi vincoli di odometria
        addOdometricConstraints();

        // Esegui l'ottimizzazione
        optimizePoseGraphWithCeres();

        // Restituisci le pose dopo l'ottimizzazione
        Napi::Object result = Napi::Object::New(env);
        Napi::Array afterOptimization = Napi::Array::New(env, poses.size());

        for (size_t i = 0; i < poses.size(); ++i) {
            Napi::Object after = Napi::Object::New(env);
            after.Set("x", poses[i].x);
            after.Set("y", poses[i].y);
            after.Set("theta", poses[i].theta);
            afterOptimization.Set(i, after);
        }

        result.Set("afterOptimization", afterOptimization);
        return result;
    } catch (const std::exception& e) {
        Napi::Error::New(env, e.what()).ThrowAsJavaScriptException();
        return env.Null();
    }
}


Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set("detectLoopClosures", Napi::Function::New(env, DetectLoopClosuresWrapped));
    exports.Set("optimizePoseGraph", Napi::Function::New(env, OptimizePoseGraphWrapped));
    return exports;
}

NODE_API_MODULE(loop_closure, Init)
