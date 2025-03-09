#include <napi.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#include <iostream>

// Funzione per configurare i parametri GICP
void configureICP(pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp, const Napi::Object& options) {
    if (options.Has("maxIterations")) {
        int maxIterations = options.Get("maxIterations").As<Napi::Number>().Int32Value();
        icp.setMaximumIterations(maxIterations);
    }
    if (options.Has("transformationEpsilon")) {
        double epsilon = options.Get("transformationEpsilon").As<Napi::Number>().DoubleValue();
        icp.setTransformationEpsilon(epsilon);
    }
    if (options.Has("euclideanFitnessEpsilon")) {
        double fitnessEpsilon = options.Get("euclideanFitnessEpsilon").As<Napi::Number>().DoubleValue();
        icp.setEuclideanFitnessEpsilon(fitnessEpsilon);
    }
    if (options.Has("maxCorrespondenceDistance")) {
        double maxDistance = options.Get("maxCorrespondenceDistance").As<Napi::Number>().DoubleValue();
        icp.setMaxCorrespondenceDistance(maxDistance);
    }
}

// Funzione per filtrare punti non validi
/*pcl::PointCloud<pcl::PointXYZ>::Ptr filterValidPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr validCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : inputCloud->points) {
        if (pcl::isFinite(point)) { // Controlla se il punto è valido
            validCloud->points.push_back(point);
        }
    }
    validCloud->width = validCloud->points.size();
    validCloud->height = 1; // Nuvola di punti 1D
    validCloud->is_dense = true;
    return validCloud;
}*/

// Funzione per convertire un array JavaScript in una PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(const Napi::Array& jsArray) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.reserve(jsArray.Length()); // Pre-alloca memoria per evitare riallocazioni
    for (size_t i = 0; i < jsArray.Length(); ++i) {
        auto point = jsArray.Get(i).As<Napi::Object>();
        cloud->points.emplace_back(
            point.Get("x").As<Napi::Number>().DoubleValue(),
            point.Get("y").As<Napi::Number>().DoubleValue(),
            point.Get("z").As<Napi::Number>().DoubleValue()
        );
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

Napi::Value PerformGICP(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    // Validazione degli input
    if (!info[0].IsArray() || !info[1].IsArray() || !info[2].IsObject()) {
        Napi::TypeError::New(env, "Expected two arrays of points and an options object").ThrowAsJavaScriptException();
        return env.Null();
    }

    auto source = info[0].As<Napi::Array>();
    auto target = info[1].As<Napi::Array>();
    auto options = info[2].As<Napi::Object>();

    // Converte i punti sorgente e target in pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source = convertToPointCloud(source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target = convertToPointCloud(target);

    // Controlla che ci siano abbastanza punti in entrambe le nuvole
    const int min_points_required = 20; // Numero minimo di punti richiesti per GICP
    if (cloud_source->size() < min_points_required || cloud_target->size() < min_points_required) {
        std::string error_message = "Cloud size too small. Source size: " + 
            std::to_string(cloud_source->size()) + ", Target size: " + 
            std::to_string(cloud_target->size()) + ". Minimum required: " + 
            std::to_string(min_points_required);
        Napi::Error::New(env, error_message).ThrowAsJavaScriptException();
        return env.Null();
    }

    // Configura GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(options.Get("maxCorrespondenceDistance").As<Napi::Number>().DoubleValue());
    gicp.setMaximumIterations(options.Get("maxIterations").As<Napi::Number>().Int32Value());
    gicp.setTransformationEpsilon(options.Get("transformationEpsilon").As<Napi::Number>().DoubleValue());
    gicp.setEuclideanFitnessEpsilon(options.Get("euclideanFitnessEpsilon").As<Napi::Number>().DoubleValue());

    pcl::PointCloud<pcl::PointXYZ> Final;
    gicp.setInputSource(cloud_source);
    gicp.setInputTarget(cloud_target);

    // Stima iniziale (opzionale)
    Eigen::Matrix4f initialGuess = Eigen::Matrix4f::Identity();
    if (options.Has("initialGuess")) {
        auto guess = options.Get("initialGuess").As<Napi::Array>();
        for (int i = 0; i < 16; ++i) {
            initialGuess(i / 4, i % 4) = guess.Get(i).As<Napi::Number>().DoubleValue();
        }
    }

    gicp.align(Final, initialGuess);

    // Controllo della convergenza
    if (!gicp.hasConverged()) {
        Napi::Error::New(env, "GICP non ha converso!").ThrowAsJavaScriptException();
        return env.Null();
    }

    // Ottieni la matrice di trasformazione e il punteggio di fitness
    Eigen::Matrix4f transformation = gicp.getFinalTransformation();
    double fitnessScore = gicp.getFitnessScore();

    // Costruisci il risultato da restituire
    Napi::Object result = Napi::Object::New(env);
    Napi::Array transformationArray = Napi::Array::New(env, 16);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transformationArray[static_cast<uint32_t>(i * 4 + j)] = Napi::Number::New(env, transformation(i, j));
        }
    }

    result.Set("transformation", transformationArray);
    result.Set("fitnessScore", Napi::Number::New(env, fitnessScore));

    return result;
}



// Inizializza il modulo Node.js
Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set("performGICP", Napi::Function::New(env, PerformGICP));
    return exports;
}

NODE_API_MODULE(gicp_module, Init)
