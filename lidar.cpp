#include "CYdLidar.h"
#include <string>
#include "napi.h"
#include <thread>
#include <vector>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>


using namespace std;
using namespace ydlidar;


// Buffer delle letture precedenti per il filtro temporale
std::vector<LaserPoint> previousScan;


// Trova il punto più vicino nella scansione precedente
double findClosestRange(double angle, const std::vector<LaserPoint>& previousScan, double maxAngleDiff = 0.01) {
    double closestRange = -1.0;
    double minDiff = maxAngleDiff;

    for (const auto& prevPoint : previousScan) {
        double diff = std::abs(prevPoint.angle - angle);
        if (diff < minDiff) {
            closestRange = prevPoint.range;
            minDiff = diff;
        }
    }

    return closestRange;
}

void filterLidarData(std::vector<double>& lidarDistances, std::vector<double>& lidarAngles) {
    const double MIN_DISTANCE = 0.16; // 🔹 Soglia minima per il LIDAR
    const double MAX_DISTANCE = 10.0; // 🔹 Soglia massima per il LIDAR

    std::vector<double> filteredDistances;
    std::vector<double> filteredAngles;
    for (size_t i = 0; i < lidarDistances.size(); ++i) {
        if (lidarDistances[i] > 0.0 && lidarDistances[i] >= MIN_DISTANCE) { // 🔹 Ignora i valori minori di 0.16 e quelli pari a zero
            filteredDistances.push_back(std::min(lidarDistances[i], MAX_DISTANCE));
            filteredAngles.push_back(lidarAngles[i]);
        }
    }
    lidarDistances = filteredDistances;
    lidarAngles = filteredAngles;
}



// Funzione per applicare un filtro temporale con matching nearest-neighbor
std::vector<LaserPoint> applyTemporalFilter(const std::vector<LaserPoint>& currentScan, double alpha = 0.6) {
    if (previousScan.empty()) {
        previousScan = currentScan;
        return currentScan;
    }

    std::vector<LaserPoint> filteredScan = currentScan;

    for (size_t i = 0; i < currentScan.size(); ++i) {
        double closestRange = findClosestRange(currentScan[i].angle, previousScan);

        if (closestRange > 0) {
            // Interpolazione solo se troviamo un punto vicino nella scansione precedente
            filteredScan[i].range = alpha * currentScan[i].range + (1 - alpha) * closestRange;
        } else {
            // Se non troviamo un match, manteniamo la misura attuale senza modificarla
            filteredScan[i].range = currentScan[i].range;
        }
    }

    // Aggiorna il buffer con la scansione corrente
    previousScan = filteredScan;

    return filteredScan;
}


// Funzione per applicare un filtro mediano alle distanze LIDAR
std::vector<LaserPoint> applyMedianFilter(const std::vector<LaserPoint>& inputScan, int kernelSize = 3) {
    std::vector<LaserPoint> filteredScan = inputScan;
    int halfKernel = kernelSize / 2;

    // Itera su tutti i punti LIDAR tranne i primi e gli ultimi (per non superare i limiti del vettore)
    for (size_t i = halfKernel; i < inputScan.size() - halfKernel; ++i) {
        double window[5];  // Massima dimensione (kernelSize=5)
        int count = 0;

        // Riempie la finestra di valori
        for (int k = -halfKernel; k <= halfKernel; ++k) {
            window[count++] = inputScan[i + k].range;
        }

        // Ordina l'array localmente senza iteratori STL
        for (int a = 0; a < count - 1; a++) {
            for (int b = a + 1; b < count; b++) {
                if (window[a] > window[b]) {
                    std::swap(window[a], window[b]);
                }
            }
        }

        // Assegna il valore mediano
        filteredScan[i].range = window[halfKernel];
    }

    return filteredScan;
}



// Classe per gestire operazioni asincrone di configurazione
class LidarConfigureWorker : public Napi::AsyncWorker {
public:

    LidarConfigureWorker(CYdLidar* laser, std::string port, int baudrate, Napi::Function& callback)
    : Napi::AsyncWorker(callback), laser(laser), port(port), baudrate(baudrate) {
    }

    ~LidarConfigureWorker() {
    }

    void Execute() override {
        // Configura il LIDAR in un thread separato
        laser->setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
        laser->setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof (int));
    }

    void OnOK() override {
        Napi::Env env = Env();
        Callback().Call({env.Null(), Napi::String::New(env, "LIDAR configured successfully")});
    }

private:
    CYdLidar* laser;
    std::string port;
    int baudrate;
};

// Classe per gestire l'inizializzazione del LIDAR

class LidarInitializeWorker : public Napi::AsyncWorker {
public:

    LidarInitializeWorker(CYdLidar* laser, Napi::Function& callback)
    : Napi::AsyncWorker(callback), laser(laser), success(false) {
    }

    ~LidarInitializeWorker() {
    }

    void Execute() override {
        success = laser->initialize();
        if (success) {
            success = laser->turnOn();
            if (!success) {
                SetError("Failed to turn on the LIDAR");
            }
        } else {
            SetError("Failed to initialize the LIDAR");
        }
    }

    void OnOK() override {
        Napi::Env env = Env();
        Callback().Call({env.Null(), Napi::String::New(env, "LIDAR initialized and turned on successfully")});
    }

    void OnError(const Napi::Error& e) override {
        Napi::Env env = Env();
        Callback().Call({e.Value(), env.Null()});
    }

private:
    CYdLidar* laser;
    bool success;
};

// Classe per gestire la scansione del LIDAR
class LidarScanWorker : public Napi::AsyncWorker {
public:
    LidarScanWorker(CYdLidar* laser, Napi::Function& callback)
    : Napi::AsyncWorker(callback), laser(laser), scanSuccess(false) {
    }

    ~LidarScanWorker() {
    }

    void Execute() override {
        scanSuccess = laser->doProcessSimple(scan);
        if (!scanSuccess) {
            SetError("Failed to process LIDAR data");
            return;
        }
        
        
        
        //  Dobbiamo estrarre le distanze e gli angoli dai dati del LIDAR
        std::vector<double> lidarDistances;
        std::vector<double> lidarAngles;
        for (const auto& point : scan.points) {
            lidarDistances.push_back(point.range);
            lidarAngles.push_back(point.angle);
        }  
        
        // 🔹 Applica il filtro sulle distanze e angoli (0.16 - 10m)
        filterLidarData(lidarDistances, lidarAngles);
        
        // 🔹 Sostituiamo i valori filtrati nei dati del LIDAR
        size_t index = 0;
        std::vector<LaserPoint> filteredPoints;
        for (auto& point : scan.points) {
            if (index < lidarDistances.size()) {
                point.range = lidarDistances[index];
                point.angle = lidarAngles[index];
                filteredPoints.push_back(point);
                index++;
            }
        }
        
        // 🔹 Aggiorniamo la nuvola di punti con solo i valori validi
        scan.points = filteredPoints;


        // Converti i dati del LiDAR in una nuvola di punti PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& point : scan.points) {
            pcl::PointXYZ pclPoint;
            pclPoint.x = point.range * cos(point.angle); // Coordinate X
            pclPoint.y = point.range * sin(point.angle); // Coordinate Y
            pclPoint.z = 0.0; // Assumi Z=0 per un LiDAR 2D
            cloud->push_back(pclPoint);
        }
        
        // 🔹 Ora possiamo elaborare i dati filtrati
        scan.points = applyMedianFilter(scan.points, 5);  // 🔹 Filtro Mediano
        scan.points = applyTemporalFilter(scan.points, 0.7);  // 🔹 Filtro Temporale Corretto
        

        // Applica il filtro PassThrough sull'asse X
        pcl::PointCloud<pcl::PointXYZ>::Ptr passFilteredCloudX(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> passX;
        passX.setInputCloud(cloud);
        passX.setFilterFieldName("x");      // Filtra in base all'asse X
        passX.setFilterLimits(-8.0, 8.0);   // Mantieni i punti con X tra -5 e 5 def -8.0 8.0
        passX.filter(*passFilteredCloudX);

        // Applica il filtro PassThrough sull'asse Y
        /*pcl::PointCloud<pcl::PointXYZ>::Ptr passFilteredCloudY(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> passY;
        passY.setInputCloud(passFilteredCloudX);
        passY.setFilterFieldName("y");      // Filtra in base all'asse Y
        passY.setFilterLimits(-8.0, 8.0);   // Mantieni i punti con Y tra -3 e 3
        passY.filter(*passFilteredCloudY);*/

        // Modifica dinamicamente il filtro di outlier in base alla densità della nuvola
        pcl::PointCloud<pcl::PointXYZ>::Ptr noiseFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
        int numPoints = passFilteredCloudX->size();
        int adaptiveK = std::max(20, std::min(100, numPoints / 50));  // Adatta il numero di vicini
        double adaptiveThresh = std::max(0.3, std::min(1.0, 50.0 / numPoints)); // Adatta la soglia
        
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(passFilteredCloudX);
        sor.setMeanK(adaptiveK);
        sor.setStddevMulThresh(adaptiveThresh);
        sor.filter(*noiseFilteredCloud);

        
        //Rimuovere punti isolati        
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusFilter;
        radiusFilter.setInputCloud(noiseFilteredCloud);
        radiusFilter.setRadiusSearch(0.2);         // Raggio di ricerca (in metri)
        radiusFilter.setMinNeighborsInRadius(3);  // Numero minimo di vicini richiesti
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
        radiusFilter.filter(*filteredCloud);

        // Applica il filtro VoxelGrid per il downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
        voxelGrid.setInputCloud(filteredCloud);
        voxelGrid.setLeafSize(0.05f, 0.05f, 0.05f); // Dimensione del voxel (in metri) def. 0.05
        voxelGrid.filter(*downsampledCloud);

        // Converti la nuvola filtrata in scan.points
        scan.points.clear();
        for (const auto& pclPoint : *downsampledCloud) {
            LaserPoint lidarPoint;
            lidarPoint.angle = atan2(pclPoint.y, pclPoint.x);  // Calcola l'angolo
            lidarPoint.range = sqrt(pclPoint.x * pclPoint.x + pclPoint.y * pclPoint.y); // Calcola la distanza
            scan.points.push_back(lidarPoint);
        }
    }


    void OnOK() override {
        Napi::Env env = Env();
        Napi::Object result = Napi::Object::New(env);
        Napi::Array angles = Napi::Array::New(env, scan.points.size());
        Napi::Array distances = Napi::Array::New(env, scan.points.size());

        for (size_t i = 0; i < scan.points.size(); i++) {
            angles[i] = Napi::Number::New(env, scan.points[i].angle);
            distances[i] = Napi::Number::New(env, scan.points[i].range);
        }

        result.Set("angles", angles);
        result.Set("distances", distances);

        Callback().Call({env.Null(), result});
    }

private:
    CYdLidar* laser;
    LaserScan scan;
    bool scanSuccess;
};

// Classe per gestire lo spegnimento del LIDAR

class LidarShutdownWorker : public Napi::AsyncWorker {
public:

    LidarShutdownWorker(CYdLidar* laser, Napi::Function& callback)
    : Napi::AsyncWorker(callback), laser(laser) {
    }

    ~LidarShutdownWorker() {
    }

    void Execute() override {
        laser->turnOff();
        laser->disconnecting();
    }

    void OnOK() override {
        Napi::Env env = Env();
        Callback().Call({env.Null(), Napi::String::New(env, "LIDAR shut down successfully")});
    }

private:
    CYdLidar* laser;
};

// Funzione per elaborare i comandi e associarli ai worker appropriati

Napi::Value ProcessLidarCommand(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();

    if (info.Length() < 1 || !info[0].IsString()) {
        Napi::TypeError::New(env, "Expected a string argument for the command").ThrowAsJavaScriptException();
        return env.Null();
    }

    std::string command = info[0].As<Napi::String>();
    static CYdLidar laser;

    if (command == "CONFIGURE") {
        if (info.Length() < 4 || !info[1].IsString() || !info[2].IsNumber() || !info[3].IsFunction()) {
            Napi::TypeError::New(env, "Expected serial port, baudrate, and callback").ThrowAsJavaScriptException();
            return env.Null();
        }

        std::string port = info[1].As<Napi::String>();
        int baudrate = info[2].As<Napi::Number>().Int32Value();
        Napi::Function callback = info[3].As<Napi::Function>();

        int optval;
        /// device type
        optval = YDLIDAR_TYPE_SERIAL;
        laser.setlidaropt(LidarPropDeviceType, &optval, sizeof (int));
        /// sample rate
        optval = 5;
        laser.setlidaropt(LidarPropSampleRate, &optval, sizeof (int));
        /// abnormal count
        optval = 4; //def 4
        laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof (int));

        //////////////////////bool property/////////////////
        /// fixed angle resolution
        bool b_optvalue = false;
        laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof (bool));
        /// rotate 180
        b_optvalue = true;
        laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof (bool));
        /// Counterclockwise
        b_optvalue = false;
        laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof (bool));
        b_optvalue = true;
        laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof (bool));
        /// one-way communication
        b_optvalue = false;
        laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof (bool));
        /// intensity
        b_optvalue = false;
        laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof (bool));
        /// Motor DTR
        b_optvalue = true;
        laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof (bool));
        /// HeartBeat
        b_optvalue = false;
        laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof (bool));


        //////////////////////float property/////////////////
        /// unit: Ã‚Â°
        float f_optvalue = 180.0f;
        laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof (float));
        f_optvalue = -180.0f;
        laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof (float));
        /// unit: m
        f_optvalue = 10.0f;
        laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof (float));
        f_optvalue = 0.12f;
        laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof (float));
        /// unit: Hz
        f_optvalue = 8.0f;
        laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof (float));

        LidarConfigureWorker* worker = new LidarConfigureWorker(&laser, port, baudrate, callback);

        worker->Queue();
        return env.Undefined();

    } else if (command == "INITIALIZE") {
        if (info.Length() < 2 || !info[1].IsFunction()) {
            Napi::TypeError::New(env, "Callback function required").ThrowAsJavaScriptException();
            return env.Null();
        }

        Napi::Function callback = info[1].As<Napi::Function>();
        LidarInitializeWorker* worker = new LidarInitializeWorker(&laser, callback);
        worker->Queue();
        return env.Undefined();

    } else if (command == "SCAN") {
        if (info.Length() < 2 || !info[1].IsFunction()) {
            Napi::TypeError::New(env, "Callback function required").ThrowAsJavaScriptException();
            return env.Null();
        }

        Napi::Function callback = info[1].As<Napi::Function>();
        LidarScanWorker* worker = new LidarScanWorker(&laser, callback);
        worker->Queue();
        return env.Undefined();

    } else if (command == "SHUTDOWN") {
        if (info.Length() < 2 || !info[1].IsFunction()) {
            Napi::TypeError::New(env, "Callback function required").ThrowAsJavaScriptException();
            return env.Null();
        }

        Napi::Function callback = info[1].As<Napi::Function>();
        LidarShutdownWorker* worker = new LidarShutdownWorker(&laser, callback);
        worker->Queue();
        return env.Undefined();

    } else {
        Napi::Error::New(env, "Invalid command").ThrowAsJavaScriptException();
        return env.Null();
    }
}

// Inizializza il modulo N-API

Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set("processCommand", Napi::Function::New(env, ProcessLidarCommand));
    return exports;
}

NODE_API_MODULE(lidar, Init)
