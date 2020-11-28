#include "../include/common.h"
#include "../include/mapper.h"
#include "../include/sensor_info.h"
#include "../include/plotter.h"
#include "ekfslam.h"
#include<iomanip>

using namespace std;

int main() 
{
    string input_mapper_name = "world.dat";
    string input_measurement_name = "sensor.dat";

    //read map data from world.dat
    Mapper mapper;
    mapper.initialize(input_mapper_name);

    //read measurement data from sensor.dat
    MeasurementPackage measurements;
    measurements.initialize(input_measurement_name);
    cout << measurements.data.size() << endl;

    // initialize draw of class Draw
    Draw draw;

    EKFSLAM ekfslam;
    ekfslam.Initialize(mapper.data.size(), 3);

    // for loop per iterating over number of measurements in sensor.dat
    for (int i = 0; i < measurements.data.size(); i++) { 
        const auto& curr_measurement = measurements.data[i];
        draw.Clear(); // assuming it resets all relevant draw parameters for next measurement
        ekfslam.ProcessMeasurement(curr_measurement);
        // inputs match Plot_State() from plotter.h
        draw.Plot_state(ekfslam.mu, ekfslam.Sigma, mapper, ekfslam.observedLandmarks, curr_measurement.laserR);
        draw.Pause(); // add 0.01s delay per measurement
        stringstream ss;
        ss << setfill('0') << setw(3) << i;
        draw.Save("../images/"+ss.str()); // save the path data taken by robot over current measurement
    }
    draw.Show(); // functions are from matplotlibcpp library

    return 0;
}
