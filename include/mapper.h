/*
import landmark data from world.dat into MapPoint struct of class vector
*/
#pragma once

#include "./common.h"
#include <string>

// Define a LandMark on the map
// Consists of ID, x position, and y position
struct MapPoint{
    uint64_t id;
    float x;
    float y;
};

// Mapper Class
// data: Vector containing MapPoints which represent all landmarks
// initialize: Function to read in landmarks from specified filename
class Mapper {
 public:
    Mapper() {}
    void initialize(const string& filename);
    vector<MapPoint> data;
};

void Mapper::initialize(const string& filename) {
    ifstream in_file(filename, ifstream::in);
    // if file fails to open, print error code
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << filename << endl;
        exit(EXIT_FAILURE);
    }

    /* TODO: Complete initialize()
    entries are integer values split by whitespace
    entries: landmark id x-coordinate y-coordinate
    ie. "1 2 1" is Landmark 1 at position (2,1)
    */
   
    /*
    initialize vector called records with type struct called MapPoint with members id,x,y
    while loop parses per line entry in world.dat
    since ifstream imports file contents as a sequence of characters, convert each to string to float
    parse through line with substring function and whitespace delimiter
    pushback to create new row for next landmark entry into vector records
    
    make sure to initialize struct with appropriate corresponding members in the main
    not sure if works completely yet
    */
    /*
    std::string line; 
    MapPoint m;
    vector<MapPoint> records; 
    if (in_file.is_open()) {
        while (getline (in_file, line)){
                    m.id= stof(line.substr(0, line.find(' ')));
                    m.x = stof(line.substr(1, line.find(' ')));
                    m.y = stof(line.substr(2, line.find(' ')));
            } 
            records.push_back(m);
        }
        */
    // copied format from sensor_info.h
	string line;
	while(getline(in_file, line)) {
		istringstream ss(line);
		MapPoint map;
		ss>>map.id;
		ss>>map.x;
		ss>>map.y;
		data.push_back(map);
        // if(debug)?
		cout << data.back().id << ": " << data.back().x << ": " << data.back().y << endl;
	}
    if (in_file.is_open()) {
        in_file.close();
    }
}
