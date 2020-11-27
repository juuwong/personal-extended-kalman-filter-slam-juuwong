
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
    read from world.dat
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
    std::string line; 
    MapPoint m;
    vector<MapPoint> records; 
    if (in_file.is_open()) {
        while (getline (in_file, line)){
                int space1 = line.find(' ');
                int space2 = line.find(' ', space1 + 1);
                m.id= stof(line.substr(0, space1)); 
                m.x = stof(line.substr(space1 + 1, space2 - space1 - 1));
                m.y = stof(line.substr(space2 + 1));
            } 
            records.push_back(m);
        }
        in_file.close();
}
