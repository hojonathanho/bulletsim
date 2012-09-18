/*
 * file.cpp
 *
 *  Created on: Sep 11, 2012
 *      Author: alex
 */

#include "file.h"
#include <iostream>
#include <fstream>
#include "my_exceptions.h"

using namespace std;

bool savePoints(const std::string& filename, const std::vector<geometry_msgs::Point32>& points) {
	ofstream file;
	file.open(filename.c_str());
	if (file.fail()) {
		cout << "geometry_msgs::Point32 points couldn't be saved to " << filename << endl;
		return false;
	}
	file.precision(20);

	file << points.size() << "\n";
	for (int i=0; i<points.size(); i++)
		file << points[i].x << " " << points[i].y << " " << points[i].z << "\n";
	file << "\n";

	file.close();
	cout << "geometry_msgs::Point32 points saved to " << filename << endl;
	return true;
}


bool loadPoints(const std::string& filename, std::vector<geometry_msgs::Point32>& points) {
  ifstream file;
  file.open(filename.c_str());
  if (file.fail()) throw FileOpenError(filename);

  int points_size;
  file >> points_size;
  points.resize(points_size);
  while (!file.eof()) {
  	for (int i=0; i<points_size; i++) {
			file >> points[i].x;
			file >> points[i].y;
			file >> points[i].z;
		}
  }

  file.close();
	cout << "geometry_msgs::Point32 points loaded from " << filename << endl;
  return true;
}
