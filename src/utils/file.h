/*
 * file.h
 *
 *  Created on: Sep 11, 2012
 *      Author: alex
 */

#ifndef FILE_H_
#define FILE_H_

#include <string>
#include <vector>
#include <geometry_msgs/Point32.h>

bool savePoints(const std::string& filename, const std::vector<geometry_msgs::Point32>& points);
bool loadPoints(const std::string& filename, std::vector<geometry_msgs::Point32>& points);

#endif /* FILE_H_ */
