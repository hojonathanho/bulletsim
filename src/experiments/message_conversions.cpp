/*
 * message_conversions.cpp
 *
 *  Created on: Sep 12, 2012
 *      Author: alex
 */

#include <ros/ros.h>
#include "utils/conversions.h"
#include "simulation/softbodies.h"
#include "bulletsim_msgs/BulletSoftObject.h"
#include "bulletsim_msgs/Face.h"
#include "visualization_msgs/Marker.h"

//Fill in the header yourself
bulletsim_msgs::BulletSoftObject toBulletSoftObjectMsg(BulletSoftObject::Ptr sim) {
	bulletsim_msgs::BulletSoftObject msg;

	msg.header.frame_id = "ground";
	msg.header.stamp = ros::Time();

	const btSoftBody::tNodeArray& nodes = sim->softBody->m_nodes;
	const btSoftBody::tFaceArray& faces = sim->softBody->m_faces;

	for (int i=0; i<nodes.size(); i++) {
		msg.vertices.push_back(toROSPoint(nodes[i].m_x));
		msg.normals.push_back(toROSPoint(nodes[i].m_n));
	}

	// compute face to nodes indices
	vector<vector<int> > face2nodes(faces.size(), vector<int>(3,-1));
	for (int i=0; i<nodes.size(); i++) {
		int j,c;
		for(j=0; j<faces.size(); j++) {
			for(c=0; c<3; c++) {
				if (&nodes[i] == faces[j].m_n[c]) {
					face2nodes[j][c] = i;
				}
			}
		}
	}

	for (int j=0; j<faces.size(); j++) {
		bulletsim_msgs::Face face;
		for (int c=0; c<3; c++) {
			face.vertex_inds.push_back(face2nodes[j][c]);
			face.normal_inds.push_back(face2nodes[j][c]);
		}
		msg.faces.push_back(face);
	}
}

visualization_msgs::Marker toMarker(bulletsim_msgs::Mesh soft_body_msg) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	for (int j=0; j<soft_body_msg.faces.size(); j++) {
		for (int c=0; c<3; c++) {
			int i = soft_body_msg.faces[j].vertex_inds[c];
			marker.points.push_back(soft_body_msg.vertices[i]);
		}
	}
}
