#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/util.h"
#include "simulation/plotting.h"
#include "simulation/bullet_io.h"

using namespace std;

PlotSpheres::Ptr spheres;
PlotLines::Ptr lines;

void plot(const btSoftBody::Node& node, const btSoftBody::Face& face) {
	spheres->plot(util::toVec3Array(vector<btVector3>(1,node.m_x)), util::toVec4Array(vector<btVector4>(1,btVector4(0,1,0,1))), vector<float>(1, 0.005*METERS));

	vector<btVector3> vertices;
	for (int d=0; d<3; d++) {
		vertices.push_back(face.m_n[d]->m_x);
		vertices.push_back(face.m_n[(d+1)%3]->m_x);
	}
	lines->setPoints(vertices, vector<btVector4>(vertices.size(), btVector4(0,0,1,1)));
}

//void plot(const btSoftBody::Face& face, const btSoftBody::Tetra& tetra)  {
//	vector<btVector3> face_v;
//	for (int d=0; d<3; d++)
//		face_v.push_back(face.m_n[d]->m_x);
//	spheres->plot(util::toVec3Array(face_v), util::toVec4Array(vector<btVector4>(face_v.size(),btVector4(0,0,1,1))), vector<float>(1, 0.005*METERS));
//
//	vector<btVector3> tetra_v;
//	for (int d=0; d<4; d++) {
//		for (int c=d+1; c<4; c++) {
//			tetra_v.push_back(tetra.m_n[d]->m_x);
//			tetra_v.push_back(tetra.m_n[c]->m_x);
//		}
//	}
//	lines->setPoints(tetra_v, vector<btVector4>(tetra_v.size(), btVector4(0,0,1,1)));
//}
//
//void plot(const btSoftBody::Face& face, const BulletSoftObject::Tetra& tetra)  {
//	btSoftBody::Tetra btTetra;
//	for (int c=0; c<4; c++) {
//		btTetra.m_n[c] = tetra.m_n[c];
//	}
//	plot(face, btTetra);
//}

void plot(const btSoftBody::Node& node, const btSoftBody::Tetra& tetra)  {
	spheres->plot(util::toVec3Array(vector<btVector3>(1,node.m_x)), util::toVec4Array(vector<btVector4>(1,btVector4(0,1,0,1))), vector<float>(1, 0.005*METERS));

	vector<btVector3> vertices;
	for (int d=0; d<4; d++) {
		for (int c=d+1; c<4; c++) {
			vertices.push_back(tetra.m_n[d]->m_x);
			vertices.push_back(tetra.m_n[c]->m_x);
		}
	}
	lines->setPoints(vertices, vector<btVector4>(vertices.size(), btVector4(0,0,1,1)));
}

void clip(int& i, int max) {
	assert(max!=0);
	i = (i+max)%max;
}

int main(int argc, char *argv[]) {
	GeneralConfig::scale = 100;
	BulletConfig::maxSubSteps = 0;
  BulletConfig::gravity = btVector3(0,0,-0.1);

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.read(argc, argv);

	Scene scene;
	util::setGlobalEnv(scene.env);

	float s = 0.20 * METERS;
	float h = 0.10 * METERS;

	vector<btVector3> top_corners;
	top_corners.push_back(btVector3(-s/2.0, -s/2.0, h+0.01*METERS));
	top_corners.push_back(btVector3(-s/2.0, s/2.0, h+0.01*METERS));
	top_corners.push_back(btVector3(s/2.0, s/2.0, h+0.01*METERS));
	top_corners.push_back(btVector3(s/2.0, -s/2.0, h+0.01*METERS));
	BulletSoftObject::Ptr sponge = makeSponge(top_corners, h, 3);
	scene.env->add(sponge);
	sponge->setColor(0,.5,.5,1);

	scene.startViewer();

	bool exit_loop = false;
	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop), "exit");
  scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, sponge.get(), 0.1f), "increase opacity");
  scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, sponge.get(), -0.1f), "decrease opacity");

	int i = 0;
	scene.addVoidKeyCallback('n',boost::bind(add, &i, 1), "next");
	scene.addVoidKeyCallback('b',boost::bind(add, &i, -1), "back");

	int c = 0;
	scene.addVoidKeyCallback('j',boost::bind(add, &c, 1), "next");
	scene.addVoidKeyCallback('h',boost::bind(add, &c, -1), "back");

	int d = 0;
	scene.addVoidKeyCallback('u',boost::bind(add, &d, 1), "next");
	scene.addVoidKeyCallback('y',boost::bind(add, &d, -1), "back");

	int mode = 0;
	scene.addVoidKeyCallback('m',boost::bind(add, &mode, 1), "next");

	btSoftBody::tNodeArray& nodes = sponge->softBody->m_nodes;
	btSoftBody::tFaceArray& faces_internal = sponge->faces_internal;
	btSoftBody::tTetraArray& tetras = sponge->softBody->m_tetras;
	vector<BulletSoftObject::Tetra> tetras_internal = sponge->tetras_internal;

	spheres.reset(new PlotSpheres);
	lines.reset(new PlotLines);
	scene.env->add(spheres);
	scene.env->add(lines);

//	cout << tetras_internal.size() << " " << faces_internal.size() << endl;
	while (!exit_loop) {
		scene.env->step(.03,2,.015);

//		cout << i << " " << d << " " << c << endl;
		if (mode == 0) {
			clip(i, nodes.size());
			clip(c, sponge->node2faces[i].size());
			plot(nodes[i], faces_internal[sponge->node2faces[i][c]]);
		} else if (mode == 1) {
			clip(i, nodes.size());
			clip(d, sponge->node2faces[i].size());
			clip(c, sponge->face2tetras[ sponge->node2faces[i][d] ].size());
			plot(nodes[i], tetras[sponge->face2tetras[ sponge->node2faces[i][d] ] [c]]);
		} else if (mode == 2) {
			vector<btVector3> boundary_vertices;
			for (int n=0; n<nodes.size(); n++) {
				if (sponge->node_boundaries[n])
					boundary_vertices.push_back(nodes[n].m_x);
			}
			spheres->plot(util::toVec3Array(boundary_vertices), util::toVec4Array(vector<btVector4>(boundary_vertices.size(),btVector4(0,1,0,1))), vector<float>(boundary_vertices.size(), 0.005*METERS));
			lines->clear();
		} else {
			mode = 0;
		}

		scene.draw();
	}

	return 0;
}

