/*
 * tetgen_helpers.cpp
 *
 *  Created on: Aug 2, 2012
 *      Author: alex
 */

#include "tetgen.h"
#include "tetgen_helpers.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

#define BUFFERSIZE 100


btSoftBody* CreateFromTetGenFile(btSoftBodyWorldInfo& worldInfo,
																	const char* ele_filename,
																	const char* face_filename,
																	const char* node_filename,
																	bool bfacelinks,
																	bool btetralinks,
																	bool bfacesfromtetras)
{
	//Open files
	ifstream ele_f, face_f, node_f;
	ele_f.open(ele_filename);
	face_f.open(face_filename);
	node_f.open(node_filename);

	//Read them into strings
	string temp;
	stringstream ele, face, node;
	while (ele_f.good()) {
			getline(ele_f, temp);
			ele << temp << "\n";
	}

	while (face_f.good()) {
			getline(face_f, temp);
			face << temp << "\n";
	}

	while (node_f.good()) {
			getline(node_f, temp);
			node << temp << "\n";
	}

	//Close the files
	ele_f.close(); face_f.close(); node_f.close();

	//Create your psb
	btSoftBody* psb=btSoftBodyHelpers::CreateFromTetGenData(worldInfo,
																													ele.str().c_str(),
																													face.str().c_str(),
																													node.str().c_str(),
																													bfacelinks,btetralinks,bfacesfromtetras);
	return psb;
}

// Very similar to save_nodes in tetgen.cxx
string get_nodes_string(tetgenio& out) {
  int i, j;

  int oldfirstnumber = out.firstnumber;
  out.firstnumber = 0;

  char buffer[BUFFERSIZE];
  stringstream ss;

  sprintf(buffer, "%d  %d  %d  %d\n", out.numberofpoints, out.mesh_dim,
  		out.numberofpointattributes, out.pointmarkerlist != NULL ? 1 : 0);
  ss << buffer;
  for (i = 0; i < out.numberofpoints; i++) {
    if (out.mesh_dim == 2) {
      sprintf(buffer, "%d  %.16g  %.16g", i + out.firstnumber, out.pointlist[i * 3],
      		out.pointlist[i * 3 + 1]);
    } else {
      sprintf(buffer, "%d  %.16g  %.16g  %.16g", i + out.firstnumber,
      		out.pointlist[i * 3], out.pointlist[i * 3 + 1], out.pointlist[i * 3 + 2]);
    }
    ss << buffer;
    for (j = 0; j < out.numberofpointattributes; j++) {
      sprintf(buffer, "  %.16g",
      		out.pointattributelist[i * out.numberofpointattributes + j]);
      ss << buffer;
    }
    if (out.pointmarkerlist != NULL) {
      sprintf(buffer, "  %d", out.pointmarkerlist[i]);
      ss << buffer;
    }
    ss << "\n";
  }
  out.firstnumber = oldfirstnumber;
  return ss.str();
}

// Very similar to save_ele in tetgen.cxx
string get_ele_string(tetgenio& out) {
  int i, j;

  int oldfirstnumber = out.firstnumber;
  out.firstnumber = 0;

  char buffer[BUFFERSIZE];
  stringstream ss;

  if (out.mesh_dim == 3) {
    sprintf(buffer, "%d  %d  %d\n", out.numberoftetrahedra, out.numberofcorners,
    		out.numberoftetrahedronattributes);
    ss << buffer;
    for (i = 0; i < out.numberoftetrahedra; i++) {
      sprintf(buffer, "%d", i + out.firstnumber);
      ss << buffer;
      for (j = 0; j < out.numberofcorners; j++) {
        sprintf(buffer, "  %5d", out.tetrahedronlist[i * out.numberofcorners + j]);
        ss << buffer;
      }
      for (j = 0; j < out.numberoftetrahedronattributes; j++) {
        sprintf(buffer, "  %g",
        		out.tetrahedronattributelist[i * out.numberoftetrahedronattributes + j]);
        ss << buffer;
      }
      ss << "\n";
    }
  } else {
    // Save a two-dimensional mesh.
    sprintf(buffer, "%d  %d  %d\n",out.numberoftrifaces,3,out.trifacemarkerlist ? 1 : 0);
    ss << buffer;
    for (i = 0; i < out.numberoftrifaces; i++) {
      sprintf(buffer, "%d", i + out.firstnumber);
      ss << buffer;
      for (j = 0; j < 3; j++) {
        sprintf(buffer, "  %5d", out.trifacelist[i * 3 + j]);
        ss << buffer;
      }
      if (out.trifacemarkerlist != NULL) {
        sprintf(buffer, "  %d", out.trifacemarkerlist[i]);
        ss << buffer;
      }
      ss << "\n";
    }
  }
  out.firstnumber = oldfirstnumber;
  return ss.str();
}

// Very similar to save_faces in tetgen.cxx
string get_faces_string(tetgenio& out) {
  int i;

  int oldfirstnumber = out.firstnumber;
  out.firstnumber = 0;

  char buffer[BUFFERSIZE];
  stringstream ss;

  sprintf(buffer, "%d  %d\n", out.numberoftrifaces,
  		out.trifacemarkerlist != NULL ? 1 : 0);
  ss << buffer;
  for (i = 0; i < out.numberoftrifaces; i++) {
    sprintf(buffer, "%d  %5d  %5d  %5d", i + out.firstnumber, out.trifacelist[i * 3],
    		out.trifacelist[i * 3 + 1], out.trifacelist[i * 3 + 2]);
    ss << buffer;
    if (out.trifacemarkerlist != NULL) {
      sprintf(buffer, "  %d", out.trifacemarkerlist[i]);
      ss << buffer;
    }
    ss << "\n";
  }
  out.firstnumber = oldfirstnumber;
  return ss.str();
}

// corners_base is clockwise
// quality:  Quality mesh generation. Minimum radius-edge ratio.
// max_tet_vol: Maximum tetrahedron volume constraint.
btSoftBody* CreatePrism(btSoftBodyWorldInfo& worldInfo,
		const vector<btVector3>& corners_base,
		const btVector3 &polygon_translation,
		float quality,
		float max_tet_vol,
		bool bfacelinks,
		bool btetralinks,
		bool bfacesfromtetras)
{

//  vector<btVector3> corners_base;
//  corners_base.push_back(btVector3(0,0,0));
//  corners_base.push_back(btVector3(2,0,0));
//  corners_base.push_back(btVector3(2,2,0));
//  corners_base.push_back(btVector3(0,2,0));
//  btVector3 polygon_translation = btVector3(0,0,12);


	tetgenio in, out;
  tetgenio::facet *f;
  tetgenio::polygon *p;
  int i, j;

  // All indices start from 1.
  in.firstnumber = 1;

  in.numberofpoints = corners_base.size() * 2;
  in.pointlist = new REAL[in.numberofpoints * 3];
  // Set base nodes
  for (i=0; i<corners_base.size(); i++) {
  	in.pointlist[i * 3]     = corners_base[i].x();
		in.pointlist[i * 3 + 1] = corners_base[i].y();
		in.pointlist[i * 3 + 2] = corners_base[i].z();
  }
  // Set top nodes
  for (i=0; i<corners_base.size(); i++) {
    in.pointlist[(i+corners_base.size()) * 3]     = polygon_translation.x() + in.pointlist[i * 3];
    in.pointlist[(i+corners_base.size()) * 3 + 1] = polygon_translation.y() + in.pointlist[i * 3 + 1];
    in.pointlist[(i+corners_base.size()) * 3 + 2] = polygon_translation.z() + in.pointlist[i * 3 + 2];
  }

  in.numberoffacets = corners_base.size() + 2;
  in.facetlist = new tetgenio::facet[in.numberoffacets];
  in.facetmarkerlist = new int[in.numberoffacets];

  // Facet 0. The base
  f = &in.facetlist[0];
  f->numberofpolygons = 1;
  f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
  f->numberofholes = 0;
  f->holelist = NULL;
  p = &f->polygonlist[0];
  p->numberofvertices = corners_base.size();
  p->vertexlist = new int[p->numberofvertices];
  for (j=0; j<p->numberofvertices; j++)
  	p->vertexlist[j] = j+1;

  // Facet 1. The top
	f = &in.facetlist[1];
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	p = &f->polygonlist[0];
	p->numberofvertices = corners_base.size();
	p->vertexlist = new int[p->numberofvertices];
	for (j=0; j<p->numberofvertices; j++)
		p->vertexlist[j] = j+corners_base.size()+1;

	// Face i. The sides
  for (i=2; i<in.numberoffacets; i++) {
		f = &in.facetlist[i];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = 4;
		p->vertexlist = new int[p->numberofvertices];
		p->vertexlist[0] = i-2                                               +1;
		p->vertexlist[1] = i-2                          +corners_base.size() +1;
		p->vertexlist[2] = (i-2 +1)%corners_base.size() +corners_base.size() +1;
		p->vertexlist[3] = (i-2 +1)%corners_base.size()                      +1;
  }

  // Set 'in.facetmarkerlist'

  in.facetmarkerlist[0] = -1;
  in.facetmarkerlist[1] = -2;
  for (i=2; i<in.numberoffacets; i++)
  	in.facetmarkerlist[i] = 0;

  // Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
  //   do quality mesh generation (q) with a specified quality bound
  //   (quality), apply a maximum volume constraint (a) (max_tet_vol),
  //   start indices at zero (z) and quiet output (Q).
  char switches[BUFFERSIZE];
  sprintf(switches, "pq%fa%fzQ", quality, max_tet_vol);
  tetrahedralize(switches, &in, &out);

	//Create your psb
	btSoftBody* psb=btSoftBodyHelpers::CreateFromTetGenData(worldInfo,
																													get_ele_string(out).c_str(),
																													get_faces_string(out).c_str(),
																													get_nodes_string(out).c_str(),
																													bfacelinks,btetralinks,bfacesfromtetras);
	return psb;
}

#undef BUFFERSIZE
