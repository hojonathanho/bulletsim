#ifndef _SOFTBODIES_H_
#define _SOFTBODIES_H_

#include "environment.h"
#include "basicobjects.h"
#include "utils/config.h"

class BulletSoftObject : public EnvironmentObject {
private:
public:
	struct Tetra {
		btSoftBody::Node* m_n[4];
		btSoftBody::Face* m_f[4];
	};
	// Analogous to softBody->m_tetras, except that this is of our own datatype Tetra and not btSoftBody::Tetra . 
	// This is more useful because the Tetra data structure contains a pointer to faces.
	vector<Tetra> tetras_internal;
	// This is filled only for tetrameshes. It contains the faces being pointed by tetras_internal.
	btSoftBody::tFaceArray faces_internal;

	void computeNodeFaceMapping();
	void computeNodeFaceTetraMapping();
	void computeBoundaries();

	// The following 2 dimensional vectors map indices between nodes, faces and tetras.
	// size(node2faces)  = [ # nodes , # faces that node i is attached to ]
	// size(face2nodes)  = [ # faces , # nodes a face has (i.e. 3) ]
	// size(face2tetras) = [ # faces , # tetras that face j is attached to (i.e. 1 or 2) ]
	// size(tetra2faces) = [ # tetras , # faces that tetra t has (i.e. 4) ]
	vector<vector<int> > node2faces;
	vector<vector<int> > face2nodes;
	vector<vector<int> > face2tetras;
	vector<vector<int> > tetra2faces;
public:
	// Currently, this should only be filled for tetrameshes. node_boundaries[i] indicates if the node is at a boundary face.
	vector<bool> node_boundaries;
	// This should only be filled for tetrameshes. face_boundaries[j] indicates if faces_internal[j] is a boundary face.
	vector<bool> face_boundaries;

protected:
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::MatrixTransform> transform;

    osg::ref_ptr<osg::Geometry> trigeom;
    osg::ref_ptr<osg::Vec3Array> trivertices, trinormals;

    osg::ref_ptr<osg::Geometry> quadgeom;
    osg::ref_ptr<osg::Vec3Array> quadvertices, quadnormals;

public:
    osg::ref_ptr<osg::Vec2Array> tritexcoords;

public:
    typedef boost::shared_ptr<BulletSoftObject> Ptr;

    boost::shared_ptr<btSoftBody> softBody;

    // constructors/destructors
    BulletSoftObject(boost::shared_ptr<btSoftBody> softBody_) : softBody(softBody_), nextAnchorHandle(0)
    {
    	if (softBody->m_tetras.size() == 0)	computeNodeFaceMapping();
    	else {
    		computeNodeFaceTetraMapping();
    		computeBoundaries();
    	}
    }
    BulletSoftObject(btSoftBody *softBody_) : softBody(softBody_), nextAnchorHandle(0)
    {
			if (softBody->m_tetras.size() == 0)	computeNodeFaceMapping();
			else {
				computeNodeFaceTetraMapping();
				computeBoundaries();
   		}
		}
    virtual ~BulletSoftObject() { }

    // serialization (TODO: serialize anchors also?)
    // for saving and loading softbodies (text format)
    static Ptr createFromFile(btSoftBodyWorldInfo& worldInfo, const char* fileName);
    static Ptr createFromFile(btSoftBodyWorldInfo& worldInfo, istream &s);
    static void saveToFile(btSoftBody *psb, const char *fileName);
    static void saveToFile(btSoftBody *psb, ostream &s);
    virtual void saveToFile(const char *fileName) const;
    virtual void saveToFile(ostream &s) const;

    void setColor(float,float,float,float);

    // just sets the image, not the texture coordinates
    void setTexture(cv::Mat image);
    // sets the image and the texture coordinates
    void setTexture(cv::Mat image, const btTransform& camFromWorld);
    void setTexture(cv::Mat image, osg::Vec2Array* texcoords);
    cv::Point2f getTexCoord(int nodeIdx);
  void adjustTransparency(float increment);

		// for softbody transforms. look at EnvironmentObject for precise definition.
  int getIndex(const btTransform& transform);
  int getIndexSize();
  btTransform getIndexTransform(int index);

  bool checkIntersection(const btVector3& start, const btVector3& end);
  vector<btVector3> getIntersectionPoints(const btVector3& start, const btVector3& end);

    // custom anchor management
    typedef int AnchorHandle;
    AnchorHandle addAnchor(btSoftBody::Node *node, btRigidBody *body, btScalar influence=1);
    AnchorHandle addAnchor(int nodeidx, btRigidBody *body, btScalar influence=1);
    void removeAnchor(AnchorHandle a);
    int getAnchorIdx(AnchorHandle h) const;
    bool hasAnchorAttached(int nodeidx) const;

    EnvironmentObject::Ptr copy(Fork &f) const;
    void postCopy(EnvironmentObject::Ptr copy, Fork &f) const;

    // called by Environment
    void init();
    void preDraw();
    void destroy();

    osg::Node *getOSGNode() const { return transform.get(); }

    // utility functions

    // check for nan/inf in the soft body state
    bool validCheck(bool nodesOnly=true) const;
    bool fullValidCheck() const { return validCheck(false); }

private:
    bool enable_texture;
		osg::Vec4f m_color;
		void setColorAfterInit();
		osg::ref_ptr<osg::Image> m_image;
		boost::shared_ptr<cv::Mat> m_cvimage;
		void setTextureAfterInit();
    AnchorHandle nextAnchorHandle;
    map<AnchorHandle, int> anchormap;
public:
		cv::Mat getTexture() {
		  if(m_cvimage) return *m_cvimage;
		  else return cv::Mat();
		}
		osg::Vec4f getColor() {return m_color;}
};


BulletSoftObject::Ptr makeCloth(float sx, float sy, btVector3 t, int resolution_x, int resolution_y, float mass);
BulletSoftObject::Ptr makeCloth(const vector<btVector3>& corners, int resolution_x, int resolution_y, float mass);

// Assumes top_corners are in a plane parallel to the xy-plane
// The bottom corners are the top_corners shifted by thickness in the negative z direction
BulletSoftObject::Ptr makeSponge(const vector<btVector3>& top_corners, float thickness, float mass, float max_tet_vol=4.0*METERS*METERS*METERS/1000000.0);

#endif // _SOFTBODIES_H_
