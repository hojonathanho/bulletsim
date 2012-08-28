#ifndef _SOFTBODIES_H_
#define _SOFTBODIES_H_

#include "environment.h"
#include "basicobjects.h"

class BulletSoftObject : public EnvironmentObject {
private:
	void computeNodeFaceMapping();
	vector<vector<int> > node2faces;
	vector<vector<int> > face2nodes;

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
    BulletSoftObject(boost::shared_ptr<btSoftBody> softBody_) : softBody(softBody_), nextAnchorHandle(0) { computeNodeFaceMapping(); }
    BulletSoftObject(btSoftBody *softBody_) : softBody(softBody_), nextAnchorHandle(0) { computeNodeFaceMapping(); }
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
		cv::Mat m_cvimage;
		void setTextureAfterInit();
    AnchorHandle nextAnchorHandle;
    map<AnchorHandle, int> anchormap;
public:
		cv::Mat getTexture() { return m_cvimage; }
		osg::Vec4f getColor() {return m_color;}
};

#endif // _SOFTBODIES_H_
