#include <osg/Geometry>
#include <osg/Geode>
#include <osg/StateSet>
#include "clouds/utils_pcl.h"

class PlotPoints : public osg::Geode {
public:
	typedef boost::shared_ptr<PlotPoints> Ptr;
	PlotPoints(float size=5);
	void setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols);
	void setPoints(const ColorCloudPtr& cloud);
	void clear();
protected:
	osg::Vec4 m_defaultColor;
	osg::ref_ptr<osg::Geometry> m_geom;
};



//class PlotCurve : public osg::Geode {
//
//public:
//  typedef osg::ref_ptr<PlotCurve> Ptr;
//  osg::Vec4 m_defaultColor;
//  osg::ref_ptr<osg::Geometry> m_geom;
//
//  PlotCurve(float width=5);
//  void setPoints(const std::vector<btVector3>& pts, const std::vector<btVector4>& cols);
////  void setPoints(const std::vector<btVector3>& pts);
//  void setPoints(const osg::ref_ptr<osg::Vec3Array>& pts, const osg::ref_ptr<osg::Vec4Array>& cols);
////  void setPoints(const osg::ref_ptr<osg::Vec3Array>& pts);
//};
