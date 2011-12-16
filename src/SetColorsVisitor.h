#include <osg/Transform>
#include <osg/Geometry>
#include <osg/Geode>
#include <iostream>

class SetColorsVisitor : public osg::NodeVisitor
{

public:
  osg::Vec4Array* colors;

  SetColorsVisitor(float r, float g, float b, float a ) {
    colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(r,g,b,a));
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
  }

  void apply( osg::Geode& geode ) {
    for (int i=0; i < geode.getNumDrawables(); i++) {
      applyDrawable(geode.getDrawable(i));
    }
  }

protected:
  void applyDrawable( osg::Drawable* drawable ) {
    osg::Geometry* geom = drawable->asGeometry();
    if (geom) {
      geom->setColorArray(colors); 
      geom->setColorBinding(osg::Geometry::BIND_OVERALL); 
    }
  }
};
