#ifndef _OSG_TOOLS_TORUS_H_
#define _OSG_TOOLS_TORUS_H_

#include <osg/Geode>
#include <osg/Vec4>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/ShadeModel>


class  Torus
{
public:
  Torus():
    _inner_radius(1.0f), _outer_radius(2.0f),
    _start_sweep(osg::inDegrees(0.0f)), _end_sweep(osg::inDegrees(360.0f)),
    _circle_cuts(20), _sweep_cuts(20) {}

  Torus(float inner,float outer,
        float start_angle=0.0f,float end_angle=360.0f,
        unsigned int cross=100,unsigned int sweep=20):
    _inner_radius(inner), _outer_radius(outer),
    _start_sweep(start_angle), _end_sweep(end_angle),
    _circle_cuts(cross), _sweep_cuts(sweep) {}

  Torus(const Torus& torus) :
    _inner_radius(torus._inner_radius), _outer_radius(torus._outer_radius),
    _start_sweep(torus._start_sweep), _end_sweep(torus._end_sweep),
    _circle_cuts(torus._circle_cuts), _sweep_cuts(torus._sweep_cuts) {}

  inline bool valid() const
    {
      bool validity(false);
      if( (_outer_radius>_inner_radius) && (_outer_radius>0.0f) )
        validity = true;
      return( validity );
    }

  inline void set(float i,float o) {_inner_radius=i; _outer_radius=o;}

  //osg::Geode* operator() () const;

  inline void setColor(const osg::Vec4& c) { _color = c; }
  inline const osg::Vec4& getColor() const { return _color; }

  inline void setInnerRadius(float inner) { _inner_radius = inner; }
  inline float getInnerRadius() const { return _inner_radius; }

  inline void setOuterRadius(float outer) { _outer_radius = outer; }
  inline float getOuterRadius() const { return _outer_radius; }

  inline void setStartSweep(float angle) { _start_sweep = angle; }
  inline float getStartSweep() const { return _start_sweep; }

  inline void setEndSweep(float angle) { _end_sweep = angle; }
  inline float getEndSweep() const { return _end_sweep; }

  inline void setCircleCuts(int cuts) { _circle_cuts = cuts; }
  inline int getCircleCuts() const { return _circle_cuts; }

  inline void setSweepCuts(int cuts) { _sweep_cuts = cuts; }
  inline int getSweepCuts() const { return _sweep_cuts; }

  osg::Geode* operator() () const
  {
    osg::Geode* geode = new osg::Geode;
    geode->setName("OsgTools_Torus_geode");

    bool create_body( this->valid() );

    bool create_caps(false);
  /// TODO: create_caps for torus
  //   if( (this->getEndSweep() - this->getStartSweep()) < osg::DegreesToRadians(360.0) )
  //     create_caps = true;

    if(create_body)
    {
      float inner = this->getInnerRadius();
      float outer = this->getOuterRadius();
      float tube_radius = (outer - inner)*0.5;
      float avg_center = (inner+outer)*0.5;

      float start_sweep = this->getStartSweep();
      float end_sweep = this->getEndSweep();

      int torus_sweeps = this->getSweepCuts();
      int circle_sweeps = this->getCircleCuts();

      float dsweep = (end_sweep - start_sweep)/(float)torus_sweeps;
      float dphi = osg::DegreesToRadians(360.0) / (float)circle_sweeps;

      for(int j=0; j<circle_sweeps; j++)
      {
        osg::Vec3Array* vertices = new osg::Vec3Array;
        osg::Vec3Array* normals = new osg::Vec3Array;

        float phi = dphi*(float)j;
        float cosPhi = cosf(phi);
        float sinPhi = sinf(phi);
        float next_cosPhi = cosf(phi+dphi);
        float next_sinPhi = sinf(phi+dphi);

        float z = tube_radius*sinPhi;
        float yPrime = avg_center + tube_radius*cosPhi;

        float next_z = tube_radius*next_sinPhi;
        float next_yPrime = avg_center + tube_radius*next_cosPhi;

        float old_x = yPrime*cosf(-dsweep);
        float old_y = yPrime*sinf(-dsweep);
        float old_z = z;

        for(int i=0; i<torus_sweeps; ++i)
        {
          float sweep = start_sweep + dsweep*i;
          float cosSweep = cosf(sweep);
          float sinSweep = sinf(sweep);

          float x = yPrime*cosSweep;
          float y = yPrime*sinSweep;

          float next_x = next_yPrime*cosSweep;
          float next_y = next_yPrime*sinSweep;

          vertices->push_back( osg::Vec3(next_x,next_y,next_z) );
          vertices->push_back( osg::Vec3(x,y,z) );

          // calculate normals
          osg::Vec3 lateral(next_x-x,next_y-y,next_z-z);
          osg::Vec3 longitudinal(x-old_x,y-old_y,z-old_z);
          osg::Vec3 normal = longitudinal ^ lateral;  // cross product
          normal.normalize();

          normals->push_back( normal );
          normals->push_back( normal );

          old_x = x; old_y = y; old_z = z;
        } // end torus loop

        // the last point
        float last_sweep = start_sweep + end_sweep;
        float cosLastSweep = cosf(last_sweep);
        float sinLastSweep = sinf(last_sweep);

        float x = yPrime*cosLastSweep;
        float y = yPrime*sinLastSweep;

        float next_x = next_yPrime*cosLastSweep;
        float next_y = next_yPrime*sinLastSweep;

        vertices->push_back( osg::Vec3(next_x,next_y,next_z) );
        vertices->push_back( osg::Vec3(x,y,z) );

        osg::Vec3 lateral(next_x-x,next_y-y,next_z-z);
        osg::Vec3 longitudinal(x-old_x,y-old_y,z-old_z);
        osg::Vec3 norm = longitudinal ^ lateral;
        norm.normalize();

        normals->push_back( norm );
        normals->push_back( norm );

        //osg::ShadeModel* shademodel = new osg::ShadeModel;
        //shademodel->setMode( osg::ShadeModel::SMOOTH );

        osg::StateSet* stateset = new osg::StateSet;
        //stateset->setAttribute( shademodel );
        stateset->setMode( GL_RESCALE_NORMAL, osg::StateAttribute::ON );


        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        geometry->setStateSet( stateset );
        geometry->setVertexArray( vertices );

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back( this->getColor() );
        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        geometry->setNormalArray( normals );
        geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP,
                                                      0,vertices->size()) );
        geode->addDrawable( geometry.get() );
      }  // end cirle loop

      if(create_caps)
      {
        /// TODO: make the starting cap
        {
          osg::ref_ptr<osg::Geometry> start_cap = new osg::Geometry;
          osg::Vec3Array* start_vertices = new osg::Vec3Array;

          float start_sweep = this->getStartSweep();
          float cosStartSweep = cosf(start_sweep);
          float sinStartSweep = sinf(start_sweep);
          float circle_cuts = this->getCircleCuts();
          float dPhi = osg::DegreesToRadians(360.0) / (float)circle_cuts;

          // define center point
          float tube_radius = (this->getInnerRadius() - this->getOuterRadius());
          float avg_center = (this->getInnerRadius() + this->getOuterRadius())*0.5;
          start_vertices->push_back( osg::Vec3(cosStartSweep*avg_center,
                                               sinStartSweep*avg_center,
                                               0.0f) );

          for(int i=0; i<circle_cuts; i++)
          {
            float phi = dPhi*(float)i;
            float cosPhi = cosf(phi);
            float sinPhi = cosf(phi);
            float yPrime = avg_center + tube_radius*cosPhi;
            float z = tube_radius*sinPhi;
            float x = cosStartSweep*yPrime;
            float y = sinStartSweep*yPrime;

            start_vertices->push_back( osg::Vec3(x,y,z) );
          }  // end circle loop

          // do last point by hand
          start_cap->setVertexArray( start_vertices );

          osg::Vec3Array* start_normals = new osg::Vec3Array;
          start_normals->push_back( osg::Vec3(cosStartSweep,sinStartSweep,0.0) );
          start_cap->setNormalArray( start_normals );
          start_cap->setNormalBinding( osg::Geometry::BIND_OVERALL );

          osg::Vec4Array* colors = new osg::Vec4Array;
          colors->push_back( this->getColor() );
          start_cap->setColorArray( colors );
          start_cap->setColorBinding( osg::Geometry::BIND_OVERALL );

          start_cap->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_FAN,
                                                         0,start_vertices->size()) );

          geode->addDrawable( start_cap.get() );
        } // end start cap

        /// TODO:  make the end cap
        {
         // float end_sweep = this->getEndSweep();
        } // end end cap
      } // endif create_caps
    } // endif create_body

    return( geode );
  }

  virtual ~Torus() {}

private:
  float _inner_radius, _outer_radius;
  float _start_sweep, _end_sweep;
  unsigned int _circle_cuts, _sweep_cuts;
  osg::Vec4 _color;
};




#endif // _OSG_TOOLS_TORUS_H_
