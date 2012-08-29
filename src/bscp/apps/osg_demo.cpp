/* OpenSceneGraph example, osgshape.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
//#include <osg/Texture2D>
//#include <osgUtil/ShaderGen>

#include <osgViewer/Viewer>

#include <osg/Math>

osg::Geode* createShapes()
{
    osg::Geode* geode = new osg::Geode();

    
    float radius = 0.8f;
    float height = 1.0f;
   
    osg::ShapeDrawable* box = new osg::ShapeDrawable(new osg::Box(osg::Vec3(2.0, 0.0, 0.0), 1.0));
    box->setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));
    geode->addDrawable(box);
    //geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),radius)));
    //geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(2.0f,0.0f,0.0f),2*radius)));
//    geode->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(4.0f,0.0f,0.0f),radius,height),hints));
//    geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(6.0f,0.0f,0.0f),radius,height),hints));
//    geode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(8.0f,0.0f,0.0f),radius,height),hints));
    return geode;
}

int main(int, char **)
{
    // construct the viewer.
    osgViewer::Viewer viewer;

    // add model to viewer.
    viewer.setSceneData( createShapes() );
    viewer.setUpViewInWindow(400, 400, 640, 480);

    return viewer.run();
}
