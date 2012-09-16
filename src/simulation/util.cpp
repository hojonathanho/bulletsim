#include "util.h"
#include "thread_socket_interface.h"
#include <boost/foreach.hpp>
#include "utils/conversions.h"

using namespace Eigen;

static Environment::Ptr gEnv;
void setGlobalEnv(Environment::Ptr env) { gEnv = env; }
Environment::Ptr getGlobalEnv() { return gEnv; }

void toggle(bool* b){
	*b = !(*b);
}

void add(int* n, int increment) {
	*n += increment;
}

namespace util {

osg::ref_ptr<osg::Vec3Array> toVec3Array(const std::vector<btVector3>& in) {
    osg::ref_ptr<osg::Vec3Array> out = new osg::Vec3Array();
    out->reserve(in.size());
    BOOST_FOREACH(const btVector3& pt, in) out->push_back(osg::Vec3(pt.x(),pt.y(),pt.z()));
    return out;
}

osg::ref_ptr<osg::Vec4Array> toVec4Array(const std::vector<btVector4>& in) {
    osg::ref_ptr<osg::Vec4Array> out = new osg::Vec4Array();
    out->reserve(in.size());
    BOOST_FOREACH(const btVector3& pt, in) out->push_back(osg::Vec4(pt.x(),pt.y(),pt.z(),pt.w()));
    return out;
}

osg::ref_ptr<osg::Vec3Array> toVec3Array(const Eigen::MatrixXf& in) {
		assert(in.cols() == 3);
    osg::ref_ptr<osg::Vec3Array> out = new osg::Vec3Array();
    out->reserve(in.rows());
    for (int row=0; row < in.rows(); row++)
    	out->push_back(osg::Vec3(in(row, 0), in(row, 1), in(row, 2)));
    return out;
}

osg::ref_ptr<osg::Vec4Array> toVec4Array(const Eigen::MatrixXf& in) {
		assert(in.cols() == 4);
    osg::ref_ptr<osg::Vec4Array> out = new osg::Vec4Array();
    out->reserve(in.rows());
    for (int row=0; row < in.rows(); row++)
    	out->push_back(osg::Vec4(in(row, 0), in(row, 1), in(row, 2), in(row, 3)));
    return out;
}

void drawSpheres(vector<btVector3> points, Vector3f color, float alpha, float radius, Environment::Ptr env) {
	PlotSpheres::Ptr plot_spheres(new PlotSpheres());
	env->add(plot_spheres);

	MatrixXf centers = toEigenMatrix(points);
	MatrixXf rgba(points.size(), 4);
	rgba << color.transpose().replicate(points.size(),1) , VectorXf::Constant(points.size(), alpha);
	VectorXf sizes = VectorXf::Constant(points.size(), radius);
	plot_spheres->plot(util::toVec3Array(centers), util::toVec4Array(rgba), toVec(sizes));
}

void drawSpheres(btVector3 point, Vector3f color, float alpha, float radius, Environment::Ptr env) {
	drawSpheres(vector<btVector3>(1,point), color, alpha, radius, env);
}

void drawLines(vector<btVector3> points0, vector<btVector3> points1, Vector3f color, float alpha, Environment::Ptr env) {
	assert(points0.size() == points1.size());
	PlotLines::Ptr plot_lines(new PlotLines());
	env->add(plot_lines);

	vector<btVector3> linePoints;
	vector<btVector4> lineColors;

	for (int i=0; i<points0.size(); i++) {
		linePoints.push_back(points0[i]);
		linePoints.push_back(points1[i]);
		lineColors.push_back(btVector4(color(0), color(1), color(2), alpha));
	}
	plot_lines->setPoints(linePoints, lineColors);
}

void drawPoly(vector<btVector3> points, Vector3f color, float alpha, Environment::Ptr env) {
	vector<btVector3> pts0;
	vector<btVector3> pts1;
	for (int i=0; i<(points.size()-1); i++) {
		pts0.push_back(points[i]);
		pts1.push_back(points[i+1]);
	}
	pts0.push_back(points[points.size()-1]);
	pts1.push_back(points[0]);

	drawLines(pts0, pts1, color, alpha, env);
}

void drawAxes(btTransform transform, float size, Environment::Ptr env) {
	PlotAxes::Ptr plot_axes(new PlotAxes());
	env->add(plot_axes);

	plot_axes->setup(transform, size);
}

static const btMatrix3x3 HAPTIC_ROTATION(btQuaternion(-M_PI/2., 0., 0.));
static inline btMatrix3x3 toHapticBtMatrix(const Matrix3d &m) {
    // note: the rows are permuted
    return btMatrix3x3(m(2, 0), m(2, 1), m(2, 2),
                       m(0, 0), m(0, 1), m(0, 2),
                       m(1, 0), m(1, 1), m(1, 2));
}
static inline btVector3 toHapticBtVector(const Vector3d &v) {
    // note: the components are permuted
    return btVector3(v.z(), v.x(), v.y());
}
bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2]) {
    Vector3d start_proxy_pos, end_proxy_pos;
    Matrix3d start_proxy_rot, end_proxy_rot;
    if (!getDeviceState(start_proxy_pos, start_proxy_rot, buttons0,
                        end_proxy_pos, end_proxy_rot, buttons1))
        return false;
    trans0 = btTransform(toHapticBtMatrix(start_proxy_rot) * HAPTIC_ROTATION,
                         toHapticBtVector(start_proxy_pos));
    trans1 = btTransform(toHapticBtMatrix(end_proxy_rot) * HAPTIC_ROTATION,
                         toHapticBtVector(end_proxy_pos));


    return true;
}


  void sendRobotState(btVector3 pos0, btVector3 pos1) {
    double out0[3] = {pos0[1], pos0[2], pos0[0]};
    double out1[3] = {pos1[1], pos1[2], pos1[0]};
    sendDeviceState(out0, true, out1, true);
    cout << "send pos: " << out0[0] << " " << out0[1] << " " << out0[2] << ", " << 
      out1[0] << " " << out1[1] << " " << out1[2] << endl;
  }

}
