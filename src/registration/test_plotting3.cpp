#include "tracking/plotting_tracking.h"
#include "simulation/plotting.h"
#include "simulation/simplescene.h"

struct BoxPlot {
	PlotLines::Ptr boxLines;
	float yaw, cx, cy, cz, sx, sy, sz;
};

int main(int argc, char* argv[]) {
	ColorCloudPtr cups = readPCD("/home/joschu/Data/cups/cups1.pcd");
	PointCloudPlot::Ptr cloudPlot(new PointCloudPlot(2));
	cloudPlot->setPoints1(cups);

	Scene s;
	s.startViewer();
	s.env->add(cloudPlot);
	s.env->add(boxLines);
	s.startLoop();

}
