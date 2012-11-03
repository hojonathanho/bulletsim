#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/assign.hpp>
#include "utils/config.h"

using namespace std;

struct VideoMergerConfig : Config {
  static std::vector<std::string> inputFiles;
  static std::string outputFile;

  VideoMergerConfig() : Config() {
        params.push_back(new ParameterVec<std::string>("inputFiles", &inputFiles, "input files"));
        params.push_back(new Parameter<std::string>("outputFile", &outputFile, "output file"));
    }
};

std::vector<std::string> VideoMergerConfig::inputFiles = boost::assign::list_of("/home/alex/rll/videos/topic_fold_diag_left_color1camera.avi")("/home/alex/rll/videos/fold_diag_left_color1camera.avi");
std::string VideoMergerConfig::outputFile = "/home/alex/fold_diag_left_merged.avi";

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(VideoMergerConfig());
  parser.read(argc, argv);

  vector<cv::VideoCapture> video_captures;
  int out_frame_count;
  int out_frame_width = 0;
  for (int i=0; i<VideoMergerConfig::inputFiles.size(); i++) {
  	video_captures.push_back(cv::VideoCapture(VideoMergerConfig::inputFiles[i]));
  	if (!video_captures[i].isOpened())
 			runtime_error("Video input file " + VideoMergerConfig::inputFiles[i] + " doesn't exits.");

  	if (i==0) out_frame_count = video_captures[i].get(CV_CAP_PROP_FRAME_COUNT);
  	else out_frame_count = min(out_frame_count, (int) video_captures[i].get(CV_CAP_PROP_FRAME_COUNT));
  	out_frame_width += video_captures[i].get(CV_CAP_PROP_FRAME_WIDTH);
  }

  // assumes height and fps are the same for all the input images
	int out_frame_height = video_captures[0].get(CV_CAP_PROP_FRAME_HEIGHT);
	double out_fps = video_captures[0].get(CV_CAP_PROP_FPS);
	int out_codec = video_captures[0].get(CV_CAP_PROP_FOURCC);

  cv::VideoWriter video_writer(VideoMergerConfig::outputFile, out_codec, out_fps, cv::Size(out_frame_width, out_frame_height));
  if (!video_writer.isOpened())
		runtime_error("Video output file " + VideoMergerConfig::outputFile + " doesn't exits.");

  for(int c=0; c<out_frame_count; c++) {
  	cv::Mat image_out(out_frame_height, out_frame_width, CV_8UC3);
  	int start_col_index = 0;
  	for (int i=0; i<video_captures.size(); i++) {
  		cv::Mat image_in;
  		video_captures[i] >> image_in;
  		image_in.copyTo(image_out.colRange(cv::Range(start_col_index, start_col_index+image_in.cols)));
  		start_col_index += image_in.cols;
  	}
  	video_writer << image_out;
  }
  return 0;
}
