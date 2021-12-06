#include "april_detection.h"


AprilDetection::AprilDetection(){

  a_detector = apriltag_detector_create();

  // tag36h11
  tf = tag36h11_create();

  //getopt_t *getopt = getopt_create();
  //const char *famname = getopt_get_string(getopt, "family");

  // configure detector
  //apriltag_detector_add_family_bits(a_detector, tf, getopt_get_int(getopt, "hamming"));

  //a_detector->quad_decimate = getopt_get_double(getopt, "decimate");
  //a_detector->quad_sigma = getopt_get_double(getopt, "blur");
  //a_detector->nthreads = getopt_get_int(getopt, "threads");
  //a_detector->debug = getopt_get_bool(getopt, "debug");
  //a_detector->refine_edges = getopt_get_bool(getopt, "refine-edges");

  apriltag_detector_add_family(a_detector, tf);
  

  return;
}

AprilDetection::~AprilDetection(){
  return;
}

pair<vector<apriltag_pose_t>, cv::Mat> AprilDetection::processImage(cv::Mat image){

  cv::Mat image_gray; 
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  image_u8_t im = { .width  = image_gray.cols,
                    .height = image_gray.rows,
                    .stride = image_gray.cols, 
                    .buf    = image_gray.data 
  };


  zarray_t * detections = apriltag_detector_detect(a_detector, &im);

  apriltag_detection_t *det;
  apriltag_detection_info_t tag_info; 
  vector<apriltag_pose_t> poses;

  tag_info.tagsize = 0.159;
  tag_info.fx = 663.57507; 
  tag_info.fy = 694.47272;
  tag_info.cx = 956.22994;
  tag_info.cy = 539.54574;

  for (int i=0; i<zarray_size(detections); i++){
    zarray_get(detections, i, &det);
    std::cout << "Tag detected: " << det->id << std::endl; 
    tag_info.det = det;
    apriltag_pose_t pose;

    // estimate pose 
    estimate_tag_pose(&tag_info, &pose);
    poses.push_back(pose);
    /*
    std::cout << "R: "; 
    for (int i=0; i<9; i++){
      std::cout << pose.R->data[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "t: " << std::endl; 
    for (int i=0; i<3; i++){
	    std::cout << pose.t->data[i];
    }
    std::cout << std::endl;
    */
  }
  
  return make_pair(poses, image);

}