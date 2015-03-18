#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";

Mat canny_output,hough_output;
int lowThreshold=100,upperThreshold=100,max_lowThreshold=255,max_upperThreshold=255;
vector<Vec2f> lines;

// void findEndPoints(float rho,float theta,int rows,int cols,Point *pt1,Point *pt2)//-----Was trying something earlier. Not necessary-------//
// {
//   double a=cos(theta),b=sin(theta);
//   pt1->x=(int)(rho/a);
//   pt2->x=(int)((rho-rows*b)/a);
//   if(theta!=0||theta!=CV_PI)
//   {
//        //---------Checking for the upper point----------//
//     if(pt1->x>=cols)
//       {
//         pt1->x=cols-1;
//         pt1->y=(int)((rho-a*(cols-1))/b);
//       }
//     else if(pt1->x<0)
//       {
//         pt1->x=0;
//         pt1->y=(int)(rho/b);
//       }
//     else
//       {
//         pt1->y=0;
//       }
//     //---------Checking for the lower point----------//
//     if(pt2->x>=cols)
//       {
//         pt2->x=cols-1;
//         pt2->y=(int)((rho-a*(cols-1))/b);
//       }
//     else if(pt2->x<0)
//       {
//         pt2->x=0;
//         pt2->y=(int)(rho/b);
//       }
//     else
//       {
//         pt2->y=rows-1;
//       } 
//   }
//   else if(theta==0)
//   {
//     pt1->x=rho;
//     pt2->x=rho;
//     pt1->y=0;
//     pt2->y=rows-1;
//   }
//   else
//   {
//     pt1->x=0;
//     pt1->y=rho;
//     pt2->x=cols-1;
//     pt2->y=rho;
//   }
 
// }

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1,&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // Taking the ROS image in CV_8UC1 format
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   
    //-----------Canny Detection---------------//

    /// Reduce noise with a kernel 3x3
    blur( cv_ptr->image, canny_output, Size(3,3) );
    createTrackbar( "Min Threshold:", OPENCV_WINDOW, &lowThreshold, max_lowThreshold);
    createTrackbar( "Max Threshold:", OPENCV_WINDOW, &upperThreshold, max_upperThreshold);
    Canny(canny_output,canny_output,lowThreshold,upperThreshold);
    //------------------------------------------//


    //---------------Hough Lines-----------------//
    hough_output.create(canny_output.size(),canny_output.type());
    hough_output=Scalar::all(0);
    HoughLines(canny_output, lines, 1, CV_PI/180, 100, 0, 0 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
       float rho = lines[i][0], theta = lines[i][1];
       Point pt1, pt2;
       double a = cos(theta), b = sin(theta);
       double x0 = a*rho, y0 = b*rho;
       pt1.x = cvRound(x0 + 1000*(-b));
       pt1.y = cvRound(y0 + 1000*(a));
       pt2.x = cvRound(x0 - 1000*(-b));
       pt2.y = cvRound(y0 - 1000*(a));
       line( hough_output, pt1, pt2, Scalar(255), 4, CV_AA);
    }
    //-------------------------------------------//

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, hough_output);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}