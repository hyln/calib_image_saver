#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "../include/calib_image_saver/chessboard/Chessboard.h"
#include <algorithm>
#include <atomic>
#include <boost/filesystem.hpp>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

rclcpp::Subscription< sensor_msgs::msg::Image >::SharedPtr image_sub;

std::string image_path;
bool is_use_OpenCV     = false;
bool is_show           = false;
std::string image_name = "IMG_";
cv::Size boardSize;

cv::Size image_size;
int image_count       = 0;
bool is_first_run     = true;
bool is_get_chessbord = false;
bool is_color         = false;
rclcpp::Time time_last, time_now;
int max_freq = 10;
int display_max_width = 1280;
int display_max_height = 720;
double min_save_interval = 1.0;
cv::Mat image_in, image_show;
cv::Mat DistributedImage;
std::vector< std::vector< cv::Point2f > > total_image_points;
std::mutex image_mutex;
std::mutex display_mutex;
std::condition_variable image_cv;
std::thread detection_thread;
std::atomic< bool > is_running{ true };
std::atomic< int > detection_state{ 0 };
std::chrono::steady_clock::time_point last_save_time = std::chrono::steady_clock::time_point::min( );
cv::Mat latest_image;
uint64_t latest_image_seq = 0;
uint64_t processed_image_seq = 0;

void
showImage( const cv::Mat& image, cv::Mat& _DistributedImage, bool mark_failed )
{
    if ( image.channels( ) == 1 )
        cv::cvtColor( image, image_show, cv::COLOR_GRAY2RGB );
    else
        image.copyTo( image_show );

    if ( mark_failed )
    {
        const int thickness = std::max( 4, std::min( image_show.cols, image_show.rows ) / 80 );
        cv::rectangle( image_show,
                       cv::Rect( 0, 0, image_show.cols, image_show.rows ),
                       cv::Scalar( 0, 0, 255 ),
                       thickness );
    }

    std::lock_guard< std::mutex > lock( display_mutex );

    cv::Mat imgROI = _DistributedImage( cv::Rect( image.cols, 0, image.cols, image.rows ) );
    image_show.copyTo( imgROI );

    cv::Mat display_image = _DistributedImage;
    const double scale = std::min( 1.0,
                                  std::min( static_cast< double >( display_max_width ) / _DistributedImage.cols,
                                            static_cast< double >( display_max_height ) / _DistributedImage.rows ) );
    if ( scale < 1.0 )
    {
        cv::resize( _DistributedImage, display_image, cv::Size( ), scale, scale, cv::INTER_AREA );
    }

    cv::namedWindow( "DistributedImage", cv::WINDOW_NORMAL );
    cv::imshow( "DistributedImage", display_image );
    cv::waitKey( 1 );
}

void
drawChessBoard( cv::Mat& image_input, cv::Mat& _DistributedImage, const std::vector< cv::Point2f >& imagePoints )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );

    cv::Mat& image = image_input;

    if ( image.channels( ) == 1 )
        cv::cvtColor( image, image, cv::COLOR_GRAY2RGB );

    for ( size_t j = 0; j < imagePoints.size( ); ++j )
    {
        cv::Point2f pObs = imagePoints.at( j );

        // green points is the observed points
        cv::circle( image,
                    cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ),
                    5,
                    green,
                    2,
                    cv::LINE_AA,
                    drawShiftBits );

        // yellow points is the observed points
        cv::circle( _DistributedImage,
                    cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ),
                    5,
                    yellow,
                    2,
                    cv::LINE_AA,
                    drawShiftBits );
    }

    cv::line( _DistributedImage, imagePoints.at( 0 ), imagePoints.at( boardSize.width - 1 ), green, 1 );
    cv::line( _DistributedImage,
              imagePoints.at( boardSize.width * ( boardSize.height - 1 ) ),
              imagePoints.at( 0 ),
              green,
              1 );
    cv::line( _DistributedImage,
              imagePoints.at( boardSize.width * ( boardSize.height - 1 ) ),
              imagePoints.at( boardSize.width * boardSize.height - 1 ),
              green,
              1 );
    cv::line( _DistributedImage,
              imagePoints.at( boardSize.width * boardSize.height - 1 ),
              imagePoints.at( boardSize.width - 1 ),
              green,
              1 );
}

void
callback_0( const sensor_msgs::msg::Image::ConstSharedPtr img )
{
    std::string encoding = img->encoding;
    if ( encoding.compare( 0, 4, "mono8" ) == 0 )
        is_color = false;
    else if ( encoding.compare( 0, 4, "bgr8" ) == 0 )
        is_color = true;

    if ( is_color )
        image_in = cv_bridge::toCvCopy( img, "bgr8" )->image;
    else
        image_in = cv_bridge::toCvCopy( img, "mono8" )->image;
    time_now = img->header.stamp;

    if ( is_first_run )
    {
        time_last         = img->header.stamp;
        image_size.height = img->height;
        image_size.width  = img->width;
        cv::Mat DistributedImage_tmp( cv::Size( image_size.width * 2, image_size.height ),
                                      CV_8UC3,
                                      cv::Scalar( 0 ) );

        DistributedImage_tmp.copyTo( DistributedImage );
        is_first_run = false;
    }

    {
        std::lock_guard< std::mutex > lock( image_mutex );
        image_in.copyTo( latest_image );
        ++latest_image_seq;
    }
    image_cv.notify_one( );

    if ( is_show )
    {
        showImage( image_in, DistributedImage, detection_state.load( ) == 2 );
    }
}

void
detectionWorker( )
{
    while ( is_running.load( ) )
    {
        cv::Mat image_input;
        uint64_t seq = 0;

        {
            std::unique_lock< std::mutex > lock( image_mutex );
            image_cv.wait( lock, [] {
                return !is_running.load( ) || latest_image_seq != processed_image_seq;
            } );

            if ( !is_running.load( ) )
                break;

            latest_image.copyTo( image_input );
            seq                 = latest_image_seq;
            processed_image_seq = latest_image_seq;
        }

        if ( image_input.empty( ) )
            continue;

        camera_model::Chessboard chessboard( boardSize, image_input );
        chessboard.findCorners( is_use_OpenCV );

        if ( chessboard.cornersFound( ) )
        {
            const auto now = std::chrono::steady_clock::now( );
            if ( last_save_time != std::chrono::steady_clock::time_point::min( )
                 && std::chrono::duration< double >( now - last_save_time ).count( ) < min_save_interval )
            {
                detection_state.store( 1 );
                continue;
            }

            std::stringstream ss_num;
            ss_num << image_count;
            std::string image_file = image_path + "/" + image_name + ss_num.str( ) + ".png";
            std::cout << "#[INFO] Get chessboard image: " << image_name + ss_num.str( ) + ".png" << std::endl;

            cv::imwrite( image_file, image_input );

            ++image_count;
            last_save_time = now;
            total_image_points.push_back( chessboard.getCorners( ) );

            if ( is_show )
            {
                std::lock_guard< std::mutex > lock( display_mutex );
                drawChessBoard( image_input, DistributedImage, total_image_points.back( ) );
            }

            is_get_chessbord = true;
            detection_state.store( 1 );
        }
        else
        {
            if ( detection_state.exchange( 2 ) != 2 )
            {
                std::cout << "#[ERROR] Get no chessboard image." << std::endl;
            }
        }

        ( void )seq;
    }
}

int
main( int argc, char** argv )
{
    rclcpp::init( argc, argv );
    auto node = rclcpp::Node::make_shared( "singleImageSaver" );

    max_freq      = node->declare_parameter< int >( "rate", max_freq );
    display_max_width = node->declare_parameter< int >( "display_max_width", display_max_width );
    display_max_height = node->declare_parameter< int >( "display_max_height", display_max_height );
    min_save_interval = node->declare_parameter< double >( "min_save_interval", min_save_interval );
    image_path    = node->declare_parameter< std::string >( "image_path", image_path );
    boardSize.width  = node->declare_parameter< int >( "board_width", boardSize.width );
    boardSize.height = node->declare_parameter< int >( "board_height", boardSize.height );
    is_use_OpenCV = node->declare_parameter< bool >( "is_use_OpenCV", is_use_OpenCV );
    is_show       = node->declare_parameter< bool >( "is_show", is_show );
    image_name    = node->declare_parameter< std::string >( "image_name", image_name );

    if ( !boost::filesystem::exists( image_path ) && !boost::filesystem::is_directory( image_path ) )
    {
        std::cerr << "#[ERROR] Cannot find Saving directory: " << image_path << "." << std::endl;
        return 1;
    }

    if ( boardSize.height <= 1 || boardSize.width <= 1 )
    {
        std::cout << "#[ERROR] Error with input chessbopard Size." << std::endl;
        return 0;
    }

    detection_thread = std::thread( detectionWorker );

    image_sub = node->create_subscription< sensor_msgs::msg::Image >( "/image_input", 1, callback_0 );

    rclcpp::spin( node );
    is_running.store( false );
    image_cv.notify_one( );
    if ( detection_thread.joinable( ) )
    {
        detection_thread.join( );
    }
    rclcpp::shutdown( );

    cv::imwrite( image_path + "/" + "Distributed.png", DistributedImage );
    std::cout << "#[INFO] Get chessboard iamges: " << image_count << std::endl;

    return 0;
}
