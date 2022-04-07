
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <pthread.h>
#include <opencv2/opencv.hpp>

#include "libcamera_app.h"
#include "libcamera_app_options.h"

namespace lccv {

class PiCamera {
public:
    PiCamera();
    ~PiCamera();

    Options *options;

    //Photo mode
    bool startPhoto();
    bool capturePhoto(cv::Mat &frame);
    bool stopPhoto();

    //Video mode
    bool startVideo();
    bool getVideoFrame(cv::Mat &frame, unsigned int timeout);
    void stopVideo();

protected:
    void run();
protected:
    LibcameraApp *app;
    void getImage(cv::Mat &frame, CompletedRequestPtr &payload);
    static void *videoThreadFunc(void *p);
    pthread_t videothread;
    unsigned int still_flags;
    unsigned int vw,vh,vstr;
    std::atomic<bool> running,frameready;
    uint8_t *framebuffer;
    std::mutex mtx;
    bool camerastarted;
};

}

int main()
{
    cv::Mat image;
    lccv::PiCamera cam;
    //cam.options->width=4056;
    //cam.options->height=3040;
    cam.options->photo_width=2028;
    cam.options->photo_height=1520;
    cam.options->verbose=true;
    cv::namedWindow("Image",cv::WINDOW_NORMAL);
    for(int i=0;i<100;i++){
        std::cout<<i<<std::endl;
        if(!cam.capturePhoto(image)){
            std::cout<<"Camera error"<<std::endl;
        }
        cv::imshow("Image",image);
        cv::waitKey(30);
    }
    cv::waitKey();
    cv::destroyWindow("Image");
}
