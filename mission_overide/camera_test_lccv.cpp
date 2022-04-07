#pragma once
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <pthread.h>





#include <sys/mman.h>

#include <condition_variable>
#include <iostream>
#include <memory>

#include <queue>
#include <set>
#include <string>
#include <thread>
#include <variant>
#include <any>
#include <map>
#include <iomanip>


#include <libcamera>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>


#include <fstream>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/transform.h>
using namespace std;

enum Exposure_Modes {
    EXPOSURE_NORMAL = libcamera::controls::ExposureNormal,
    EXPOSURE_SHORT = libcamera::controls::ExposureShort,
    EXPOSURE_CUSTOM = libcamera::controls::ExposureCustom
};

enum Metering_Modes {
    METERING_CENTRE = libcamera::controls::MeteringCentreWeighted,
    METERING_SPOT = libcamera::controls::MeteringSpot,
    METERING_MATRIX = libcamera::controls::MeteringMatrix,
    METERING_CUSTOM = libcamera::controls::MeteringCustom
};

enum WhiteBalance_Modes {
    WB_AUTO = libcamera::controls::AwbAuto,
    WB_NORMAL = libcamera::controls::AwbAuto,
    WB_INCANDESCENT = libcamera::controls::AwbIncandescent,
    WB_TUNGSTEN = libcamera::controls::AwbTungsten,
    WB_FLUORESCENT = libcamera::controls::AwbFluorescent,
    WB_INDOOR = libcamera::controls::AwbIndoor,
    WB_DAYLIGHT = libcamera::controls::AwbDaylight,
    WB_CLOUDY = libcamera::controls::AwbCloudy,
    WB_CUSTOM = libcamera::controls::AwbAuto
};

class Options
{
public:
	Options()
	{
        timeout=1000;
        metering_index = Metering_Modes::METERING_CENTRE;
        exposure_index=Exposure_Modes::EXPOSURE_NORMAL;
        awb_index=WhiteBalance_Modes::WB_AUTO;
        saturation=1.0f;
        contrast=1.0f;
        sharpness=1.0f;
	brightness=0.0f;
	shutter=0.0f;
	gain=0.0f;
	ev=0.0f;
	roi_x=roi_y=roi_width=roi_height=0;
	awb_gain_r=awb_gain_b=0;
        denoise="auto";
        verbose=false;
	transform=libcamera::Transform::Identity;
	camera=0;
	}

	virtual ~Options() {}

	virtual void Print() const;

    void setMetering(Metering_Modes meteringmode){metering_index=meteringmode;}
    void setWhiteBalance(WhiteBalance_Modes wb){awb_index = wb;}
    void setExposureMode(Exposure_Modes exp){exposure_index = exp;}

    int getExposureMode(){return exposure_index;}
    int getMeteringMode(){return metering_index;}
    int getWhiteBalance(){return awb_index;}

	bool help;
	bool version;
	bool list_cameras;
	bool verbose;
	uint64_t timeout; // in ms
    unsigned int photo_width, photo_height;
    unsigned int video_width, video_height;
	bool rawfull;
	libcamera::Transform transform;
	float roi_x, roi_y, roi_width, roi_height;
	float shutter;
	float gain;
	float ev;
	float awb_gain_r;
	float awb_gain_b;
	float brightness;
	float contrast;
	float saturation;
	float sharpness;
	float framerate;
	std::string denoise;
	std::string info_text;
	unsigned int camera;

protected:
	int metering_index;
	int exposure_index;
    int awb_index;

private:
};

class Options;
struct CompletedRequest;
using CompletedRequestPtr = std::shared_ptr<CompletedRequest>;

namespace controls = libcamera::controls;
namespace properties = libcamera::properties;

class LibcameraApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;
	using ControlList = libcamera::ControlList;
	using Request = libcamera::Request;
	using CameraManager = libcamera::CameraManager;
	using Camera = libcamera::Camera;
	using CameraConfiguration = libcamera::CameraConfiguration;
	using FrameBufferAllocator = libcamera::FrameBufferAllocator;
	using StreamRole = libcamera::StreamRole;
	using StreamRoles = libcamera::StreamRoles;
	using PixelFormat = libcamera::PixelFormat;
	using StreamConfiguration = libcamera::StreamConfiguration;
	using BufferMap = Request::BufferMap;
	using Size = libcamera::Size;
	using Rectangle = libcamera::Rectangle;
	enum class MsgType
	{
		RequestComplete,
		Quit
	};
	typedef std::variant<CompletedRequestPtr> MsgPayload;
	struct Msg
	{
		Msg(MsgType const &t) : type(t) {}
		template <typename T>
		Msg(MsgType const &t, T p) : type(t), payload(std::forward<T>(p))
		{
		}
		MsgType type;
		MsgPayload payload;
	};

	// Some flags that can be used to give hints to the camera configuration.
	static constexpr unsigned int FLAG_STILL_NONE = 0;
	static constexpr unsigned int FLAG_STILL_BGR = 1; // supply BGR images, not YUV
	static constexpr unsigned int FLAG_STILL_RGB = 2; // supply RGB images, not YUV
	static constexpr unsigned int FLAG_STILL_RAW = 4; // request raw image stream
	static constexpr unsigned int FLAG_STILL_DOUBLE_BUFFER = 8; // double-buffer stream
	static constexpr unsigned int FLAG_STILL_TRIPLE_BUFFER = 16; // triple-buffer stream
	static constexpr unsigned int FLAG_STILL_BUFFER_MASK = 24; // mask for buffer flags

	static constexpr unsigned int FLAG_VIDEO_NONE = 0;
	static constexpr unsigned int FLAG_VIDEO_RAW = 1; // request raw image stream
	static constexpr unsigned int FLAG_VIDEO_JPEG_COLOURSPACE = 2; // force JPEG colour space

	LibcameraApp(std::unique_ptr<Options> const opts = nullptr);
	virtual ~LibcameraApp();

	Options *GetOptions() const { return options_.get(); }

	std::string const &CameraId() const;
	void OpenCamera();
	void CloseCamera();

	void ConfigureStill(unsigned int flags = FLAG_STILL_NONE);
    void ConfigureViewfinder();

	void Teardown();
	void StartCamera();
	void StopCamera();

	Msg Wait();
	void PostMessage(MsgType &t, MsgPayload &p);

	Stream *GetStream(std::string const &name, unsigned int *w = nullptr, unsigned int *h = nullptr,
					  unsigned int *stride = nullptr) const;
	Stream *ViewfinderStream(unsigned int *w = nullptr, unsigned int *h = nullptr,
							 unsigned int *stride = nullptr) const;
	Stream *StillStream(unsigned int *w = nullptr, unsigned int *h = nullptr, unsigned int *stride = nullptr) const;
	Stream *RawStream(unsigned int *w = nullptr, unsigned int *h = nullptr, unsigned int *stride = nullptr) const;
	Stream *VideoStream(unsigned int *w = nullptr, unsigned int *h = nullptr, unsigned int *stride = nullptr) const;
	Stream *LoresStream(unsigned int *w = nullptr, unsigned int *h = nullptr, unsigned int *stride = nullptr) const;
	Stream *GetMainStream() const;

	std::vector<libcamera::Span<uint8_t>> Mmap(FrameBuffer *buffer) const;

	void SetControls(ControlList &controls);
	void StreamDimensions(Stream const *stream, unsigned int *w, unsigned int *h, unsigned int *stride) const;

protected:
	std::unique_ptr<Options> options_;

private:
	template <typename T>
	class MessageQueue
	{
	public:
		template <typename U>
		void Post(U &&msg)
		{
			std::unique_lock<std::mutex> lock(mutex_);
			queue_.push(std::forward<U>(msg));
			cond_.notify_one();
		}
		T Wait()
		{
			std::unique_lock<std::mutex> lock(mutex_);
			cond_.wait(lock, [this] { return !queue_.empty(); });
			T msg = std::move(queue_.front());
			queue_.pop();
			return msg;
		}
		void Clear()
		{
			std::unique_lock<std::mutex> lock(mutex_);
			queue_ = {};
		}

	private:
		std::queue<T> queue_;
		std::mutex mutex_;
		std::condition_variable cond_;
	};

	void setupCapture();
	void makeRequests();
	void queueRequest(CompletedRequest *completed_request);
	void requestComplete(Request *request);
	void configureDenoise(const std::string &denoise_mode);

	std::unique_ptr<CameraManager> camera_manager_;
	std::shared_ptr<Camera> camera_;
	bool camera_acquired_ = false;
	std::unique_ptr<CameraConfiguration> configuration_;
	std::map<FrameBuffer *, std::vector<libcamera::Span<uint8_t>>> mapped_buffers_;
	std::map<std::string, Stream *> streams_;
	FrameBufferAllocator *allocator_ = nullptr;
	std::map<Stream *, std::queue<FrameBuffer *>> frame_buffers_;
	std::queue<Request *> free_requests_;
	std::vector<std::unique_ptr<Request>> requests_;
	std::mutex completed_requests_mutex_;
	std::set<CompletedRequest *> completed_requests_;
	bool camera_started_ = false;
	std::mutex camera_stop_mutex_;
	MessageQueue<Msg> msg_queue_;
	// For setting camera controls.
	std::mutex control_mutex_;
	ControlList controls_;
	// Other:
	uint64_t last_timestamp_;
	uint64_t sequence_ = 0;
};

struct FrameInfo
{
	FrameInfo(libcamera::ControlList &ctrls)
		: exposure_time(0.0), digital_gain(0.0), colour_gains({ { 0.0f, 0.0f } }), focus(0.0), aelock(false)
	{
		if (ctrls.contains(libcamera::controls::ExposureTime))
			exposure_time = ctrls.get<int32_t>(libcamera::controls::ExposureTime);

		if (ctrls.contains(libcamera::controls::AnalogueGain))
			analogue_gain = ctrls.get(libcamera::controls::AnalogueGain);

		if (ctrls.contains(libcamera::controls::DigitalGain))
			digital_gain = ctrls.get(libcamera::controls::DigitalGain);

		if (ctrls.contains(libcamera::controls::ColourGains))
		{
			libcamera::Span<const float> gains = ctrls.get(libcamera::controls::ColourGains);
			colour_gains[0] = gains[0], colour_gains[1] = gains[1];
		}

		if (ctrls.contains(libcamera::controls::FocusFoM))
			focus = ctrls.get(libcamera::controls::FocusFoM);

		if (ctrls.contains(libcamera::controls::AeLocked))
			aelock = ctrls.get(libcamera::controls::AeLocked);
	}

	std::string ToString(std::string &info_string) const
	{
		std::string parsed(info_string);

		for (auto const &t : tokens)
		{
			std::size_t pos = parsed.find(t);
			if (pos != std::string::npos)
			{
				std::stringstream value;
				value << std::fixed << std::setprecision(2);

				if (t == "%frame")
					value << sequence;
				else if (t == "%fps")
					value << fps;
				else if (t == "%exp")
					value << exposure_time;
				else if (t == "%ag")
					value << analogue_gain;
				else if (t == "%dg")
					value << digital_gain;
				else if (t == "%rg")
					value << colour_gains[0];
				else if (t == "%bg")
					value << colour_gains[1];
				else if (t == "%focus")
					value << focus;
				else if (t == "%aelock")
					value << aelock;

				parsed.replace(pos, t.length(), value.str());
			}
		}

		return parsed;
	}

	unsigned int sequence;
	float exposure_time;
	float analogue_gain;
	float digital_gain;
	std::array<float, 2> colour_gains;
	float focus;
	float fps;
	bool aelock;

private:
	// Info text tokens.
	inline static const std::string tokens[] = { "%frame", "%fps", "%exp",	 "%ag",	   "%dg",
												 "%rg",	   "%bg",  "%focus", "%aelock" };
};

class Metadata
{
public:
	Metadata() = default;

	Metadata(Metadata const &other)
	{
		std::scoped_lock other_lock(other.mutex_);
		data_ = other.data_;
	}

	Metadata(Metadata &&other)
	{
		std::scoped_lock other_lock(other.mutex_);
		data_ = std::move(other.data_);
		other.data_.clear();
	}

	template <typename T>
	void Set(std::string const &tag, T &&value)
	{
		std::scoped_lock lock(mutex_);
		data_.insert_or_assign(tag, std::forward<T>(value));
	}

	template <typename T>
	int Get(std::string const &tag, T &value) const
	{
		std::scoped_lock lock(mutex_);
		auto it = data_.find(tag);
		if (it == data_.end())
			return -1;
		value = std::any_cast<T>(it->second);
		return 0;
	}

	void Clear()
	{
		std::scoped_lock lock(mutex_);
		data_.clear();
	}

	Metadata &operator=(Metadata const &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_ = other.data_;
		return *this;
	}

	Metadata &operator=(Metadata &&other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_ = std::move(other.data_);
		other.data_.clear();
		return *this;
	}

	void Merge(Metadata &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_.merge(other.data_);
	}

	template <typename T>
	T *GetLocked(std::string const &tag)
	{
		// This allows in-place access to the Metadata contents,
		// for which you should be holding the lock.
		auto it = data_.find(tag);
		if (it == data_.end())
			return nullptr;
		return std::any_cast<T>(&it->second);
	}

	template <typename T>
	void SetLocked(std::string const &tag, T &&value)
	{
		// Use this only if you're holding the lock yourself.
		data_.insert_or_assign(tag, std::forward<T>(value));
	}

	// Note: use of (lowercase) lock and unlock means you can create scoped
	// locks with the standard lock classes.
	// e.g. std::lock_guard<RPiController::Metadata> lock(metadata)
	void lock() { mutex_.lock(); }
	void unlock() { mutex_.unlock(); }

private:
	mutable std::mutex mutex_;
	std::map<std::string, std::any> data_;
};

struct CompletedRequest
{
	using BufferMap = libcamera::Request::BufferMap;
	using ControlList = libcamera::ControlList;
	using Request = libcamera::Request;

	CompletedRequest(unsigned int seq, Request *r)
		: sequence(seq), buffers(r->buffers()), metadata(r->metadata()), request(r)
	{
		r->reuse();
	}
	unsigned int sequence;
	BufferMap buffers;
	ControlList metadata;
	Request *request;
	float framerate;
	Metadata post_process_metadata;
};
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
