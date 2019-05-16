#include "cv.h"
#include "highgui.h"
#include "Control.h"
#include "TCPServer.h"

using namespace cv;

class Tracker 
{
private:
	Mat imvid;
	Mat imthresh;
    Control *MCU;
	TCPServer Server;
public:
	Mat hsvFilter(const Mat &src, int hueLow, int hueHigh);
	Tracker();
	~Tracker();
	void Update();
	void Draw();
};