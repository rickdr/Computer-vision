#include <string>
namespace nl_uu_science_gmt
{
	class camera_calibration
	{
	public:
		camera_calibration(int argc, char* argv[], std::string video_file, int camera_amount);
		int calibration(int argc, char* argv[], std::string video_file);
	};
}