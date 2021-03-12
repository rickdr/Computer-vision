#include <string>
namespace nl_uu_science_gmt
{
	class camera_calibration
	{
	public:
		camera_calibration();
		void calibration(int argc, char* argv[], std::string video_file, int camera_amount);
		int calibrate(int argc, char* argv[], std::string video_path);
		static void color_model(std::string video_file, int camera_amount);
	};
}