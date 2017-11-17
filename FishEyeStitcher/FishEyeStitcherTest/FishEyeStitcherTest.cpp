#include "../FishEyeStitcher/include/FishStitcherMain.h"
#include <iostream>

//std::string dir = "test1";
std::string dir = "D:/Academic-Research/Datas/ImageStitchingDatas/Hotel/test5";
std::string name = "result.jpg";
int result_h = 4000;
bool do_hdr = !true, do_fine_tune =true;
int hdr_number = 3;
std::string image_suffix = "jpg";

void PrintUsage()
{
    std::cout << "Usage(cmd)  :  FishEyeStitcherTest.exe  + [flags]" << std::endl;
    std::cout << "               Flags:" << std::endl;
    std::cout << "               -work_dir work_path" << std::endl;
    std::cout << "                         Use work_path as the work directory, Default set to test1" << std::endl;
    std::cout << "               -height height_value" << std::endl;
    std::cout << "                         Set the image height of result. Default set to 4000" << std::endl;
    std::cout << "               -hdr (yes|no)" << std::endl;
    std::cout << "                         Set  whether to HDR processing, Default set to no" << std::endl;
    std::cout << "               -hdr_num hdr_num" << std::endl;
    std::cout << "                         Set the number of a group images for HDR processing, Default set to 3" << std::endl;
    std::cout << "               -name name_value" << std::endl;
    std::cout << "                         Set the result name, Default set to 'result.jpg'" << std::endl;
	std::cout << "               -fine_tune (yes|no)" << std::endl;
	std::cout << "                         Set whether fine-tuning the misalignments, Default set to yes" << std::endl;
}
 
int parseCmdArgs(int argc, char** argv)
{
    if (argc == 1)
    {
        std::cout << "Use default setting: work_dir    = test1" << std::endl;
        std::cout << "*******************: height      = 4000" << std::endl;
        std::cout << "*******************: hdr         = no  " << std::endl;
        std::cout << "*******************: hdr_num     = 3  " << std::endl;
        std::cout << "*******************: name        = 'result.jpg'  " << std::endl;
        std::cout << "Moreover, you can use then command \"FishEyeStitcherTest.exe -help\" to get Usage" << std::endl;
    }
 
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "-help" || std::string(argv[i]) == "/?")
        {
            PrintUsage();
            return 0;
        }
        else if (std::string(argv[i]) == "-work_dir")
        {
            dir = std::string(argv[i + 1]);
            i++;
        }
        else if (std::string(argv[i]) == "-height")
        {
            int height = atof(argv[i + 1]);
            result_h = height;
            i++;
        }
        else if (std::string(argv[i]) == "-hdr")
        {
            if (std::string(argv[i + 1]) == "yes")
                do_hdr = true;
            else if (std::string(argv[i + 1]) == "no")
                do_hdr = false;
            i++;
        }
        else if (std::string(argv[i]) == "-hdr_num")
        {
            hdr_number = atof(argv[i + 1]);
            i++;
        }
        else if (std::string(argv[i]) == "-name")
        {
            //TODO Check the result name which must be the image name
            name = std::string(argv[i + 1]);
            i++;
        }
		else if (std::string(argv[i]) == "-fine_tune")
		{
			if (std::string(argv[i + 1]) == "yes")
				do_fine_tune = true;
			else if (std::string(argv[i + 1]) == "no")
				do_fine_tune = false;
			i++;
		}
		else
		{
			return 0;
		}
    }
 
    return 1;
}

int main(int argc, char *argv[])
{
	 //PrintUsage();
    int retval = parseCmdArgs(argc, argv);
    if (!retval)
    {
		PrintUsage();
        return -1;
    }
	int result = FishStitcherMain(dir, name, image_suffix, result_h, do_fine_tune, do_hdr, hdr_number);
	std::cout << "result num is : " << result << std::endl;
	return 1;
}
