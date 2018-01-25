#include <FishStitcherMain.h>
#include <FishStitcher.h>
#include <TraverFolder.h>
#include <HDRMerger.h>
#include <fstream>
#include <sstream>


int FishStitcherMain(const std::string &dir, const std::string &name, const std::string &image_suffix, 
					  int result_h, bool do_fine_tune, bool do_hdr, int hdr_number)
{

#if COST_TIME
	double duration;
	duration = static_cast<double>(cv::getTickCount());
	double total_time;
	total_time = static_cast<double>(cv::getTickCount());
#endif


	if (result_h <= 0 || result_h >= 100000)
	{
		//std::cout << "result_h is not possible(0<result_h<100000)" << std::endl;
		return 1;
	}

	std::string legalNames[6] = { ".jpg",".JPG",".png",".PNG",".tiff", ".TIFF" };
	bool is_legal = false;
	for (size_t i = 0; i < 6; i++)
	{
		if (name.find(legalNames[i]) != std::string::npos)
		{
			is_legal = true;
			break;
		}
	}
	if (!is_legal)
	{
		//std::cout << "result name is not possible(support: jpb,png,tiff)" << std::endl;
		return 2;
	}

	std::vector<std::string> filelist, templist;
	{
		TraverFolder traverfolder;
		traverfolder.setFolderPath(dir);
		if (!traverfolder.IsFind())
		{
			//std::cout << "cannot open the folder " << dir << std::endl;
			return 3;
		}
		traverfolder.getFileFullPath(templist, image_suffix);
		for (size_t i = 0; i < templist.size(); i++)
		{
			if (templist[i].find(image_suffix) != std::string::npos)
				filelist.push_back(templist[i]);
		}
		if (filelist.size() < 2)
		{
			//std::cerr << "cannot find enough(>=2) images with special image_suffix" << std::endl;
			return 4;
		}
	}
	cv::Size fish_image_size;
	std::vector<cv::Mat> images;

	//º”‘ÿÕº∆¨
	int num_images = 0;
	std::cout << "\n.............Start to load images............." << std::endl;
	for (size_t i = 0; i < filelist.size(); i++, num_images++)
	{
		cv::Mat temp_image = cv::imread(filelist[i]);
		if (i != 0 && temp_image.size() != fish_image_size)
			break;
		if (i == 0)fish_image_size = temp_image.size();
		images.push_back(temp_image);

	}

	if (num_images < 2)
	{
		//std::cout << "cannot open enough(>=2) images" << std::endl;
		return 5;
	}

#if COST_TIME
	duration = static_cast<double>(cv::getTickCount()) - duration;
	std::cout << "LoadImage cost time : " << 1000 * (duration / cv::getTickFrequency()) << " ms " << std::endl;
	duration = static_cast<double>(cv::getTickCount());
#endif

	if (do_hdr)
	{
		std::cout << "\n.............Start to HDR images............." << std::endl;
		if (images.size() % hdr_number != 0)
		{
			//std::cout << "for doing hdr, but multiples of hdr_number is not the number of images" << std::endl;
			return 6;
		}
		else
		{
			size_t num_arr = images.size() / hdr_number;

			std::vector<std::vector<cv::Mat> > src(num_arr);
			std::vector<cv::Mat> merged_imgs;
			for (size_t i = 0, idx = 0; i < num_arr; i++, idx += hdr_number)
				src[i] = std::vector<cv::Mat>(images.begin() + idx, images.begin() + idx + hdr_number);
			
			HDRMerger hdrmerger;
			if(!hdrmerger(src, merged_imgs))return -1;
			images.clear();
			images = merged_imgs;
		}

#if COST_TIME
		duration = static_cast<double>(cv::getTickCount()) - duration;
		std::cout << "HDRMerge cost time : " << 1000 * (duration / cv::getTickFrequency()) << " ms " << std::endl;
		duration = static_cast<double>(cv::getTickCount());
#endif
	}
	CircleFish::FishStitcher fishstitcher(result_h);
	cv::Mat result;

	if (fishstitcher(images, do_fine_tune, dir, result))
	{
		size_t pos = dir.rfind('\\');
		std::string final_name;
		if (pos == std::string::npos)
			final_name = dir;
		else
			final_name = dir.substr(pos, dir.size());

		std::string result_name = dir + "/" + name;
		std::cout << "\nwrite the result to :" << result_name << std::endl;
		if (!cv::imwrite(result_name, result))
		{
			//std::cout << "Failed to write the result" << std::endl;
			return 7;
		}
	}
	else
	{
		//std::cout << "there some errors in the core stitching algorithm" << std::endl;
		return 8;
	}


#if COST_TIME
	total_time = static_cast<double>(cv::getTickCount()) - total_time;
	std::cout << "All processing cost time : " << 1000 * (total_time / cv::getTickFrequency()) << " ms " << std::endl;
#endif

	return 0;
}
