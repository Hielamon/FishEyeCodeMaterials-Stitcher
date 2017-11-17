#include <string>

#if (defined WIN32 || defined _WIN32)
#if defined(FISHEYESTITCHER_EXPORTS) || defined(FishEyeStitcher_EXPORTS)
#define FISHSTITCHER_API __declspec(dllexport)
#else 
#define FISHSTITCHER_API __declspec(dllimport)
#endif // FISHEYESTITCHER_EXPORTS
#elif defined __GNUC__ && __GNUC__ >= 4
#define FISHSTITCHER_API __attribute__ ((visibility ("default")))
#endif

//Execute the Fish Eye Images Stitching algorithm
//return 0,if stitching successfully.     
//return 1,if result_h is not possible(0<result_h<100000)
//return 2,if result name is not possible(support: jpb,png,tiff)                                         
//return 3,if cannot open the input folder(dir).                                   
//return 4,if cannot find enough(>=2) images with special image_suffix.            
//return 5,if cannot open enough(>=2) images.                                     
//return 6,for doing hdr, but multiples of hdr_number is not the number of images. 
//return 7,Failed to write the result. 
//return 8,there some errors in the core stitching algorithm.                      
int FISHSTITCHER_API FishStitcherMain(const std::string &dir, const std::string &name, const std::string &image_suffix, 
					  int result_h, bool do_fine_tune, bool do_hdr, int hdr_number);
