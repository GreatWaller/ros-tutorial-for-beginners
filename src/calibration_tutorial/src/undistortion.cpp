#include <opencv2/opencv.hpp>

int main()
{
    const cv::Mat M = (cv::Mat_<double>(3, 3) << 5.3398795245975896e+02, 0., 3.2838647449406972e+02,
                       0., 5.2871082110006125e+02, 2.3684272831168110e+02,
                       0., 0., 1.);
    const cv::Mat D = (cv::Mat_<double>(5, 1) << -2.5896599091487332e-01, -1.2618381088945435e-01, 0., 0., 0.);

    const int ImgWidth = 640;
    const int ImgHeight = 480;

    cv::Mat map1, map2;
    cv::Size imageSize(ImgWidth, ImgHeight);
    // const double alpha = 1;
    // cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0);

    cv::initUndistortRectifyMap(M, D, cv::Mat(), M, imageSize, CV_16SC2, map1, map2);

    std::string InputPath = "test/projection/left01.jpg";
    cv::Mat RawImage = cv::imread(InputPath);
    // cv::imshow("RawImage", RawImage);

    cv::Mat UndistortImage;
    remap(RawImage, UndistortImage, map1, map2, cv::INTER_LINEAR);

    // std::string OutputPath = "test/projection/left01_undistorted.jpg";
    // cv::imwrite(OutputPath, UndistortImage);

    cv::Mat img_concat;
    cv::hconcat(RawImage, UndistortImage, img_concat);
    cv::imwrite("test/calibration/left01_raw_undistorted.jpg",img_concat);

    return 0;
}