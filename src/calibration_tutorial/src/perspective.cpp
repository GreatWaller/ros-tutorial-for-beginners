#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

void perspectiveCorrection(const string &img1Path, const string &img2Path, const Size &patternSize)
{
    Mat img1 = imread( samples::findFile(img1Path) );
    Mat img2 = imread( samples::findFile(img2Path) );

    //! [find-corners]
    vector<Point2f> corners1, corners2;
    bool found1 = findChessboardCorners(img1, patternSize, corners1);
    bool found2 = findChessboardCorners(img2, patternSize, corners2);
    //! [find-corners]

    if (!found1 || !found2)
    {
        cout << "Error, cannot find the chessboard corners in both images." << endl;
        return;
    }

    //! [estimate-homography]
    Mat H = findHomography(corners1, corners2);
    cout << "H:\n" << H << endl;
    //! [estimate-homography]

    //! [warp-chessboard]
    Mat img1_warp;
    warpPerspective(img1, img1_warp, H, img1.size());
    //! [warp-chessboard]

    Mat img_draw_warp;
    hconcat(img1_warp, img2, img_draw_warp);
    // imshow("Desired chessboard view / Warped source chessboard view", img_draw_warp);
    imwrite("test/calibration/perspective2.jpg",img_draw_warp);
}

const char* params
    = "{ help h         |       | print usage }"
      "{ image1         | left02.jpg | path to the source chessboard image }"
      "{ image2         | left01.jpg | path to the desired chessboard image }"
      "{ width bw       | 9     | chessboard width }"
      "{ height bh      | 6     | chessboard height }";


int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, params);

    if (parser.has("help"))
    {
        parser.about("Code for homography tutorial.\n"
            "Example 2: perspective correction.\n");
        parser.printMessage();
        return 0;
    }

    Size patternSize(parser.get<int>("width"), parser.get<int>("height"));
    perspectiveCorrection(parser.get<String>("image1"),
                          parser.get<String>("image2"),
                          patternSize);

    return 0;
}

// rosrun calibration_tutorial calibration_tutorial_perspective_node --image1=test/calibration/images/left02.jpg --image2=test/calibration/images/left01.jpg