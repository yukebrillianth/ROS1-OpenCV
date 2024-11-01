#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/TeleportAbsolute.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <math.h>
#include <chrono>
#include <ctime>
#include <ratio>

using namespace cv;
using namespace std;
using namespace std::chrono;

double calculateDistance(Point2f p1, Point2f p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

void moveRobot(ros::NodeHandle &nh, const string &name, float x, float y)
{
    ros::ServiceClient moveClient = nh.serviceClient<turtlesim::TeleportAbsolute>("/" + name + "/teleport_absolute", 1);
    turtlesim::TeleportAbsolute teleport_srv;
    teleport_srv.request.x = x;
    teleport_srv.request.y = y;
    teleport_srv.request.theta = 0;

    moveClient.call(teleport_srv);
}

void killTurtle(ros::NodeHandle &nh, const string &name)
{
    ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("kill", 1);
    turtlesim::Kill kill_srv;
    kill_srv.request.name = name;

    if (kill_client.call(kill_srv))
    {
        cout << "Killed turtle: " << name.c_str() << endl;
    }
    else
    {
        cout << "Failed to kill turtle: " << name.c_str() << endl;
    }
}

void spawnTurtle(ros::NodeHandle &nh, const string &name, float x, float y)
{
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn", 1);

    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = x;
    spawn_srv.request.y = y;
    spawn_srv.request.theta = 0;
    spawn_srv.request.name = name;

    if (spawn_client.call(spawn_srv))
    {
        cout << "Spawned turtle: " << spawn_srv.response.name.c_str() << endl;
    }
    else
    {
        cout << "Failed to spawn turtle: " << spawn_srv.response.name.c_str() << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_pub");
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher pub_data = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // Init opencv
    VideoCapture cap("/home/yukebrillianth/KULIAH/ROBOTIK/MAGANG/WEEK-3/ROS_OPENCV/cam.mp4");
    if (!cap.isOpened())
    {
        cout << "Unable to access video source!" << endl;
        return -1;
    }

    Mat frame, frame_hsv, frame_thres;

    /**
     * @brief Config Trackbar
     *
     */
    int h[2] = {100, 118}, s[2] = {155, 255}, v[2] = {117, 255};
    namedWindow("Control");

    createTrackbar("Hmin", "Control", &h[0], 255);
    createTrackbar("Smin", "Control", &s[0], 255);
    createTrackbar("Vmin", "Control", &v[0], 255);

    createTrackbar("Hmax", "Control", &h[1], 255);
    createTrackbar("Smax", "Control", &s[1], 255);
    createTrackbar("Vmax", "Control", &v[1], 255);

    Point2f startBallPosition(0, 0);
    double lastDistance = 0;

    // Wait conn (untuk launcher)
    ros::Duration(2).sleep();
    killTurtle(nh, "turtle1");

    bool isBallSpawned = false;
    bool isRobotSpawned = false;

    // Init time awal
    high_resolution_clock::time_point lastTime = high_resolution_clock::now();

    while (true)
    {
        cap >> frame;

        // Handle if video reached end
        if (frame.empty())
        {
            break;
        }

        // Convert RGB to HSV
        cvtColor(frame, frame_hsv, COLOR_RGB2HSV);
        inRange(frame_hsv, Scalar(h[0], s[0], v[0]), Scalar(h[1], s[1], v[1]), frame_thres);

        vector<vector<Point>> contours;
        findContours(frame_thres, contours, RETR_TREE, CHAIN_APPROX_NONE);

        if (!contours.empty())
        {
            // Get Largest Contour Area
            int largestContourAreaI = 0;
            double largestArea = contourArea(contours[0]);

            // Get largest contour area
            for (size_t i = 0; i < contours.size(); i++)
            {
                double area = contourArea(contours[i]);
                cout << "Area: " << area << endl;

                if (area > largestArea && area < 4000.0)
                {
                    largestArea = area;
                    largestContourAreaI = i;
                }
            }
            // Get center of robot
            Point2f centerOfRobot(frame.cols / 2, (frame.rows / 2) - 20);
            circle(frame, centerOfRobot, 4, Scalar(0, 0, 255), -1);

            Moments m = moments(contours[largestContourAreaI]);
            double mArea = m.m00;
            if (mArea != 0)
            {
                double mX = m.m10;
                double mY = m.m01;

                // Posisi bola
                Point2f ballPosition(mX / mArea, mY / mArea);
                cout << startBallPosition.x << ", " << startBallPosition.y << endl;
                circle(frame, ballPosition, 3, Scalar(0, 0, 255), -1);

                Point2f circleLine;
                float BallRadius;

                // Hitung min enclosing circle
                minEnclosingCircle(contours[largestContourAreaI], circleLine, BallRadius);
                circle(frame, circleLine, (int)BallRadius, Scalar(0, 255, 255), 2);

                // cout << BallRadius << endl;

                // Calculate distance
                double distance = calculateDistance(ballPosition, centerOfRobot);

                // Init start ball pos
                if (startBallPosition.x == 0 && startBallPosition.y == 0)
                {
                    startBallPosition = ballPosition;
                    lastDistance = distance;
                }

                // Spawn ball
                if (!isBallSpawned)
                {
                    spawnTurtle(nh, "ball", 1 + (startBallPosition.x - (frame.cols / 2)) / 100, 1 + ((frame.rows / 2) - startBallPosition.y) / 100);
                    isBallSpawned = true;
                }

                // Get robot pos
                double robotX = cvRound(startBallPosition.x - ballPosition.x);
                double robotY = cvRound(ballPosition.y - startBallPosition.y);
                line(frame, centerOfRobot, ballPosition, Scalar(255, 0, 0));

                // Calculate speed
                high_resolution_clock::time_point tNow = high_resolution_clock::now();
                duration<double> time_span = duration_cast<duration<double>>(tNow - lastTime);
                lastTime = tNow;
                float speed = abs(((lastDistance - distance) * 10) / time_span.count());
                lastDistance = distance;

                // Tampilkan informasi pada frame
                string posInfo = "Robot Position: (" + to_string(int(robotX / 10)) + "cm, " + to_string(int(robotY / 10)) + " cm)";
                string robotInfo = "Position: (" + to_string(int(robotX / 10)) + ", " + to_string(int(robotY / 10)) + ")";
                string ballPosInfo = "Ball Position: (" + to_string(int(startBallPosition.x / 10)) + "cm, " + to_string(int(startBallPosition.y / 10)) + " cm)";
                string distanceInfo = "Distance: " + to_string(int(distance * 10)) + " cm";
                string speedInfo = "Speed: " + to_string((int)speed) + " cm/s";
                cout << ballPosInfo << endl;
                cout << posInfo << endl;

                putText(frame, robotInfo, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                // putText(frame, posInfo, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                // putText(frame, ballPosInfo, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                putText(frame, distanceInfo, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                putText(frame, speedInfo, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                // lastDistance

                // Spawn robot
                if (!isRobotSpawned)
                {
                    spawnTurtle(nh, "robot", 1, 1);
                    isRobotSpawned = true;
                }

                // Move
                moveRobot(nh, "robot", 1 + (robotX / 100), 1 + (robotY / 100));
            }
        }

        // Show thresholded image
        imshow("Threshold", frame_thres);

        // Show original image
        imshow("Camera", frame);

        if (waitKey(24) == 'q')
        {
            break;
        }
    }

    return 0;
}